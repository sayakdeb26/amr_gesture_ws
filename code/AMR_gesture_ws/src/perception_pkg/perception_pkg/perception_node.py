from .vlm_transport_ros import VLMClientROS
#!/usr/bin/env python3
import os
import json
import time
import subprocess
from typing import Optional, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time

import requests

from amr_interfaces.msg import (
    UnknownGesture,
    Intent,
    ConfirmRequest,
    ConfirmReply,
    TrainingExample,
)

def now_stamp(node: Node) -> Time:
    return node.get_clock().now().to_msg()

def _probe_clip(path: str) -> Tuple[float, int]:
    """Return (fps, frames). Uses ffprobe if present; else (0.0, 0)."""
    try:
        out = subprocess.check_output([
            'ffprobe', '-v', 'error',
            '-select_streams', 'v:0',
            '-show_entries', 'stream=avg_frame_rate,nb_frames',
            '-of', 'json', path
        ], stderr=subprocess.STDOUT, text=True, timeout=3)
        data = json.loads(out)
        streams = data.get('streams', [])
        if not streams:
            return 0.0, 0
        st = streams[0]
        fps = 0.0
        fr = st.get('avg_frame_rate') or '0/1'
        if isinstance(fr, str) and '/' in fr:
            num, den = fr.split('/', 1)
            try:
                num = float(num)
                den = float(den) if float(den) != 0 else 1.0
                fps = num / den
            except Exception:
                fps = 0.0
        frames = 0
        nb = st.get('nb_frames')
        if nb is not None and str(nb).isdigit():
            frames = int(nb)
        return float(fps), int(frames)
    except Exception:
        return 0.0, 0

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self._vlm_client = VLMClientROS(self)
        self._vlm_transport = 'ros'

        # Config
        self.vlm_url = os.environ.get('VLM_URL', 'http://127.0.0.1:8000')
        self.test_clip = os.environ.get('PERCEPTION_TEST_CLIP')  # absolute path to a test video
        self.confirm_timeout_s = float(os.environ.get('CONFIRM_TIMEOUT_S', '8'))
        self.auto_approve_on_timeout = os.environ.get('AUTO_APPROVE_ON_TIMEOUT', '0') == '1'

        self.get_logger().info(

            f'perception_node up. VLM_URL={self.vlm_url}, '
            f'test_clip={self.test_clip if self.test_clip else "<none>"}, '
            f'confirm_timeout={self.confirm_timeout_s:.1f}s, '
            f'auto_approve={self.auto_approve_on_timeout}'
        )
        if not self.test_clip:
            self.get_logger().warn('No PERCEPTION_TEST_CLIP set; live capture not implemented yet.')

        # Pub/Sub
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.pub_intents = self.create_publisher(Intent, '/intents_raw', qos)
        self.pub_confirm_req = self.create_publisher(ConfirmRequest, '/vlm/confirm_request', qos)
        self.pub_training = self.create_publisher(TrainingExample, '/lstm/training_example', qos)

        self.sub_unknown = self.create_subscription(
            UnknownGesture, '/lstm/unknown', self.on_unknown_gesture, qos
        )
        self.sub_confirm = self.create_subscription(
            ConfirmReply, '/ui/confirm_reply', self.on_confirm_reply, qos
        )

        # state for confirmation handshake
        self.pending: Optional[Dict] = None
        self.timeout_timer = None

        # quick health check
        try:
            r = requests.get(f'{self.vlm_url}/healthz', timeout=2)
            if r.status_code == 200:
                self.get_logger().info('VLM /healthz OK')
        except Exception as e:
            self.get_logger().warn(f'VLM /healthz failed: {e}')

    # ------------------ callbacks ------------------


    def on_unknown_gesture(self, msg):
        self.get_logger().info(f"🔥 on_unknown_gesture fired: src={getattr(msg, 'source', '')} hint={getattr(msg, 'hint', '')} conf={getattr(msg, 'confidence', 0.0):.2f} wf={getattr(msg, 'window_frames', 0)}")
        # Expect: low-confidence event → call ROS VLM service → ask UI to confirm

        clip_path = getattr(self, 'test_clip', '')  # current implementation uses a test clip file

        if not clip_path or not os.path.isfile(clip_path):

            self.get_logger().warn('No valid test clip; cannot call VLM.')

            return

        

        # Optional hint carried by message (if present)

        label_hint = getattr(msg, 'hint', '') if msg is not None else ''

        

        # Call ROS VLM service

        try:

            resp = self._vlm_client.infer(clip_path, label_hint)

            label = resp.get('label', 'UNKNOWN')

            conf = float(resp.get('confidence', 0.0))

            lat  = int(resp.get('latency_ms', 0))

            self.get_logger().info(f'VLM result: {label} conf={conf:.2f} lat={lat}ms')

        except Exception as e:

            self.get_logger().error(f'VLM call failed: {e}')

            return

        

        # Build and publish confirmation request to UI

        try:

            req = ConfirmRequest()

            req.stamp = now_stamp(self)

            # Field names vary across schemas — set what exists

            if hasattr(req, 'candidate_label'):

                req.candidate_label = label

            elif hasattr(req, 'label'):

                req.label = label

            if hasattr(req, 'candidate_conf'):

                req.candidate_conf = conf

            elif hasattr(req, 'confidence'):

                req.confidence = conf

            if hasattr(req, 'hint'):

                req.hint = label_hint

            if hasattr(req, 'source'):

                req.source = 'vlm-ros'

        

            self.pub_confirm_req.publish(req)

            self.get_logger().info(f'sent /vlm/confirm_request: label={label} conf={conf:.2f}')

        except Exception as e:

            self.get_logger().error(f'ConfirmRequest publish failed: {e}')

            return

    def on_confirm_timeout(self):
        if not self.pending:
            return
        if self.auto_approve_on_timeout:
            self.get_logger().warn('Timeout; auto-approving.')
            self.publish_intent_and_example(self.pending['label'], True, self.pending['clip_path'])
            self.get_logger().warn('No confirm reply within timeout; dropping.')
        # clear
        self.pending = None
        if self.timeout_timer:
            self.destroy_timer(self.timeout_timer)
            self.timeout_timer = None

    def on_confirm_reply(self, msg: ConfirmReply):
        if not self.pending:
            # stray reply
            return
        # stop timeout
        if self.timeout_timer:
            self.destroy_timer(self.timeout_timer)
            self.timeout_timer = None

        approved = bool(msg.approved)
        final_label = msg.final_label.strip() if msg.final_label else self.pending['label']
        self.publish_intent_and_example(final_label, approved, self.pending['clip_path'])

        # clear
        self.pending = None

    # ------------------ helpers ------------------

    def publish_intent_and_example(self, label: str, approved: bool, clip_path: str):
        # Always publish Intent when approved; drop otherwise
        if approved:
            it = Intent()
            it.stamp = now_stamp(self)
            it.label = label
            it.confidence = float(self.pending.get('conf', 0.0)) if self.pending else 0.0
            it.latency_ms = int(self.pending.get('lat_ms', 0)) if self.pending else 0
            it.source = 'vlm-http'
            self.pub_intents.publish(it)
            self.get_logger().info(f'Published /intents_raw (APPROVED): {label} ({it.confidence:.2f})')

            # Probe metadata for TrainingExample
            fps, frames = _probe_clip(clip_path)

            te = TrainingExample()
            te.stamp = it.stamp
            te.label = label
            te.clip_path = clip_path  # absolute; central_db_node will copy/symlink
            te.fps = float(fps)
            te.frames = int(frames)
            te.source = 'vlm-http'
            te.notes = ''  # optional free text; keep empty for now
            self.pub_training.publish(te)
            self.get_logger().info(f'Published /lstm/training_example: {label} -> {clip_path} (fps={fps:.2f}, frames={frames})')
            self.get_logger().info('User rejected candidate; nothing published.')

def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

