#!/usr/bin/env python3
import os
import time
from pathlib import Path
import requests

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

from amr_interfaces.msg import (
    UnknownGesture,   # /lstm/unknown
    ConfirmRequest,   # /vlm/confirm_request
    ConfirmReply,     # /ui/confirm_reply
    Intent,           # /intents_raw
    TrainingExample,  # /lstm/training_example  (T1.2)
)

class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('perception_node')

        # Env configuration (simple)
        self.vlm_url = os.getenv('VLM_URL', 'http://127.0.0.1:8000')
        self.test_clip = os.getenv('PERCEPTION_TEST_CLIP', '')
        self.confirm_timeout_s = float(os.getenv('CONFIRM_TIMEOUT_S', '8'))
        self.auto_approve_on_timeout = os.getenv('AUTO_APPROVE_ON_TIMEOUT', '1') in ('1', 'true', 'True')

        # Subs / pubs
        self.sub_unknown = self.create_subscription(
            UnknownGesture, '/lstm/unknown', self.on_unknown_gesture, 10
        )
        self.pub_confirm_req = self.create_publisher(ConfirmRequest, '/vlm/confirm_request', 10)
        self.sub_confirm_reply = self.create_subscription(
            ConfirmReply, '/ui/confirm_reply', self.on_confirm_reply, 10
        )
        self.pub_intents = self.create_publisher(Intent, '/intents_raw', 10)
        self.pub_training = self.create_publisher(TrainingExample, '/lstm/training_example', 10)

        self._pending = None  # holds (stamp, label, conf, clip_path)

        clip_note = self.test_clip if self.test_clip else '<none>'
        self.get_logger().info(
            f'perception_node up. VLM_URL={self.vlm_url}, test_clip={clip_note}, '
            f'confirm_timeout={self.confirm_timeout_s:.1f}s, auto_approve={self.auto_approve_on_timeout}'
        )
        if not self.test_clip:
            self.get_logger().warn('No PERCEPTION_TEST_CLIP set; live capture not implemented yet.')

    # --- Callbacks ---

    def on_unknown_gesture(self, msg: UnknownGesture) -> None:
        # Choose a clip path (for now we use a dummy/test file)
        clip_path = self.test_clip
        if not clip_path or not Path(clip_path).exists():
            self.get_logger().error(f'No test clip available at {clip_path!r}. Skipping VLM call.')
            return

        # Call VLM sidecar
        label, conf, lat_ms = self.call_vlm_infer_clip(clip_path)
        self.get_logger().info(f'VLM result: {label} conf={conf:.2f} lat={lat_ms}ms')

        # Ask UI to confirm
        req = ConfirmRequest()
        req.stamp = self.get_clock().now().to_msg()
        req.label = label
        req.confidence = float(conf)
#       req.source = 'vlm-http'
        self.pub_confirm_req.publish(req)

        # Remember pending request to match the reply
        self._pending = (req.stamp, label, float(conf), clip_path)

        # Arm a timeout timer
        self.create_timer(self.confirm_timeout_s, self._timeout_once)

    def on_confirm_reply(self, msg: ConfirmReply) -> None:
        if self._pending is None:
            return
        stamp, label, conf, clip = self._pending
        self._pending = None

        # Publish final intent (approved or rejected); for now we only publish when approved
        if msg.approved:
            final_label = msg.final_label if msg.final_label else label
            self.publish_intent(final_label, conf, source='vlm-http')
            # Also send a training example (T1.2)
            self.publish_training_example(final_label, clip)
            self.get_logger().info(f'Confirm reply: approved=True, final={final_label}')
        else:
            self.get_logger().info('Confirm reply: approved=False (dropped).')

    # --- Helpers ---

    def _timeout_once(self) -> None:
        # One-shot: if still pending after timeout
        if self._pending is None:
            return
        stamp, label, conf, clip = self._pending
        self._pending = None
        if self.auto_approve_on_timeout:
            self.publish_intent(label, conf, source='vlm-http')
            self.publish_training_example(label, clip)
            self.get_logger().warn('No confirm reply within timeout; AUTO-APPROVED.')
        else:
            self.get_logger().warn('No confirm reply within timeout; dropping.')

    def publish_intent(self, label: str, conf: float, source: str) -> None:
        msg = Intent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.label = label
        msg.confidence = float(conf)
        msg.latency_ms = 0
        msg.source = source
        self.pub_intents.publish(msg)
        self.get_logger().info(f'Published /intents_raw: {label} ({conf:.2f})')

    def publish_training_example(self, label: str, clip_path: str) -> None:
        # Relay clip path & label to DB node
        te = TrainingExample()
        te.stamp = self.get_clock().now().to_msg()
        te.label = label
        te.clip_path = clip_path  # DB node will move/copy/symlink as configured
        te.source = 'vlm-http'
        self.pub_training.publish(te)

    def call_vlm_infer_clip(self, clip_path: str):
        url = f'{self.vlm_url.rstrip("/")}/infer_clip'
        t0 = time.time()
        with open(clip_path, 'rb') as f:
            files = {'clip': (Path(clip_path).name, f, 'video/mp4')}
            resp = requests.post(url, files=files, data={'context': '{}'}, timeout=60)
        dt = int((time.time() - t0) * 1000)
        resp.raise_for_status()
        js = resp.json()
        return js.get('label', 'UNKNOWN'), float(js.get('conf', 0.0)), dt

def main() -> None:
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

