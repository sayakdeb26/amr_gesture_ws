#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
VLM Bridge (recorder-aware):
- Listens for low-confidence LSTM events (/lstm/unknown)
- Waits for recorder's /recorder/clip_ready (JSON) to get the real clip_path
- Calls VLM service (/vlm/infer) with that clip
- Publishes a ConfirmRequest to the UI
- Waits (confirm_timeout_s) for /ui/confirm_reply
- On approval, publishes /intents_raw and (optionally) /db/training_example
"""

import json
import time
import threading
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile

from builtin_interfaces.msg import Time as RosTime
from std_msgs.msg import String
from amr_interfaces.msg import UnknownGesture, ConfirmRequest, ConfirmReply, Intent
from vlm_interfaces.srv import InferClip


def now_stamp(node: Node) -> RosTime:
    return node.get_clock().now().to_msg()


class VLMBridge(Node):
    def __init__(self):
        super().__init__('vlm_bridge_node')

        # -------- Parameters --------
        self.declare_parameter('default_clip', '')
        self.declare_parameter('confirm_timeout_s', 20.0)     # UI timeout (already in your code)
        self.declare_parameter('wait_clip_timeout_s', 5.0)    # how long to wait for recorder after /lstm/unknown

        self.default_clip: str = str(self.get_parameter('default_clip').value or '')
        self.confirm_timeout_s: float = float(self.get_parameter('confirm_timeout_s').value)
        self.wait_clip_timeout_s: float = float(self.get_parameter('wait_clip_timeout_s').value)

        # -------- ROS I/O --------
        qos = QoSProfile(depth=10)

        # LSTM low-confidence trigger
        self.sub_unk = self.create_subscription(
            UnknownGesture, '/lstm/unknown', self.on_unknown, qos
        )

        # Recorder notifications: std_msgs/String JSON
        self.sub_clip = self.create_subscription(
            String, '/recorder/clip_ready', self.on_clip_ready, qos
        )

        # UI confirm I/O
        self.pub_req = self.create_publisher(ConfirmRequest, '/vlm/confirm_request', qos)
        self.sub_reply = self.create_subscription(
            ConfirmReply, '/ui/confirm_reply', self.on_reply, qos
        )

        # Final intent
        self.pub_intent = self.create_publisher(Intent, '/intents_raw', qos)

        # VLM service client
        self.cli = self.create_client(InferClip, '/vlm/infer')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('VLM service /vlm/infer not available yet; will retry on first call.')

        # Optional: central DB publisher (TrainingExample)
        try:
            from amr_interfaces.msg import TrainingExample  # type: ignore
            self.TrainingExample = TrainingExample
            self.pub_train = self.create_publisher(TrainingExample, '/db/training_example', qos)
        except Exception:
            self.TrainingExample = None
            self.pub_train = None

        # -------- Internal state --------
        self._awaiting: bool = False
        self._pending_label: Optional[str] = None
        self._request_time: float = 0.0
        self._watchdog_timer = None  # rclpy Timer handle

        # Latest recorder outputs
        self._clips_by_window: Dict[int, str] = {}
        self._last_clip: Optional[str] = None
        self._cond = threading.Condition()

        # carry suggestion metadata for DB on approval
        self._last_suggestion: Dict[str, object] = {}  # {'clip_path': str, 'label': str, 'conf': float}

        self.get_logger().info(
            f'vlm_bridge_node up. default_clip="{self.default_clip}", '
            f'confirm_timeout_s={self.confirm_timeout_s:.1f}, wait_clip_timeout_s={self.wait_clip_timeout_s:.1f}'
        )

    # ---------- Recorder side ----------
    def on_clip_ready(self, msg: String):
        """Cache the recorder's clip_path keyed by window_id."""
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'/recorder/clip_ready JSON parse failed: {e}')
            return

        clip_path = str(data.get('clip_path', '')).strip()
        window_id = int(data.get('window_id', -1))
        if not clip_path:
            return

        with self._cond:
            if window_id >= 0:
                self._clips_by_window[window_id] = clip_path
            self._last_clip = clip_path
            self._cond.notify_all()

        self.get_logger().info(f'Recorder clip cached: window_id={window_id} path="{clip_path}"')

    def _await_clip_for_window(self, window_id: int) -> Optional[str]:
        """Wait briefly for the recorder to publish a clip for this window_id."""
        deadline = time.time() + max(0.0, float(self.wait_clip_timeout_s))
        with self._cond:
            while time.time() < deadline:
                if window_id in self._clips_by_window:
                    return self._clips_by_window[window_id]
                # Small wait; awakened by on_clip_ready
                self._cond.wait(timeout=0.1)
        # Fallback: last seen clip, or default_clip
        return self._clips_by_window.get(window_id) or self._last_clip or (self.default_clip or '')

    # ---------- LSTM unknown callback ----------
    def on_unknown(self, msg: UnknownGesture):
        if self._awaiting:
            self.get_logger().info('Already awaiting confirmation; skipping new unknown.')
            return

        # Prefer the recorder's clip for this window; fall back to default_clip
        window_id = int(getattr(msg, 'window_id', -1))
        label_hint = (getattr(msg, 'hint', '') or '').strip()
        clip_path = self._await_clip_for_window(window_id)

        if not clip_path:
            self.get_logger().warn('No clip available (recorder/default); aborting VLM call.')
            return

        self.get_logger().info(f'Calling VLM: clip="{clip_path}" hint="{label_hint}" (window_id={window_id})')
        req = InferClip.Request(clip_path=clip_path, label_hint=label_hint)
        fut: Future = self.cli.call_async(req)
        # Stash the clip used so we can store it in DB if approved
        self._last_suggestion = {'clip_path': clip_path}
        fut.add_done_callback(self._after_vlm)

    # ---------- After VLM returns ----------
    def _after_vlm(self, fut: Future):
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f'VLM call failed: {e}')
            return

        label = (res.label or 'UNKNOWN').strip()
        conf = float(res.confidence)
        rationale = res.rationale or ''

        self.get_logger().info(f'VLM → label="{label}" conf={conf:.2f} note="{rationale}"')

        # complete metadata for potential DB push
        self._last_suggestion['label'] = label
        self._last_suggestion['conf'] = conf

        # Publish UI confirmation request
        req = ConfirmRequest()
        req.stamp = now_stamp(self)
        req.candidate_label = label
        req.candidate_conf = conf
        req.hint = rationale
        req.source = 'vlm'
        self.pub_req.publish(req)

        # Arm watchdog
        self._awaiting = True
        self._pending_label = label
        self._request_time = time.time()
        if self._watchdog_timer:
            try:
                self._watchdog_timer.cancel()
            except Exception:
                pass
            self._watchdog_timer = None
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_once)

    def _watchdog_once(self):
        if not self._awaiting:
            if self._watchdog_timer:
                try:
                    self._watchdog_timer.cancel()
                except Exception:
                    pass
                self._watchdog_timer = None
            return

        elapsed = time.time() - self._request_time
        if elapsed >= self.confirm_timeout_s:
            self.get_logger().warn('UI timeout; rejecting suggestion.')
            self._awaiting = False
            self._pending_label = None
            if self._watchdog_timer:
                try:
                    self._watchdog_timer.cancel()
                except Exception:
                    pass
                self._watchdog_timer = None

    # ---------- UI reply ----------
    def on_reply(self, rep: ConfirmReply):
        if not self._awaiting:
            return

        # Stop watchdog
        self._awaiting = False
        if self._watchdog_timer:
            try:
                self._watchdog_timer.cancel()
            except Exception:
                pass
            self._watchdog_timer = None

        final = rep.final_label.strip() if rep.approved and rep.final_label else None

        if rep.approved and final:
            # Emit intent
            it = Intent()
            it.stamp = now_stamp(self)
            it.label = final
            it.confidence = 0.95
            it.latency_ms = 0
            it.source = 'ui+vlm'
            self.pub_intent.publish(it)
            self.get_logger().info(f'Finalized: {final}')

            # Push to central DB (optional)
            self._emit_training_example(final)
        else:
            self.get_logger().info('Label rejected by operator; no intent emitted.')

        self._pending_label = None

    # ---------- Helpers ----------
    def _emit_training_example(self, final_label: str):
        """Publish a TrainingExample to the central DB, if that interface is available."""
        if not (self.TrainingExample and self.pub_train):
            self.get_logger().warn('TrainingExample interface not available; skipping DB push.')
            return

        msg = self.TrainingExample()

        # Set only fields that exist on the message
        if hasattr(msg, 'stamp'):
            msg.stamp = now_stamp(self)
        if hasattr(msg, 'label'):
            msg.label = final_label
        if hasattr(msg, 'clip_path'):
            msg.clip_path = str(self._last_suggestion.get('clip_path', ''))
        if hasattr(msg, 'confidence'):
            try:
                msg.confidence = float(self._last_suggestion.get('conf', 1.0))
            except Exception:
                pass
        if hasattr(msg, 'source'):
            msg.source = 'ui+vlm'

        self.pub_train.publish(msg)
        self.get_logger().info(
            f'DB push: TrainingExample(label={final_label}, clip={getattr(msg, "clip_path", "")})'
        )


def main():
    rclpy.init()
    node = VLMBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

