#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
VLM Bridge:
- Listens for low-confidence LSTM events (/lstm/unknown)
- Calls VLM service (/vlm/infer)
- Publishes a ConfirmRequest to the UI
- Waits (confirm_timeout_s) for /ui/confirm_reply
- On approval, publishes /intents_raw and (optionally) /db/training_example
"""

import time
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile

from builtin_interfaces.msg import Time as RosTime
from amr_interfaces.msg import UnknownGesture, ConfirmRequest, ConfirmReply, Intent
from vlm_interfaces.srv import InferClip


def now_stamp(node: Node) -> RosTime:
    return node.get_clock().now().to_msg()


class VLMBridge(Node):
    def __init__(self):
        super().__init__('vlm_bridge_node')

        # -------- Parameters --------
        self.declare_parameter('default_clip', '')
        self.declare_parameter('confirm_timeout_s', 20.0)  # default to 20s

        self.default_clip: str = str(self.get_parameter('default_clip').value or '')
        self.confirm_timeout_s: float = float(self.get_parameter('confirm_timeout_s').value)

        # -------- ROS I/O --------
        qos = QoSProfile(depth=10)
        self.sub_unk = self.create_subscription(
            UnknownGesture, '/lstm/unknown', self.on_unknown, qos
        )
        self.pub_req = self.create_publisher(ConfirmRequest, '/vlm/confirm_request', qos)
        self.sub_reply = self.create_subscription(
            ConfirmReply, '/ui/confirm_reply', self.on_reply, qos
        )
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
        # carry suggestion metadata for DB on approval
        self._last_suggestion: Dict[str, object] = {}  # {'clip_path': str, 'label': str, 'conf': float}

        self.get_logger().info(
            f'vlm_bridge_node up. default_clip="{self.default_clip}", '
            f'confirm_timeout_s={self.confirm_timeout_s:.1f}'
        )

    # ---------- Callbacks ----------
    def on_unknown(self, msg: UnknownGesture):
        if self._awaiting:
            self.get_logger().info('Already awaiting confirmation; skipping new unknown.')
            return

        clip_path = self.default_clip or ''
        label_hint = (msg.hint or '').strip()

        self.get_logger().info(f'Calling VLM: clip="{clip_path}" hint="{label_hint}"')
        req = InferClip.Request(clip_path=clip_path, label_hint=label_hint)
        fut: Future = self.cli.call_async(req)
        fut.add_done_callback(self._after_vlm)

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

        # stash suggestion metadata for DB if approved
        self._last_suggestion = {
            'clip_path': self.default_clip or '',
            'label': label,
            'conf': conf,
        }

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
        # Cancel previous timer if any, then create a fresh one
        if self._watchdog_timer:
            try:
                self._watchdog_timer.cancel()
            except Exception:
                pass
            self._watchdog_timer = None
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_once)

    def _watchdog_once(self):
        if not self._awaiting:
            # done—stop this timer
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
            # stop timer
            if self._watchdog_timer:
                try:
                    self._watchdog_timer.cancel()
                except Exception:
                    pass
                self._watchdog_timer = None

    def on_reply(self, rep: ConfirmReply):
        if not self._awaiting:
            return

        # Stop the watchdog when a reply arrives
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

