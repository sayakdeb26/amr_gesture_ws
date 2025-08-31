#!/usr/bin/env python3
# Perception node:
# - subs:  /lstm/unknown            (amr_interfaces/UnknownGesture)
# - pubs:  /vlm/confirm_request     (amr_interfaces/ConfirmRequest)
#         /intents_raw             (amr_interfaces/Intent)
# - subs:  /ui/confirm_reply        (amr_interfaces/ConfirmReply)
#
# Flow:
#  1) on UnknownGesture -> call VLM (using PERCEPTION_TEST_CLIP if provided)
#  2) publish ConfirmRequest, set "pending" and RETURN (non-blocking)
#  3) on ConfirmReply within timeout -> publish /intents_raw
#     else on timeout -> auto-approve (if AUTO_APPROVE_ON_TIMEOUT=1) or drop.

import os
import json
import time
import threading
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from amr_interfaces.msg import Intent
from amr_interfaces.msg import UnknownGesture
from amr_interfaces.msg import ConfirmRequest
from amr_interfaces.msg import ConfirmReply

import requests


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # ---- config via env ----
        self.vlm_url = os.getenv('VLM_URL', 'http://127.0.0.1:8000').rstrip('/')
        self.test_clip = os.getenv('PERCEPTION_TEST_CLIP', '').strip() or None
        self.confirm_timeout_s = float(os.getenv('CONFIRM_TIMEOUT_S', '8'))
        self.auto_approve_on_timeout = os.getenv('AUTO_APPROVE_ON_TIMEOUT', '0') == '1'

        # callback group allows concurrent callbacks (timer + subs)
        self.cb_group = ReentrantCallbackGroup()

        # ---- pubs/subs ----
        self.pub_intents = self.create_publisher(Intent, '/intents_raw', 10)
        self.pub_confirm_req = self.create_publisher(ConfirmRequest, '/vlm/confirm_request', 10)

        self.sub_unknown = self.create_subscription(
            UnknownGesture, '/lstm/unknown', self.on_unknown, 10, callback_group=self.cb_group
        )
        self.sub_confirm_reply = self.create_subscription(
            ConfirmReply, '/ui/confirm_reply', self.on_confirm_reply, 10, callback_group=self.cb_group
        )

        # ---- pending confirm (single-flight) ----
        self._pending_lock = threading.Lock()
        self._pending: Optional[Dict[str, Any]] = None  # {'label','conf','lat_ms','timer'}

        self.get_logger().info(
            f'perception_node up. VLM_URL={self.vlm_url}, '
            f'test_clip={self.test_clip or "<none>"}, '
            f'confirm_timeout={self.confirm_timeout_s:.1f}s, '
            f'auto_approve={self.auto_approve_on_timeout}'
        )

        if not self.test_clip:
            self.get_logger().warn('No PERCEPTION_TEST_CLIP set; live capture not implemented yet.')

    # =========================
    # Callbacks
    # =========================
    def on_unknown(self, msg: UnknownGesture):
        """Triggered by LSTM low-confidence branch."""
        self.get_logger().info(
            f'UnknownGesture received (conf={msg.confidence:.2f}, hint="{msg.hint}") … calling VLM'
        )
        label, conf, lat_ms = self._call_vlm(self.test_clip)

        # publish confirm request and set pending (non-blocking)
        req = ConfirmRequest()
        req.stamp = self.get_clock().now().to_msg()
        req.label = label
        req.confidence = float(conf)
        self.pub_confirm_req.publish(req)

        self._set_pending(label=label, conf=float(conf), lat_ms=int(lat_ms))

    def on_confirm_reply(self, msg: ConfirmReply):
        """Approval/rejection from UI."""
        p = self._consume_pending()
        if not p:
            return  # either timed out or nothing pending

        # cancel timeout timer
        t = p.get('timer')
        if t:
            try:
                t.cancel()
            except Exception:
                pass

        if msg.approved:
            final_label = msg.final_label.strip() if msg.final_label else p['label']
            self.get_logger().info(f'Confirm reply: approved=True, final={final_label}')
            self._publish_intent(final_label, p['conf'], p['lat_ms'], source='vlm-http')
        else:
            self.get_logger().info('Confirm reply: approved=False; dropping.')

    # =========================
    # Pending / Timer
    # =========================
    def _set_pending(self, *, label: str, conf: float, lat_ms: int):
        """Set one in-flight confirmation with timeout."""
        with self._pending_lock:
            # clear previous
            if self._pending and self._pending.get('timer'):
                try:
                    self._pending['timer'].cancel()
                except Exception:
                    pass

            # timer to handle timeout
            timer = self.create_timer(self.confirm_timeout_s, self._on_confirm_timeout,
                                      callback_group=self.cb_group)
            self._pending = {'label': label, 'conf': conf, 'lat_ms': lat_ms, 'timer': timer}

    def _consume_pending(self) -> Optional[Dict[str, Any]]:
        """Atomically take and clear pending state."""
        with self._pending_lock:
            p = self._pending
            self._pending = None
            return p

    def _peek_pending(self) -> Optional[Dict[str, Any]]:
        with self._pending_lock:
            return dict(self._pending) if self._pending else None

    def _on_confirm_timeout(self):
        """Timer callback when kiosk didn't reply in time."""
        p = self._consume_pending()
        if not p:
            return
        try:
            p['timer'].cancel()
        except Exception:
            pass

        if self.auto_approve_on_timeout:
            self.get_logger().warn('No confirm reply within timeout; AUTO-APPROVE on.')
            self._publish_intent(p['label'], p['conf'], p['lat_ms'], source='vlm-http')
        else:
            self.get_logger().warn('No confirm reply within timeout; dropping.')

    # =========================
    # VLM call
    # =========================
    def _call_vlm(self, clip_path: Optional[str]):
        """
        Call FastAPI VLM /infer_clip.
        Returns (label:str, conf:float, lat_ms:int). Always returns something.
        """
        if not clip_path or not os.path.exists(clip_path):
            # Fallback stub (keeps pipeline alive)
            self.get_logger().warn('Test clip missing; using stub VLM result.')
            return 'WAVE_STOP', 0.84, 0

        url = f'{self.vlm_url}/infer_clip'
        t0 = time.time()
        try:
            with open(clip_path, 'rb') as f:
                files = {'clip': (os.path.basename(clip_path), f, 'video/mp4')}
                data = {'context': json.dumps({})}
                r = requests.post(url, files=files, data=data, timeout=15)
            r.raise_for_status()
            resp = r.json()
            label = str(resp.get('label', 'WAVE_STOP'))
            conf = float(resp.get('conf', 0.5))
            lat_ms = int(resp.get('lat_ms', int((time.time() - t0) * 1000)))
            self.get_logger().info(f'VLM result: {label} conf={conf:.2f} lat={lat_ms}ms')
            return label, conf, lat_ms
        except Exception as e:
            self.get_logger().error(f'VLM call failed: {e}; falling back.')
            return 'WAVE_STOP', 0.5, int((time.time() - t0) * 1000)

    # =========================
    # Publish intent
    # =========================
    def _publish_intent(self, label: str, conf: float, lat_ms: int, *, source: str):
        out = Intent()
        out.stamp = self.get_clock().now().to_msg()
        out.label = label
        out.confidence = float(conf)
        out.latency_ms = int(lat_ms)
        out.source = source
        self.pub_intents.publish(out)
        self.get_logger().info(f'Published /intents_raw: {label} ({conf:.2f})')

def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)  # Reentrant group lets timer + subs run concurrently
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

