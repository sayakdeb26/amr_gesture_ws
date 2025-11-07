#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
VLM Bridge (session-locked):
- On /lstm/unknown: freeze pipeline, compute ROI from /lstm/keypoints_window,
  request recorder [t-3s, t+3s], wait for clip, call VLM once, show UI, resume.
- Publishes /pipeline/hold (Bool, transient_local).
- Backward compatible with legacy JSON /recorder/clip_ready.
"""
import json, time, threading, uuid, math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Time as RosTime

from vlm_interfaces.msg import UnknownGesture, KeypointsWindow, RecorderRequest, ClipReady
# Keep your existing UI/VLM imports if any (not shown in this trimmed node),
# e.g., from vlm_interfaces.msg import ConfirmRequest, ConfirmReply, Intent

WINDOW_PRE = 3.0
WINDOW_POST = 3.0

class VlmBridgeNode(Node):
    def __init__(self):
        super().__init__('vlm_bridge_node')

        # ---- params (keep your defaults; expose timeouts) ----
        self.declare_parameter('confirm_timeout_s', 15.0)
        self.declare_parameter('wait_clip_timeout_s', 5.0)
        self.confirm_timeout_s = float(self.get_parameter('confirm_timeout_s').value)
        self.wait_clip_timeout_s = float(self.get_parameter('wait_clip_timeout_s').value)

        # ---- QoS ----
        qos_sub = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_pub = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_hold = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)

        # ---- state ----
        self.session_id: Optional[str] = None
        self.latest_kp: Optional[KeypointsWindow] = None
        self.clip_for_session: Optional[str] = None

        # ---- I/O ----
        self.sub_unknown = self.create_subscription(UnknownGesture, '/lstm/unknown', self.on_unknown, qos_sub)
        self.sub_kp      = self.create_subscription(KeypointsWindow, '/lstm/keypoints_window', self.on_kp, qos_sub)

        self.pub_hold    = self.create_publisher(Bool, '/pipeline/hold', qos_hold)
        self.pub_req     = self.create_publisher(RecorderRequest, '/recorder/request', qos_pub)

        self.sub_ready   = self.create_subscription(ClipReady, '/recorder/clip_ready_msg', self.on_clip_ready_msg, qos_sub)
        self.sub_ready_j = self.create_subscription(String, '/recorder/clip_ready', self.on_clip_ready_json, qos_sub)

        self.get_logger().info("VLM Bridge (session-locked) ready.")

    # ---------- Subscribers ----------
    def on_kp(self, msg: KeypointsWindow):
        self.latest_kp = msg

    def on_unknown(self, msg: UnknownGesture):
        if self.session_id is not None:
            self.get_logger().warn("Session already active; ignoring new unknown.")
            return
        # Start session
        self.session_id = str(uuid.uuid4())
        self.clip_for_session = None

        # publish hold=True
        self._set_hold(True)

        # compute ROI from latest keypoints
        x0, y0, x1, y1 = self._roi_from_keypoints(self.latest_kp)

        # build RecorderRequest
        req = RecorderRequest()
        req.session_id = self.session_id
        req.window_id = f"win_{msg.window_frames or 0}"
        req.t_center = msg.stamp if isinstance(msg.stamp, RosTime) else RosTime(sec=0, nanosec=0)
        req.pre_secs = float(WINDOW_PRE)
        req.post_secs = float(WINDOW_POST)
        req.x_min, req.y_min, req.x_max, req.y_max = x0, y0, x1, y1
        self.pub_req.publish(req)
        self.get_logger().info(f"[{self.session_id}] RecorderRequest sent ROI=({x0:.3f},{y0:.3f},{x1:.3f},{y1:.3f})")

        # spawn watcher thread: wait for /recorder/clip_ready*
        threading.Thread(target=self._wait_clip_then_vlm, daemon=True).start()

    def on_clip_ready_msg(self, msg: ClipReady):
        if self.session_id and msg.session_id == self.session_id and msg.success:
            self.clip_for_session = msg.clip_path

    def on_clip_ready_json(self, js: String):
        try:
            d = json.loads(js.data)
        except Exception:
            return
        if not isinstance(d, dict):
            return
        if self.session_id and d.get('session_id') == self.session_id and d.get('success'):
            self.clip_for_session = d.get('clip_path')

    # ---------- Helpers ----------
    def _roi_from_keypoints(self, kp: Optional[KeypointsWindow]) -> Tuple[float,float,float,float]:
        # Default full-frame fallback
        if kp is None or kp.frames == 0 or kp.joints_per_frame == 0:
            return 0.0, 0.0, 1.0, 1.0

        J = int(kp.joints_per_frame)
        F = int(kp.frames)
        arr = kp.data
        if len(arr) < F * J * 3:
            return 0.0, 0.0, 1.0, 1.0

        # gather all x,y in [0,1]
        xs, ys = [], []
        # row-major: frame0_j0x, frame0_j0y, frame0_j0z, ...
        for f in range(F):
            base = f * J * 3
            for j in range(J):
                x = arr[base + j*3 + 0]
                y = arr[base + j*3 + 1]
                if 0.0 <= x <= 1.0 and 0.0 <= y <= 1.0:
                    xs.append(x); ys.append(y)
        if not xs:
            return 0.0, 0.0, 1.0, 1.0

        x_min, x_max = max(0.0, min(xs)), min(1.0, max(xs))
        y_min, y_max = max(0.0, min(ys)), min(1.0, max(ys))

        # pad 30%
        pad_x = (x_max - x_min) * 0.30
        pad_y = (y_max - y_min) * 0.30
        x_min = max(0.0, x_min - pad_x); y_min = max(0.0, y_min - pad_y)
        x_max = min(1.0, x_max + pad_x); y_max = min(1.0, y_max + pad_y)
        return x_min, y_min, x_max, y_max

    def _set_hold(self, val: bool):
        m = Bool(data=val)
        self.pub_hold.publish(m)
        self.get_logger().info(f"/pipeline/hold = {val}")

    def _wait_clip_then_vlm(self):
        # wait for recorder clip
        t0 = time.time()
        while self.session_id and self.clip_for_session is None and (time.time()-t0) < self.wait_clip_timeout_s:
            time.sleep(0.05)

        if not self.session_id:
            return

        if not self.clip_for_session:
            self.get_logger().error(f"[{self.session_id}] recorder timeout")
            self._finalize_session(resume=True)
            return

        clip = self.clip_for_session
        self.get_logger().info(f"[{self.session_id}] got clip: {clip}")

        # ===== Call your VLM & UI flow =====
        # Keep your original logic here (service/client to /vlm/infer, publish ConfirmRequest, wait ConfirmReply).
        # For brevity, we simulate a positive decision after confirm_timeout_s or immediate path.

        # TODO: integrate your existing call + confirm logic. Here we just sleep ≤ confirm_timeout_s.
        t1 = time.time()
        while (time.time()-t1) < self.confirm_timeout_s:
            time.sleep(0.05)
            # If your real code sets a decision, break here.

        # Resume either way
        self._finalize_session(resume=True)

    def _finalize_session(self, resume: bool):
        if resume:
            self._set_hold(False)
        self.get_logger().info(f"[{self.session_id}] session end; resume={resume}")
        self.session_id = None
        self.clip_for_session = None

def main():
    rclpy.init()
    node = VlmBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
