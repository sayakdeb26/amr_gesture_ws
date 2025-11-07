#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Recorder node with 6 s ring-buffer, ROI crop, and RecorderRequest→ClipReady flow.
import os, time, math, threading, shutil
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple, Deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# New typed msgs (with graceful fallback to JSON String for legacy)
try:
    from vlm_interfaces.msg import RecorderRequest, ClipReady
    HAVE_IFACES = True
except Exception:
    HAVE_IFACES = False
from std_msgs.msg import String

@dataclass
class FrameItem:
    t_ns: int
    img: np.ndarray

class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')

        # ---- params (defaults keep your old behavior unless overridden) ----
        self.declare_parameter('image_topic', '/image_raw_10hz')
        self.declare_parameter('target_fps', 10)
        self.declare_parameter('buffer_seconds', 6.0)     # 3s pre + 3s post
        self.declare_parameter('expansion_ratio', 0.30)   # 30% bbox pad
        self.declare_parameter('data_root', os.path.expanduser('~/amr_gesture_ws/data/training'))
        self.declare_parameter('samples_subdir', 'samples')
        self.declare_parameter('filename_pattern', 'clip_{session_id}.mp4')
        self.declare_parameter('fourcc', 'mp4v')          # broadly available
        self.declare_parameter('write_full_frame', False) # if True, ignore ROI
        self.declare_parameter('blur_outside_roi', False) # optional privacy
        self.declare_parameter('max_width', 640)          # optional downscale for output
        self.declare_parameter('max_height', 480)

        self.image_topic    = self.get_parameter('image_topic').value
        self.target_fps     = int(self.get_parameter('target_fps').value)
        self.buf_secs       = float(self.get_parameter('buffer_seconds').value)
        self.expand_ratio   = float(self.get_parameter('expansion_ratio').value)
        self.data_root      = self.get_parameter('data_root').value
        self.samples_subdir = self.get_parameter('samples_subdir').value
        self.filename_pat   = self.get_parameter('filename_pattern').value
        self.fourcc_name    = self.get_parameter('fourcc').value
        self.write_full     = bool(self.get_parameter('write_full_frame').value)
        self.blur_outside   = bool(self.get_parameter('blur_outside_roi').value)
        self.max_w          = int(self.get_parameter('max_width').value)
        self.max_h          = int(self.get_parameter('max_height').value)

        # ---- buffers / state ----
        self.bridge = CvBridge()
        self.buffer: Deque[FrameItem] = deque()
        self.buffer_lock = threading.Lock()
        self.latest_shape: Optional[Tuple[int,int, int]] = None  # (H,W,C)
        self.active_session: Optional[str] = None     # single-shot guard

        # compute a soft cap on buffer frames (assume ~30 fps input worst case)
        self.soft_cap_frames = int(self.buf_secs * 40)

        # ---- QoS ----
        qos_sub = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        qos_pub = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        qos_transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # ---- I/O ----
        self.sub_img = self.create_subscription(Image, self.image_topic, self.on_image, qos_sub)

        if HAVE_IFACES:
            self.sub_req = self.create_subscription(RecorderRequest, '/recorder/request', self.on_request_msg, qos_pub)
            self.pub_ready_msg = self.create_publisher(ClipReady, '/recorder/clip_ready_msg', qos_pub)
        else:
            self.get_logger().warn("amr_interfaces not found; only JSON String API will work.")

        # Always keep legacy JSON for compatibility
        self.pub_ready_json = self.create_publisher(String, '/recorder/clip_ready', qos_pub)

        # ensure output dir
        self.out_dir = os.path.join(self.data_root, self.samples_subdir)
        os.makedirs(self.out_dir, exist_ok=True)

        self.get_logger().info(f"Recorder ready. Buffer={self.buf_secs}s, topic={self.image_topic}, out={self.out_dir}")

    # ----------- image buffering -----------
    def on_image(self, msg: Image):
        try:
            cvimg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return
        h, w = cvimg.shape[:2]
        self.latest_shape = (h, w, 3)
        t_ns = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        item = FrameItem(t_ns=t_ns, img=cvimg)

        with self.buffer_lock:
            self.buffer.append(item)
            # pop left by time horizon (prefer) or soft cap
            horizon_ns = int(self.buf_secs * 1e9)
            cutoff = t_ns - horizon_ns
            while self.buffer and self.buffer[0].t_ns < cutoff:
                self.buffer.popleft()
            if len(self.buffer) > self.soft_cap_frames:
                self.buffer.popleft()

    # ----------- request handling -----------
    def on_request_msg(self, req: 'RecorderRequest'):
        if self.active_session:
            self.get_logger().warn(f"Recorder busy with session {self.active_session}, ignoring new request {req.session_id}")
            return
        self.active_session = req.session_id or f"sess_{int(time.time()*1000)}"
        threading.Thread(target=self._handle_request, args=(req,), daemon=True).start()

    def _handle_request(self, req: 'RecorderRequest'):
        try:
            clip_path, ok, msg = self._write_clip(req)
        except Exception as e:
            ok, msg, clip_path = False, f"Exception: {repr(e)}", ""
        # publish both typed and JSON responses
        if HAVE_IFACES:
            m = ClipReady(session_id=req.session_id, clip_path=clip_path, success=bool(ok), message=str(msg))
            self.pub_ready_msg.publish(m)
        j = String()
        j.data = ('{"session_id":"%s","clip_path":"%s","success":%s,"message":"%s"}'
                  % (req.session_id, clip_path.replace('"','\\"'), "true" if ok else "false", str(msg).replace('"','\\"')))
        self.pub_ready_json.publish(j)
        self.active_session = None

    # ----------- core writer -----------
    def _write_clip(self, req: 'RecorderRequest'):
        if self.latest_shape is None:
            return "", False, "no frames buffered yet"

        center_ns = req.t_center.sec*10**9 + req.t_center.nanosec
        start_ns  = center_ns - int(req.pre_secs*1e9)
        end_ns    = center_ns + int(req.post_secs*1e9)

        with self.buffer_lock:
            frames = [f for f in list(self.buffer) if start_ns <= f.t_ns <= end_ns]

        if not frames:
            return "", False, "no frames for requested window"

        # Normalize ROI → pixels
        h, w, _ = self.latest_shape
        x_min = max(0.0, min(1.0, float(req.x_min)))
        y_min = max(0.0, min(1.0, float(req.y_min)))
        x_max = max(0.0, min(1.0, float(req.x_max)))
        y_max = max(0.0, min(1.0, float(req.y_max)))

        if self.write_full or x_max <= x_min or y_max <= y_min:
            roi_px = (0, 0, w, h)
        else:
            X0 = int(x_min * w); Y0 = int(y_min * h)
            X1 = int(x_max * w); Y1 = int(y_max * h)
            # expand
            pad_x = int((X1 - X0) * self.expand_ratio)
            pad_y = int((Y1 - Y0) * self.expand_ratio)
            X0 = max(0, X0 - pad_x); Y0 = max(0, Y0 - pad_y)
            X1 = min(w, X1 + pad_x); Y1 = min(h, Y1 + pad_y)
            roi_px = (X0, Y0, X1 - X0, Y1 - Y0)

        # Optional resize target to keep output small
        out_w, out_h = roi_px[2], roi_px[3]
        scale = min(1.0, self.max_w / max(1, out_w), self.max_h / max(1, out_h))
        if scale < 1.0:
            out_w = int(out_w * scale); out_h = int(out_h * scale)

        # Prepare VideoWriter
        fourcc = cv2.VideoWriter_fourcc(*self.fourcc_name)
        clip_name = self.filename_pat.format(session_id=req.session_id or "session")
        save_path = os.path.join(self.out_dir, clip_name)
        writer = cv2.VideoWriter(save_path, fourcc, self.target_fps, (out_w, out_h))
        if not writer.isOpened():
            return "", False, f"VideoWriter({self.fourcc_name}) failed"

        try:
            for fi in frames:
                frame = fi.img
                x, y, ww, hh = roi_px
                if ww > 0 and hh > 0:
                    crop = frame[y:y+hh, x:x+ww]
                else:
                    crop = frame
                if self.blur_outside and ww>0 and hh>0:
                    # Keep full frame size, blur outside ROI (privacy mode)
                    blurred = cv2.blur(frame, (25,25))
                    blurred[y:y+hh, x:x+ww] = crop
                    out = blurred
                else:
                    out = crop
                if scale < 1.0:
                    out = cv2.resize(out, (out_w, out_h), interpolation=cv2.INTER_AREA)
                writer.write(out)
        finally:
            writer.release()

        return save_path, True, "ok"

def main():
    rclpy.init()
    node = RecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
