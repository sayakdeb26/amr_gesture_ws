#!/usr/bin/env python3
import os, cv2, numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from amr_interfaces.msg import UnknownGesture, KeypointsWindow

# ---- MediaPipe Tasks (GPU-capable)
import mediapipe as mp
from mediapipe.tasks.python.core.base_options import BaseOptions
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode
from mediapipe.framework.formats import landmark_pb2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

WINDOW = 30

class KeypointExtractorNode(Node):
    def __init__(self):
        super().__init__('keypoint_extractor_node')

        # ---------- parameters ----------
        self.declare_parameter('source', 'ros')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('stride', 1)
        self.declare_parameter('debug', True)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('annotated_topic', '/keypoints_annotated')
        self.declare_parameter('hand_task_path', os.path.expanduser('~/amr_gesture_ws/models/mediapipe/hand_landmarker.task'))
        self.declare_parameter('use_gpu', True)

        self.source_mode   = self.get_parameter('source').get_parameter_value().string_value
        self.image_topic   = self.get_parameter('image_topic').get_parameter_value().string_value
        self.stride        = int(self.get_parameter('stride').value)
        self.debug         = bool(self.get_parameter('debug').value)
        self.pub_anno      = bool(self.get_parameter('publish_annotated').value)
        self.anno_topic    = self.get_parameter('annotated_topic').get_parameter_value().string_value
        self.task_path     = self.get_parameter('hand_task_path').get_parameter_value().string_value
        self.want_gpu      = bool(self.get_parameter('use_gpu').value)

        qos = QoSProfile(depth=10)
        self.pub_kp   = self.create_publisher(KeypointsWindow, '/lstm/keypoints_window', qos)
        self.pub_anno_img = self.create_publisher(Image, self.anno_topic, qos) if self.pub_anno else None

        self.bridge = CvBridge()
        self.buf = deque(maxlen=WINDOW)
        self.frame_idx = 0

        # Build MediaPipe HandLandmarker (GPU -> CPU fallback)
        self.landmarker = None
        prov = 'GPU' if self.want_gpu else 'CPU'
        try:
            self.landmarker = self._make_landmarker('GPU')
            self.get_logger().info('MediaPipe Tasks HandLandmarker: GPU delegate active')
        except Exception as e:
            if self.want_gpu:
                self.get_logger().warn(f'GPU delegate failed: {e} ; falling back to CPU')
            self.landmarker = self._make_landmarker('CPU')
            self.get_logger().info('MediaPipe Tasks HandLandmarker: CPU active')

        # ROS image subscription
        if self.source_mode != 'ros':
            self.get_logger().warn(f"Only 'ros' mode implemented here; got {self.source_mode}. Using ROS.")
        self.sub = self.create_subscription(Image, self.image_topic, self.on_ros_image, qos)

        self.get_logger().info(
            f'keypoint_extractor_node up. mode=ros:{self.image_topic}, stride={self.stride}, debug={self.debug}'
        )

    def _make_landmarker(self, mode: str) -> HandLandmarker:
        if not os.path.isfile(self.task_path):
            raise FileNotFoundError(f"Missing hand task file: {self.task_path}")

        delegate = BaseOptions.Delegate.GPU if mode == 'GPU' else BaseOptions.Delegate.CPU
        opts = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.task_path, delegate=delegate),
            running_mode=RunningMode.IMAGE,      # frame-by-frame
            num_hands=2,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5
        )
        return HandLandmarker.create_from_options(opts)

    # ---------- ROS image callback ----------
    def on_ros_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge convert failed: {e}')
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        res = self.landmarker.detect(mp_image)

        # Collect per-hand (x,y,z) in image coords scaled to [0,1]
        pts = []
        n_hands = 0
        if res and res.hand_landmarks:
            n_hands = len(res.hand_landmarks)
            h, w = rgb.shape[:2]
            for hand in res.hand_landmarks:
                for lm in hand:  # 21 landmarks
                    # MP Tasks returns normalized coords already in [0,1], z relative.
                    pts.extend([float(lm.x), float(lm.y), float(lm.z)])

        # Pad to two hands to keep shape consistent (21*3*2 = 126)
        if n_hands == 1:
            pts.extend([0.0] * (21 * 3))

        # Maintain rolling window; publish every 'stride'
        if pts:
            # Convert to (x,y) only for F=84 (LSTM expects [30,84])
            xy_only = []
            for i in range(0, len(pts), 3):
                xy_only.extend([pts[i], pts[i+1]])
            self.buf.append(np.asarray(xy_only, dtype=np.float32))

            self.frame_idx += 1
            if len(self.buf) == WINDOW and (self.frame_idx % max(self.stride, 1) == 0):
                win = np.zeros((WINDOW, len(self.buf[-1])), dtype=np.float32)  # (30, 84)
                recent = list(self.buf)[-WINDOW:]
                win[-len(recent):] = np.stack(recent, axis=0)
                flat = win.ravel().tolist()

                kp = KeypointsWindow()
                kp.stamp = self.get_clock().now().to_msg()
                kp.frames = WINDOW
                kp.joints_per_frame = int(len(self.buf[-1]) // 2)  # 42
                kp.data = flat
                kp.source = 'mediapipe-tasks'
                self.pub_kp.publish(kp)

        # Optional debug overlay publisher
        if self.pub_anno_img is not None:
            vis = bgr.copy()
            if res and res.hand_landmarks:
                for hand in res.hand_landmarks:
                    for lm in hand:
                        x = int(lm.x * vis.shape[1])
                        y = int(lm.y * vis.shape[0])
                        cv2.circle(vis, (x, y), 2, (0,255,0), -1)
            try:
                self.pub_anno_img.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
            except Exception:
                pass

def main():
    rclpy.init()
    node = KeypointExtractorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
