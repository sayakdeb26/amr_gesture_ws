from mediapipe.tasks.python.vision.core.vision_task_running_mode import VisionTaskRunningMode as RunningMode
#!/usr/bin/env python3
from pathlib import Path
#!/usr/bin/env python3
import os, cv2, numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

try:
    from vlm_interfaces.msg import UnknownGesture, KeypointsWindow
except Exception:
    from std_msgs.msg import Float32MultiArray as KeypointsWindow
    UnknownGesture = None  # not used by KPX

# MediaPipe imports (assuming installed in your env)
import mediapipe as mp
from mediapipe.tasks.python.core.base_options import BaseOptions
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

WINDOW = 30

class KeypointExtractorNode(Node):
    def __init__(self):
        super().__init__('keypoint_extractor_node')

        # ---------- parameters ----------
        self.declare_parameter('image_topic', '/image_raw_10hz')
        self.declare_parameter('stride', 1)
        self.declare_parameter('publish_annotated', False)
        self.declare_parameter('debug', False)
        default_hand_model = str(Path.home() / 'amr_gesture_ws/models/mediapipe/hand_landmarker.task')
        self.declare_parameter('hand_model_path', default_hand_model)

        self.image_topic = self.get_parameter('image_topic').value
        self.stride = int(self.get_parameter('stride').value)
        self.pub_ann = bool(self.get_parameter('publish_annotated').value)
        self.debug = bool(self.get_parameter('debug').value)

        # ---------- QoS ----------
        sub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
        hold_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)

        # ---------- ROS I/O ----------
        self.bridge = CvBridge()
        self.paused = False

        self.sub_img = self.create_subscription(Image, self.image_topic, self.on_img, sub_qos)
        self.sub_hold = self.create_subscription(
            __import__('std_msgs.msg', fromlist=['Bool']).Bool, '/pipeline/hold', self.on_hold, hold_qos
        )
        self.pub_window = self.create_publisher(KeypointsWindow, '/lstm/keypoints_window', pub_qos)
        if self.pub_ann:
            self.pub_overlay = self.create_publisher(Image, '/keypoints/overlay', pub_qos)
        else:
            self.pub_overlay = None

        # ---------- MediaPipe HandLandmarker ----------
        hand_model = self.get_parameter('hand_model_path').value
        self.get_logger().info(f'Using MediaPipe hand model: {hand_model}')
        base_opts = BaseOptions(model_asset_path=hand_model)
        opts = HandLandmarkerOptions(base_options=base_opts, num_hands=2, running_mode=RunningMode.VIDEO)
        self.landmarker = HandLandmarker.create_from_options(opts)

        self.q = deque(maxlen=WINDOW)
        self.frame_idx = 0

        self.get_logger().info(f"KPX up: {self.image_topic} → /lstm/keypoints_window (paused={self.paused})")

    def on_hold(self, m):
        self.paused = bool(m.data)
        self.get_logger().info(f"KPX pause={self.paused}")

    def on_img(self, msg: Image):
        if self.paused:
            return
        self.frame_idx += 1
        if (self.frame_idx % max(1, self.stride)) != 0:
            return
        cvimg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        H, W = cvimg.shape[:2]

        # run mediapipe detection
        mpimg = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(cvimg, cv2.COLOR_BGR2RGB))
        # VIDEO API with timestamp from header

        ts_ms = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1000000

        res = self.landmarker.detect_for_video(mpimg, ts_ms)

        # Collect normalized (x,y,z) for both hands (fill zeros when absent)
        # 21 joints per hand → 42 * 3 = 126 values per frame
        def hand_to_list(hand):
            out = []
            for lm in hand:
                out.extend([lm.x, lm.y, getattr(lm, 'z', 0.0)])
            return out

        L = res.hand_landmarks[0] if len(res.hand_landmarks) >= 1 else []
        R = res.hand_landmarks[1] if len(res.hand_landmarks) >= 2 else []
        L = hand_to_list(L) if L else [0.0]*63
        R = hand_to_list(R) if R else [0.0]*63
        frame_vec = L + R  # 126 floats

        self.q.append(frame_vec)

        if len(self.q) == self.q.maxlen:
            # publish window
            mw = KeypointsWindow()
            mw.stamp = msg.header.stamp
            mw.frames = len(self.q)
            mw.joints_per_frame = 42
            # flatten row-major: [f0..., f1..., ...]
            data = []
            for f in list(self.q):
                data.extend(f)
            mw.data = data
            mw.source = 'mediapipe'
            self.pub_window.publish(mw)

            if self.pub_overlay:
                # (Optional) draw simple dots on overlay
                overlay = cv2.cvtColor(mpimg.numpy_view(), cv2.COLOR_RGB2BGR)
                for i in range(0, 63, 3):
                    x = int(L[i] * W); y = int(L[i+1] * H)
                    cv2.circle(overlay, (x,y), 2, (0,255,0), -1)
                for i in range(0, 63, 3):
                    x = int(R[i] * W); y = int(R[i+1] * H)
                    cv2.circle(overlay, (x,y), 2, (0,0,255), -1)
                out = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                out.header = msg.header
                self.pub_overlay.publish(out)

def main():
    rclpy.init()
    node = KeypointExtractorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
