#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from collections import deque
from amr_interfaces.msg import UnknownGesture, KeypointsWindow
import mediapipe as mp

WINDOW = 30
STRIDE = 3  # publish every STRIDE frames in webcam mode

class KeypointExtractorNode(Node):
    def __init__(self):
        super().__init__('keypoint_extractor_node')
        self.test_clip = os.environ.get('PERCEPTION_TEST_CLIP', '').strip()
        self.use_webcam = os.environ.get('KP_WEBCAM', '1').strip() == '1'
        self.pub = self.create_publisher(KeypointsWindow, '/lstm/keypoints_window', 10)

        self.sub = None
        if self.test_clip and not self.use_webcam:
            self.sub = self.create_subscription(UnknownGesture, '/lstm/unknown', self.on_unknown, 10)

        self.hands = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5)
        mode = 'webcam' if self.use_webcam else (f'clip={self.test_clip}' if self.test_clip else 'idle')
        self.get_logger().info(f'keypoint_extractor_node up. mode={mode}')

        if self.use_webcam:
            self.timer = self.create_timer(0.0, self.tick_webcam)  # as fast as camera
            self.cap = cv2.VideoCapture(0)
            self.buf = deque(maxlen=WINDOW)
            self.frame_idx = 0
            if not self.cap.isOpened():
                self.get_logger().warn('Webcam open failed.')
        else:
            self.cap = None
            self.buf = None

    def _extract_pts(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        pts = []
        n = 0
        if res.multi_hand_landmarks:
            n = len(res.multi_hand_landmarks)
            for hand in res.multi_hand_landmarks:
                for lm in hand.landmark:
                    pts.extend([float(lm.x), float(lm.y), float(lm.z)])
        if n == 1:
            pts.extend([0.0] * 63)
        return pts

    def _publish_buf(self, flat, joints_per_frame, source):
        msg = KeypointsWindow()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frames = WINDOW
        msg.joints_per_frame = int(joints_per_frame)
        msg.data = flat
        msg.source = source
        self.pub.publish(msg)

    def tick_webcam(self):
        if self.cap is None:
            return
        ok, frame = self.cap.read()
        if not ok:
            return
        pts = self._extract_pts(frame)
        if pts:
            self.buf.append(pts)
            self.frame_idx += 1
            if len(self.buf) == WINDOW and (self.frame_idx % STRIDE == 0):
                flat = np.array(list(self.buf), dtype=np.float32).flatten().tolist()
                joints_per_frame = len(self.buf[-1]) // 3
                self._publish_buf(flat, joints_per_frame, 'mediapipe-live')

    def on_unknown(self, _: UnknownGesture):
        if not self.test_clip or not os.path.isfile(self.test_clip):
            self.get_logger().warn('No valid PERCEPTION_TEST_CLIP; skip.')
            return
        cap = cv2.VideoCapture(self.test_clip)
        buf = deque(maxlen=WINDOW)
        while len(buf) < WINDOW:
            ok, frame = cap.read()
            if not ok:
                break
            pts = self._extract_pts(frame)
            if pts:
                buf.append(pts)
        cap.release()
        if len(buf) == 0:
            self.get_logger().warn('No keypoints extracted.')
            return
        last_pts = buf[-1]
        joints_per_frame = int(len(last_pts) / 3)
        if len(buf) < WINDOW:
            pad = [0.0] * (WINDOW - len(buf)) * len(last_pts)
            flat = np.array(list(buf), dtype=np.float32).flatten().tolist() + pad
        else:
            flat = np.array(list(buf), dtype=np.float32).flatten().tolist()
        self._publish_buf(flat, joints_per_frame, 'mediapipe-clip')
        self.get_logger().info(f'Published /lstm/keypoints_window frames={WINDOW} joints/frame={joints_per_frame}')

def main():
    rclpy.init()
    node = KeypointExtractorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

