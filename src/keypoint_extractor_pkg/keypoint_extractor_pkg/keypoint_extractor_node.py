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

class KeypointExtractorNode(Node):
    def __init__(self):
        super().__init__('keypoint_extractor_node')
        self.test_clip = os.environ.get('PERCEPTION_TEST_CLIP', '')
        self.pub = self.create_publisher(KeypointsWindow, '/lstm/keypoints_window', 10)
        self.sub = self.create_subscription(UnknownGesture, '/lstm/unknown', self.on_unknown, 10)
        self.hands = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5)
        self.get_logger().info(f'keypoint_extractor_node up. test_clip={self.test_clip if self.test_clip else "<none>"}')

    def on_unknown(self, _: UnknownGesture):
        if not self.test_clip or not os.path.isfile(self.test_clip):
            self.get_logger().warn('No valid PERCEPTION_TEST_CLIP; skip.')
            return

        cap = cv2.VideoCapture(self.test_clip)
        buf = deque(maxlen=WINDOW)
        two_hands_any = False

        while len(buf) < WINDOW:
            ok, frame = cap.read()
            if not ok:
                break
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = self.hands.process(rgb)
            pts = []
            n = 0
            if res.multi_hand_landmarks:
                n = len(res.multi_hand_landmarks)
                for hand in res.multi_hand_landmarks:
                    for lm in hand.landmark:
                        pts.extend([float(lm.x), float(lm.y), float(lm.z)])
            if n == 1:
                pts.extend([0.0] * 63)  # pad second hand
            two_hands_any = two_hands_any or (n == 2)
            if pts:
                buf.append(pts)

        cap.release()
        if len(buf) == 0:
            self.get_logger().warn('No keypoints extracted.')
            return

        last_pts = buf[-1]
        joints_per_frame = int(len(last_pts) / 3)

        if len(buf) < WINDOW:
            pad = [0.0] * ((WINDOW - len(buf)) * len(last_pts))
            flat = np.array(list(buf), dtype=np.float32).flatten().tolist() + pad
            frames = WINDOW
        else:
            flat = np.array(list(buf), dtype=np.float32).flatten().tolist()
            frames = WINDOW

        msg = KeypointsWindow()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frames = frames                 # was self.window_frames
        msg.joints_per_frame = joints_per_frame  # was hardcoded 63
        msg.data = flat
        msg.source = 'mediapipe'

        self.pub.publish(msg)
        self.get_logger().info(
            f'Published /lstm/keypoints_window frames={frames} joints/frame={joints_per_frame} two_hands={two_hands_any}'
        )

def main():
    rclpy.init()
    node = KeypointExtractorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

