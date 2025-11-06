#!/usr/bin/env python3
import os
import cv2
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from amr_interfaces.msg import UnknownGesture, KeypointsWindow

import mediapipe as mp

WINDOW = 30

class KeypointExtractorNode(Node):
    def __init__(self):
        super().__init__('keypoint_extractor_node')

        # ---------- parameters / env ----------
        self.declare_parameter('source', 'ros')
        self.declare_parameter('image_topic', '/image_raw_10hz')
        self.declare_parameter('webcam_index', 0)
        self.declare_parameter('stride', 3)
        self.declare_parameter('show_window', True)  # Fenster immer anzeigen

        self.source_mode   = self.get_parameter('source').get_parameter_value().string_value
        self.image_topic   = self.get_parameter('image_topic').get_parameter_value().string_value
        self.webcam_index  = int(self.get_parameter('webcam_index').value)
        self.stride        = int(self.get_parameter('stride').value)
        self.show_window   = bool(self.get_parameter('show_window').value)

        env_test_clip = os.environ.get('PERCEPTION_TEST_CLIP', '').strip()
        self.test_clip = env_test_clip if env_test_clip else ''
        if self.source_mode == 'clip' and not self.test_clip:
            self.get_logger().warn('source=clip but PERCEPTION_TEST_CLIP is empty.')

        qos = QoSProfile(depth=10)
        self.pub_kp = self.create_publisher(KeypointsWindow, '/lstm/keypoints_window', qos)

        # Mediapipe Hands
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.draw_spec = self.mp_draw.DrawingSpec(thickness=2, circle_radius=2,color=(31,130,245))

        # buffers
        self.buf = deque(maxlen=WINDOW)
        self.frame_idx = 0

        # mode routing
        if self.source_mode == 'ros':
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image
            self.bridge = CvBridge()
            self.sub = self.create_subscription(
                Image, self.image_topic, self.on_ros_image, qos
            )
        elif self.source_mode == 'webcam':
            self.cap = cv2.VideoCapture(self.webcam_index)
            if not self.cap.isOpened():
                self.get_logger().warn('Webcam open failed.')
                self.cap = None
            self.timer = self.create_timer(0.0, self.tick_webcam)
        elif self.source_mode == 'clip':
            self.sub_unknown = self.create_subscription(
                UnknownGesture, '/lstm/unknown', self.on_unknown, qos
            )

        if self.show_window:
            cv2.namedWindow('Video + Keypoints', cv2.WINDOW_NORMAL)

    # ---------- core helpers ----------
    def _extract_pts(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)

        pts = []
        n = 0
        if res.multi_hand_landmarks:
            n = len(res.multi_hand_landmarks)
            for hand in res.multi_hand_landmarks[:2]:
                for lm in hand.landmark:
                    pts.extend([float(lm.x), float(lm.y), float(lm.z)])

        if n < 2:
            pts.extend([0.0] * ((2 - n) * 21 * 3))

        if len(pts) != 126:
            if len(pts) < 126:
                pts.extend([0.0] * (126 - len(pts)))
            elif len(pts) > 126:
                pts = pts[:126]

        return pts, res

    def _publish_buf(self, flat, joints_per_frame, source):
        msg = KeypointsWindow()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frames = WINDOW
        msg.joints_per_frame = int(joints_per_frame)
        msg.data = flat
        msg.source = source
        self.pub_kp.publish(msg)

    def _show_frame(self, bgr, res):
        if not self.show_window:
            return
        # Mediapipe Overlay
        if res and res.multi_hand_landmarks:
            for hand in res.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(
                    bgr, hand, mp.solutions.hands.HAND_CONNECTIONS,
                    self.draw_spec, self.draw_spec
                )
        cv2.imshow('Video + Keypoints', bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.show_window = False
            cv2.destroyWindow('Video + Keypoints')

    # ---------- ROS image mode ----------
    def on_ros_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge convert failed: {e}')
            return

        pts, res = self._extract_pts(bgr)
        self._show_frame(bgr.copy(), res)

        if pts:
            self.buf.append(pts)
            self.frame_idx += 1
            if len(self.buf) == WINDOW and (self.frame_idx % max(self.stride, 1) == 0):
                flat = np.array(list(self.buf), dtype=np.float32).flatten().tolist()
                joints_per_frame = len(self.buf[-1]) // 3
                self._publish_buf(flat, joints_per_frame, 'mediapipe-ros')

    # ---------- webcam mode ----------
    def tick_webcam(self):
        if self.cap is None:
            return
        ok, frame = self.cap.read()
        if not ok:
            return
        pts, res = self._extract_pts(frame)
        self._show_frame(frame.copy(), res)

        if pts:
            self.buf.append(pts)
            self.frame_idx += 1
            if len(self.buf) == WINDOW and (self.frame_idx % max(self.stride, 1) == 0):
                flat = np.array(list(self.buf), dtype=np.float32).flatten().tolist()
                joints_per_frame = len(self.buf[-1]) // 3
                self._publish_buf(flat, joints_per_frame, 'mediapipe-live')

    # ---------- clip mode ----------
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
            pts, res = self._extract_pts(frame)
            self._show_frame(frame.copy(), res)
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
        else:
            flat = np.array(list(buf), dtype=np.float32).flatten().tolist()
        self._publish_buf(flat, joints_per_frame, 'mediapipe-clip')
        self.get_logger().info(f'Published /lstm/keypoints_window frames={WINDOW} joints/frame={joints_per_frame}')

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = KeypointExtractorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
