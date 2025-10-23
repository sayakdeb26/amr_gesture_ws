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
        # preferred: source=ros and provide image_topic for ROS image mode
        self.declare_parameter('source', 'ros')                   # 'ros' | 'webcam' | 'clip'
        self.declare_parameter('image_topic', '/image_raw_10hz')  # ROS subscribed image (if source=ros)
        self.declare_parameter('webcam_index', 0)
        self.declare_parameter('stride', 3)                       # publish every N frames
        self.declare_parameter('debug', False)                    # show OpenCV window (overlay)
        self.declare_parameter('debug_stride', 3)                 # draw every N frames when debugging

        self.source_mode   = self.get_parameter('source').get_parameter_value().string_value
        self.image_topic   = self.get_parameter('image_topic').get_parameter_value().string_value
        self.webcam_index  = int(self.get_parameter('webcam_index').value)
        self.stride        = int(self.get_parameter('stride').value)
        self.debug         = bool(self.get_parameter('debug').value)
        self.debug_stride  = int(self.get_parameter('debug_stride').value)

        # legacy envs kept for compatibility
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
        self.draw_spec = self.mp_draw.DrawingSpec(thickness=2, circle_radius=2)

        # buffers
        self.buf = deque(maxlen=WINDOW)
        self.frame_idx = 0
        self.debug_idx = 0

        # mode routing
        if self.source_mode == 'ros':
            # We subscribe to a *BGR8* image topic produced by your simplifier.
            # To avoid cv_bridge dependency (and keep things very light),
            # we expect /image_raw_10hz to already be BGR8 and we get it via cv2.imdecode path.
            # BUT: the simplifier publishes sensor_msgs/Image — so we DO use cv_bridge here.
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image
            self.bridge = CvBridge()
            self.sub = self.create_subscription(
                Image, self.image_topic, self.on_ros_image, qos
            )
            mode = f'ros:{self.image_topic}'
        elif self.source_mode == 'webcam':
            self.cap = cv2.VideoCapture(self.webcam_index)
            if not self.cap.isOpened():
                self.get_logger().warn('Webcam open failed.')
                self.cap = None
            self.timer = self.create_timer(0.0, self.tick_webcam)
            mode = f'webcam:{self.webcam_index}'
        elif self.source_mode == 'clip':
            # Wait for trigger; we run only when an UnknownGesture arrives
            self.sub_unknown = self.create_subscription(
                UnknownGesture, '/lstm/unknown', self.on_unknown, qos
            )
            mode = f'clip:{self.test_clip if self.test_clip else "<none>"}'
        else:
            mode = f'unknown({self.source_mode})'
            self.get_logger().warn(f'Unknown source mode: {self.source_mode}')

        self.get_logger().info(
            f'keypoint_extractor_node up. mode={mode}, stride={self.stride}, debug={self.debug}'
        )

    # ---------- core helpers ----------
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
        # pad to 2 hands if only 1 detected to keep shape consistent (63 joints/hand * 2 * 3floats)
        if n == 1:
            pts.extend([0.0] * 63)
        return pts, res

    def _publish_buf(self, flat, joints_per_frame, source):
        msg = KeypointsWindow()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frames = WINDOW
        msg.joints_per_frame = int(joints_per_frame)
        msg.data = flat
        msg.source = source
        self.pub_kp.publish(msg)

    def _maybe_debug(self, bgr, res):
        """Draw overlay and show window if debug is enabled."""
        if not self.debug:
            return
        self.debug_idx += 1
        if self.debug_idx % max(self.debug_stride, 1) != 0:
            return
        try:
            if res and res.multi_hand_landmarks:
                for hand in res.multi_hand_landmarks:
                    self.mp_draw.draw_landmarks(
                        bgr, hand, mp.solutions.hands.HAND_CONNECTIONS,
                        self.draw_spec, self.draw_spec
                    )
            cv2.imshow('Keypoints Debug', bgr)
            # press q to close the preview without killing node
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.debug = False
                cv2.destroyWindow('Keypoints Debug')
        except cv2.error:
            # headless / wayland issues: just ignore
            pass

    # ---------- ROS image mode ----------
    def on_ros_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge convert failed: {e}')
            return

        pts, res = self._extract_pts(bgr)
        self._maybe_debug(bgr.copy(), res)

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
        self._maybe_debug(frame.copy(), res)

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
            self._maybe_debug(frame.copy(), res)
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

    # clean up debug window on shutdown
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

