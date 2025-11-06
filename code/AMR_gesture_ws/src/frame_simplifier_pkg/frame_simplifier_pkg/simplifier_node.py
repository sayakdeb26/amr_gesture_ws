#!/usr/bin/env python3
import threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class SimplifierNode(Node):
    def __init__(self):
        super().__init__('frame_simplifier')
        self.declare_parameter('in_topic', '/image_raw')
        self.declare_parameter('out_topic', '/image_raw_10hz')
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 10.0)

        self.in_topic  = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.W = int(self.get_parameter('width').value)
        self.H = int(self.get_parameter('height').value)
        self.FPS = float(self.get_parameter('fps').value)

        # Drop-latest QoS: best effort, keep last=1
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        pub_qos = QoSProfile(depth=1)

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_bgr = None

        self.create_subscription(Image, self.in_topic, self._on_image, sub_qos)
        self.pub = self.create_publisher(Image, self.out_topic, pub_qos)

        self.timer = self.create_timer(1.0/self.FPS, self._tick)
        self.get_logger().info(f"simplifier: {self.in_topic} -> {self.out_topic} @ {self.W}x{self.H} {self.FPS:.1f}fps")

    def _on_image(self, msg: Image):
        try:
            # Convert once, to bgr8
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge convert failed: {e}")
            return
        # Resize fast (area or linear)
        if bgr.shape[1] != self.W or bgr.shape[0] != self.H:
            bgr = cv2.resize(bgr, (self.W, self.H), interpolation=cv2.INTER_AREA)
        with self._lock:
            self._latest_bgr = bgr

    def _tick(self):
        with self._lock:
            bgr = None if self._latest_bgr is None else self._latest_bgr.copy()
        if bgr is None:
            return
        try:
            out = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"publish failed: {e}")

def main():
    rclpy.init()
    node = SimplifierNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
