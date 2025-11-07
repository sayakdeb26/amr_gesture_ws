#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SimplifierNode(Node):
    def __init__(self):
        super().__init__('frame_simplifier')
        self.declare_parameter('in_topic', '/camera/image_raw')
        self.declare_parameter('out_topic', '/image_raw_10hz')
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 10.0)

        self.in_topic  = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.W = int(self.get_parameter('width').value)
        self.H = int(self.get_parameter('height').value)
        self.FPS = float(self.get_parameter('fps').value)

        self.bridge = CvBridge()
        self.paused = False

        sub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
        hold_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)

        self.sub_img = self.create_subscription(Image, self.in_topic, self.on_img, sub_qos)
        self.pub_img = self.create_publisher(Image, self.out_topic, pub_qos)
        self.sub_hold = self.create_subscription(
            # Bool imported lazily to avoid extra dependency at top
            __import__('std_msgs.msg', fromlist=['Bool']).Bool, '/pipeline/hold', self.on_hold, hold_qos
        )

        self.next_time = time.time()
        self.get_logger().info(f"Simplifier up: {self.in_topic} → {self.out_topic} @ {self.FPS} fps")

    def on_hold(self, m):
        self.paused = bool(m.data)
        self.get_logger().info(f"Simplifier pause={self.paused}")

    def on_img(self, msg: Image):
        if self.paused:
            return
        now = time.time()
        if now < self.next_time:
            return
        self.next_time = now + (1.0 / max(1e-3, self.FPS))
        # resize
        cvimg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cvimg = cv2.resize(cvimg, (self.W, self.H), interpolation=cv2.INTER_AREA)
        out = self.bridge.cv2_to_imgmsg(cvimg, encoding='bgr8')
        out.header = msg.header
        self.pub_img.publish(out)

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
