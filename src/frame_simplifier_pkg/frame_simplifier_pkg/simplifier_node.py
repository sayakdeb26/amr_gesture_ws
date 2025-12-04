#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class SimplifierNode(Node):
    def __init__(self):
        super().__init__('simplifier_node')

        # Parameters
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/frames/simplified')
        self.declare_parameter('output_width', 320)
        self.declare_parameter('output_height', 240)
        self.declare_parameter('target_fps', 10.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_width = self.get_parameter('output_width').value
        self.output_height = self.get_parameter('output_height').value
        self.target_fps = self.get_parameter('target_fps').value

        self.min_interval = 1.0 / self.target_fps
        self.last_pub_time = 0.0

        self.bridge = CvBridge()

        # Subscriber
        self.sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10)

        # Publisher
        self.pub = self.create_publisher(Image, self.output_topic, 10)

        self.get_logger().info(f'Simplifier node started. Listening on {self.input_topic}, publishing to {self.output_topic} at {self.target_fps} Hz')

    def image_callback(self, msg):
        now = time.time()
        if now - self.last_pub_time < self.min_interval:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        # Resize
        try:
            resized_image = cv2.resize(cv_image, (self.output_width, self.output_height), interpolation=cv2.INTER_AREA)
        except Exception as e:
             self.get_logger().error(f'cv2 resize exception: {e}')
             return

        # Publish
        try:
            out_msg = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
            out_msg.header = msg.header # Keep original timestamp/frame_id? Or update? 
            # Keeping original timestamp is better for synchronization if needed, but we are downsampling.
            # Let's keep original header to track latency.
            self.pub.publish(out_msg)
            self.last_pub_time = now
        except Exception as e:
            self.get_logger().error(f'cv_bridge publish exception: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SimplifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
