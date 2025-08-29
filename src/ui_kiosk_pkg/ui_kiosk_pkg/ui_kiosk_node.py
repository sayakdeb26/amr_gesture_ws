#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UIKioskNode(Node):
    def __init__(self):
        super().__init__('ui_kiosk_node')
        # Subscriptions
        self.sub = self.create_subscription(String, '/vlm/confirm_request', self.on_confirm_request, 10)
        # Publisher
        self.pub = self.create_publisher(String, '/ui/confirm_reply', 10)
        self.get_logger().info('ui_kiosk_node running.')

    def on_confirm_request(self, msg: String):
        # Log incoming request and auto-approve (placeholder)
        self.get_logger().info(f"UI received request: {msg.data}")
        reply = String()
        reply.data = '{"approved": true, "final_label": "WAVE_STOP"}'
        self.pub.publish(reply)
        self.get_logger().info("UI sent dummy reply")
        
def main():
    rclpy.init()
    node = UIKioskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
