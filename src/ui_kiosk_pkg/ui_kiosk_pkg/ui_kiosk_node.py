#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UIKioskNode(Node):
    def __init__(self):
        super().__init__('ui_kiosk_node')
        self.sub = self.create_subscription(String, '/vlm/confirm_request', self.on_req, 10)
        self.pub = self.create_publisher(String, '/ui/confirm_reply', 10)
        self.get_logger().info('ui_kiosk_node up (stub).')

    def on_req(self, msg: String):
        self.get_logger().info(f'confirm_request: {msg.data}')
        # Always approve in this stub after a short delay
        self.create_timer(0.2, self._approve_once)

    def _approve_once(self):
        self.destroy_timer(self.timers[-1]) if hasattr(self, 'timers') else None
        self.pub.publish(String(data='approved:WAVE_STOP'))

def main():
    rclpy.init()
    node = UIKioskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
