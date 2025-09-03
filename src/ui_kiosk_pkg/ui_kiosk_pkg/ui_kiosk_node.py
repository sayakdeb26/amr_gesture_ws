#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from amr_interfaces.msg import ConfirmRequest, ConfirmReply

class UIKioskNode(Node):
    def __init__(self):
        super().__init__('ui_kiosk_node')
        self.sub = self.create_subscription(
            ConfirmRequest, '/vlm/confirm_request', self.on_request, 10
        )
        self.pub = self.create_publisher(ConfirmReply, '/ui/confirm_reply', 10)
        self.get_logger().info('ui_kiosk_node up (auto-approve).')

    def on_request(self, msg: ConfirmRequest):
        label = msg.candidate_label or ''
        conf = float(getattr(msg, 'candidate_conf', 0.0))
        hint = msg.hint or ''
        self.get_logger().info(f'ConfirmRequest: label={label} conf={conf:.2f} hint={hint}')
        rep = ConfirmReply()
        rep.stamp = self.get_clock().now().to_msg()
        rep.approved = True
        rep.final_label = label
        self.pub.publish(rep)
        self.get_logger().info(f'Auto-approved: {label}')

def main():
    rclpy.init()
    n = UIKioskNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

