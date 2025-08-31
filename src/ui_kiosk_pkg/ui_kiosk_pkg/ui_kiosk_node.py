import rclpy
from rclpy.node import Node
from amr_interfaces.msg import ConfirmRequest, ConfirmReply
from builtin_interfaces.msg import Time

class UIKioskNode(Node):
    def __init__(self):
        super().__init__("ui_kiosk_node")
        self.sub_req = self.create_subscription(
            ConfirmRequest, "/vlm/confirm_request", self.on_request, 10
        )
        self.pub_rep = self.create_publisher(ConfirmReply, "/ui/confirm_reply", 10)

    def on_request(self, msg: ConfirmRequest):
        self.get_logger().info(f"ConfirmRequest: label={msg.label} conf={msg.confidence:.2f}")
        rep = ConfirmReply()
        rep.stamp = self.get_clock().now().to_msg()
        rep.approved = True
        rep.final_label = msg.label
        self.pub_rep.publish(rep)
        self.get_logger().info(f"Auto-approved: {rep.final_label}")

def main():
    rclpy.init()
    node = UIKioskNode()
    rclpy.spin(node)
    rclpy.shutdown()

