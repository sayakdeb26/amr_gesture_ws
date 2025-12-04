import rclpy
from rclpy.node import Node
from amr_interfaces.msg import ConfirmRequest

def main():
    rclpy.init()
    node = Node('manual_trigger')
    pub = node.create_publisher(ConfirmRequest, '/vlm/confirm_request', 10)
    
    msg = ConfirmRequest()
    msg.session_id = "1764328662"
    msg.candidate_label = "TEST_LABEL"
    msg.candidate_conf = 0.95
    msg.hint = "Manual trigger"
    
    node.get_logger().info(f"Publishing request for session {msg.session_id}")
    pub.publish(msg)
    
    # Wait a bit to ensure it's sent
    import time
    time.sleep(1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
