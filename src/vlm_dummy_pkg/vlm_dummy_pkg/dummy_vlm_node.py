#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vlm_interfaces.srv import InferClip

class DummyVLMNode(Node):
    def __init__(self):
        super().__init__('dummy_vlm_node')
        self.srv = self.create_service(InferClip, '/vlm/infer', self.infer_callback)
        self.get_logger().info('Dummy VLM Service Ready')

    def infer_callback(self, request, response):
        self.get_logger().info(f"Received VLM request for session {request.session_id}, clip {request.clip_path}")
        
        # Dummy logic
        response.label = "UNKNOWN"
        response.confidence = 0.0
        response.rationale = "Dummy VLM: not implemented yet"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DummyVLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
