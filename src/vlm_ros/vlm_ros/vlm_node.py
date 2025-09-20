import time
import rclpy
from rclpy.node import Node
from vlm_interfaces.srv import InferClip

class VLMNode(Node):
    def __init__(self):
        super().__init__('vlm_node')
        self.srv = self.create_service(InferClip, 'vlm/infer', self.handle_infer)
        self.get_logger().info('vlm_node up: providing /vlm/infer')
        # TODO: load your real VLM model here when ready.

    def handle_infer(self, request, response):
        t0 = time.time()
        # TODO: replace stub with actual inference on request.clip_path
        response.label = 'WAVE_STOP'
        response.confidence = 0.72
        response.rationale = 'ROS VLM stub'
        response.latency_ms = int((time.time() - t0) * 1000)
        return response

def main():
    rclpy.init()
    node = VLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
