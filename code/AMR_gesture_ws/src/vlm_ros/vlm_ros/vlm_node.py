#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vlm_interfaces.srv import InferClip

class VLMNode(Node):
    def __init__(self):
        super().__init__('vlm_node')
        self.srv = self.create_service(InferClip, '/vlm/infer', self.handle_infer)
        self.get_logger().info('VLM stub service ready at /vlm/infer')

    def handle_infer(self, req, resp):
        # req.clip_path: str, req.label_hint: str
        label = (req.label_hint or 'wave_stop').strip()
        resp.label = label
        resp.confidence = 0.72
        resp.rationale = f'stub: clip={req.clip_path}'
        self.get_logger().info(f"VLM infer: label={resp.label} conf={resp.confidence:.2f}")
        return resp

def main():
    rclpy.init()
    node = VLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
