#!/usr/bin/env python3
import random, rclpy
from rclpy.node import Node
from vlm_interfaces.srv import InferClip

LABELS = ["WAVE","STOP","THUMBS_UP","OPEN_PALM","UNKNOWN"]

class DummyVLM(Node):
    def __init__(self):
        super().__init__("dummy_vlm_service")
        self.srv = self.create_service(InferClip, "/vlm/infer", self.cb)
        self.get_logger().info("Dummy VLM service ready at /vlm/infer")

    def cb(self, req, resp):
        lab = random.choice(LABELS)
        resp.label = lab
        resp.confidence = 0.55 if lab=="UNKNOWN" else 0.82
        resp.rationale = f"dummy({lab}) from {req.clip_path or 'n/a'}"
        return resp

def main():
    rclpy.init()
    n = DummyVLM()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
