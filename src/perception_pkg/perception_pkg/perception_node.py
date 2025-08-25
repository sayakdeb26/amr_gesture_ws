#!/usr/bin/env python3
import os, json, requests, rclpy
from rclpy.node import Node
from amr_interfaces.msg import Intent
from builtin_interfaces.msg import Time as TimeMsg

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.vlm_url = os.getenv('VLM_URL', 'http://127.0.0.1:8000')
        self.pub = self.create_publisher(Intent, '/intents_raw', 10)
        self.timer = self.create_timer(2.0, self.tick)
        self.get_logger().info(f'VLM_URL = {self.vlm_url}')

    def tick(self):
        try:
            payload = {"frames": [], "context": {}}
            r = requests.post(f"{self.vlm_url}/infer", json=payload, timeout=2)
            r.raise_for_status()
            data = r.json() if r.headers.get('content-type','').startswith('application/json') else {}
            label = data.get('label', 'UNKNOWN')
            conf = float(data.get('conf', 0.0))
            lat  = int(data.get('lat_ms', 0))
            src  = data.get('source', 'vlm-http')
            self.get_logger().info(f"VLM response: {json.dumps(data)}")

            now = self.get_clock().now().to_msg()
            msg = Intent(
                stamp=TimeMsg(sec=now.sec, nanosec=now.nanosec),
                label=label,
                confidence=conf,
                latency_ms=lat,
                source=src,
            )
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"VLM call failed: {e}")

def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
