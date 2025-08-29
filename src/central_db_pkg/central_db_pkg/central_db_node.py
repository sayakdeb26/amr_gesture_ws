#!/usr/bin/env python3
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from amr_interfaces.msg import Intent  # /intents_raw

class CentralDBNode(Node):
    def __init__(self):
        super().__init__('central_db_node')
        # Database paths
        self.base_dir = Path(os.getenv('AMR_DB_DIR', str(Path.home() / 'amr_db')))
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.intents_log = self.base_dir / 'intents.jsonl'
        self.samples_dir = self.base_dir / 'samples'
        self.samples_dir.mkdir(parents=True, exist_ok=True)

        # Subscriptions
        self.sub_intents = self.create_subscription(Intent, '/intents_raw', self.on_intent, 10)

        self.get_logger().info(f'central_db_node running. DB @ {self.base_dir}')

    def on_intent(self, msg: Intent):
        # Save intent to JSON lines
        rec = {
            'ts': time.time(),
            'stamp': {'sec': msg.stamp.sec, 'nsec': msg.stamp.nanosec},
            'label': msg.label,
            'confidence': float(msg.confidence),
            'latency_ms': int(msg.latency_ms),
            'source': msg.source,
        }
        with self.intents_log.open('a', encoding='utf-8') as f:
            f.write(json.dumps(rec) + '\n')
        self.get_logger().info(f'logged intent: {rec["label"]} conf={rec["confidence"]:.2f}')

def main():
    rclpy.init()
    node = CentralDBNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
