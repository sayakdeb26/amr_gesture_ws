#!/usr/bin/env python3
import json
import os
import shutil
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from vlm_interfaces.msg import Intent, TrainingExample

class CentralDBNode(Node):
    def __init__(self):
        super().__init__('central_db_node')

        # Root DB dir (env override allowed)
        self.base_dir = Path(os.getenv('AMR_DB_DIR', str(Path.home() / 'amr_db')))
        self.base_dir.mkdir(parents=True, exist_ok=True)

        # Files/dirs
        self.intents_log = self.base_dir / 'intents.jsonl'
        self.samples_dir = self.base_dir / 'samples'
        self.samples_dir.mkdir(parents=True, exist_ok=True)
        self.training_log = self.base_dir / 'training_examples.jsonl'

        # Subs
        self.sub_intents = self.create_subscription(Intent, '/intents_raw', self.on_intent, 10)
        self.sub_training = self.create_subscription(TrainingExample, '/lstm/training_example', self.on_training_example, 10)

        self.get_logger().info(f'central_db_node running. DB @ {self.base_dir}')

    # --------- callbacks ----------

    def on_intent(self, msg: Intent):
        rec = {
            'ts': time.time(),
            'stamp': {'sec': int(msg.stamp.sec), 'nsec': int(msg.stamp.nanosec)},
            'label': msg.label,
            'confidence': float(msg.confidence),
            'latency_ms': int(getattr(msg, 'latency_ms', 0)),
            'source': msg.source,
        }
        with self.intents_log.open('a', encoding='utf-8') as f:
            f.write(json.dumps(rec) + '\n')
        self.get_logger().info(f'logged intent: {rec["label"]} conf={rec["confidence"]:.2f}')

    def on_training_example(self, msg: TrainingExample):
        ts = time.time()
        label = (msg.label or 'UNKNOWN').strip()
        src_path = Path(msg.clip_path or '')
        if not src_path.is_file():
            self.get_logger().warn(f'training_example: source clip missing: {src_path}')
            return

        # Preserve extension; name: <epoch>_<label><ext>
        dst_name = f'{int(ts)}_{label}{src_path.suffix or ".mp4"}'
        dst_path = self.samples_dir / dst_name

        try:
            shutil.copy2(src_path, dst_path)
            rel = str(dst_path.relative_to(self.base_dir))
            rec = {
                'ts': ts,
                'stamp': {'sec': int(msg.stamp.sec), 'nsec': int(msg.stamp.nanosec)},
                'label': label,
                'clip': rel,
                'fps': float(msg.fps),
                'frames': int(msg.frames),
                'source': msg.source or '',
                'notes': msg.notes or '',
            }
            with self.training_log.open('a', encoding='utf-8') as f:
                f.write(json.dumps(rec) + '\n')
            self.get_logger().info(f'stored training example: {label} -> {rel} (fps={rec["fps"]:.2f}, frames={rec["frames"]})')
        except Exception as e:
            self.get_logger().error(f'failed to store training example: {e}')

def main():
    rclpy.init()
    node = CentralDBNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

