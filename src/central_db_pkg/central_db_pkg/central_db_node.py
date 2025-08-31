#!/usr/bin/env python3
import json
import os
import time
import shutil
import uuid
from pathlib import Path

import rclpy
from rclpy.node import Node

from amr_interfaces.msg import Intent, TrainingExample


class CentralDBNode(Node):
    def __init__(self):
        super().__init__('central_db_node')

        # Root DB dir
        self.base_dir = Path(os.getenv('AMR_DB_DIR', str(Path.home() / 'amr_db')))
        self.base_dir.mkdir(parents=True, exist_ok=True)

        # Files/dirs
        self.intents_log = self.base_dir / 'intents.jsonl'
        self.samples_dir = self.base_dir / 'samples'
        self.samples_dir.mkdir(parents=True, exist_ok=True)
        self.te_log = self.base_dir / 'training_examples.jsonl'

        # Subscriptions
        self.sub_intents = self.create_subscription(Intent, '/intents_raw', self.on_intent, 10)
        self.sub_te = self.create_subscription(
            TrainingExample, '/lstm/training_example', self.on_training_example, 10
        )

        # Behavior: copy (default) or symlink into samples/
        self.symlink_samples = os.getenv('DB_SAMPLES_SYMLINK', '0') == '1'

        self.get_logger().info(f'central_db_node running. DB @ {self.base_dir}')

    # -------- handlers --------

    def on_intent(self, msg: Intent):
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

    def on_training_example(self, msg: TrainingExample):
        # Persist clip (copy or symlink) into samples/
        src = Path(msg.clip_path)
        if not src.exists():
            self.get_logger().warn(f'TrainingExample clip not found: {src}')
            return

        # samples/<epoch_ms>_<uuid>_<label><ext>
        epoch_ms = int(time.time() * 1000)
        ext = src.suffix or '.mp4'
        rel_name = f'{epoch_ms}_{uuid.uuid4().hex}_{msg.label}{ext}'
        dst = self.samples_dir / rel_name

        try:
            if self.symlink_samples:
                # Prefer relative symlink when possible
                try:
                    rel_src = os.path.relpath(src, start=self.samples_dir)
                except Exception:
                    rel_src = str(src)
                dst.symlink_to(rel_src)
            else:
                shutil.copy2(src, dst)
        except Exception as e:
            self.get_logger().error(f'Failed to persist sample: {e}')
            return

        rec = {
            'ts': time.time(),
            'stamp': {'sec': msg.stamp.sec, 'nsec': msg.stamp.nanosec},
            'label': msg.label,
            'clip_relpath': f'samples/{rel_name}',
            'fps': int(msg.fps),
            'frames': int(msg.frames),
            'source': msg.source,
        }
        with self.te_log.open('a', encoding='utf-8') as f:
            f.write(json.dumps(rec) + '\n')

        self.get_logger().info(f'stored training example → {rec["clip_relpath"]} ({msg.label})')


def main():
    rclpy.init()
    node = CentralDBNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

