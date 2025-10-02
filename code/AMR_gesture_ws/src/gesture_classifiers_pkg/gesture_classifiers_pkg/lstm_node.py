#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

from amr_interfaces.msg import (
    KeypointsWindow,
    Intent,
    UnknownGesture,
)

def now_stamp(node: Node) -> Time:
    return node.get_clock().now().to_msg()

class LSTMNode(Node):
    def __init__(self):
        super().__init__('lstm_node')

        self.declare_parameter('conf_threshold', 0.80)
        self.declare_parameter('min_frames', 30)
        self.declare_parameter('normalize', True)

        self.conf_threshold: float = float(self.get_parameter('conf_threshold').value)
        self.min_frames: int = int(self.get_parameter('min_frames').value)
        self.normalize: bool = bool(self.get_parameter('normalize').value)

        self.pub_intent = self.create_publisher(Intent, '/intents_raw', 10)
        self.pub_unknown = self.create_publisher(UnknownGesture, '/lstm/unknown', 10)

        self.sub_kp = self.create_subscription(KeypointsWindow, '/lstm/keypoints_window', self.on_keypoints, 10)

        self.get_logger().info(
            f'lstm_node up. conf_threshold={self.conf_threshold:.2f}, '
            f'min_frames={self.min_frames}, normalize={self.normalize}'
        )

    def on_keypoints(self, msg: KeypointsWindow):
        # Basic guards
        if msg.frames < self.min_frames:
            self.get_logger().warn(f'Keypoints frames={msg.frames} < min_frames={self.min_frames}, sending unknown')
            self.publish_unknown(hint='wave')
            return

        J = max(int(msg.joints_per_frame), 1)
        F = max(int(msg.frames), 1)
        expect = J * F
        if len(msg.data) < expect:
            self.get_logger().warn(f'Keypoints length={len(msg.data)} < expected={expect}, sending unknown')
            self.publish_unknown(hint='wave')
            return

        # Compute a super-simple motion/energy score as a placeholder “confidence”
        # Data is row-major: [f0 j0..jJ-1, f1 j0..jJ-1, ...]
        # We take per-joint variance across frames and aggregate.
        # This roughly correlates with visible motion vs. no-hands/garbage.
        mean_per_joint = [0.0] * J
        for f in range(F):
            base = f * J
            row = msg.data[base:base + J]
            for j in range(J):
                mean_per_joint[j] += row[j]
        mean_per_joint = [m / F for m in mean_per_joint]

        var_sum = 0.0
        for f in range(F):
            base = f * J
            row = msg.data[base:base + J]
            for j in range(J):
                d = row[j] - mean_per_joint[j]
                var_sum += d * d

        # Normalize by F and J so the scale is stable across windows
        energy = var_sum / (F * J)

        # Map energy to [0,1] as a pseudo-confidence (tunable)
        # A gentle logistic squashing
        conf = 1.0 / (1.0 + math.exp(-10.0 * (energy - 0.02)))

        if conf >= self.conf_threshold:
            # High-confidence “WAVE_STOP” placeholder; later replace with real model label
            it = Intent()
            it.stamp = now_stamp(self)
            it.label = 'WAVE_STOP'
            it.confidence = float(conf)
            it.latency_ms = 0
            it.source = 'lstm'
            self.pub_intent.publish(it)
            self.get_logger().info(f'High-conf intent: {it.label} ({it.confidence:.2f}) J={J} F={F} energy={energy:.4f}')
        else:
            self.get_logger().info(f'Low-conf (conf={conf:.2f} < {self.conf_threshold:.2f}); handing to VLM')
            self.publish_unknown(hint='wave')

    def publish_unknown(self, hint: str):
        unk = UnknownGesture()
        unk.stamp = now_stamp(self)
        unk.confidence = 0.25
        unk.window_frames = self.min_frames
        unk.source = 'lstm'
        unk.hint = hint
        self.pub_unknown.publish(unk)

def main():
    rclpy.init()
    node = LSTMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

