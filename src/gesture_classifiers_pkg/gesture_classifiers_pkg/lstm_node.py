#!/usr/bin/env python3
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from builtin_interfaces.msg import Time

try:
    import numpy as np
except Exception:
    np = None

from amr_interfaces.msg import KeypointsWindow, Intent, UnknownGesture


def now_stamp(node: Node) -> Time:
    return node.get_clock().now().to_msg()


class LstmNode(Node):
    def __init__(self):
        super().__init__('lstm_node')
        qos = QoSProfile(depth=10)

        # Params
        self.declare_parameter('conf_threshold', 0.80)
        self.declare_parameter('min_frames', 30)
        self.declare_parameter('normalize', True)
        self.declare_parameter('label', 'WAVE_STOP')

        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.min_frames = int(self.get_parameter('min_frames').value)
        self.normalize = bool(self.get_parameter('normalize').value)
        self.label = str(self.get_parameter('label').value)

        self.pub_intent = self.create_publisher(Intent, '/intents_raw', qos)
        self.pub_unknown = self.create_publisher(UnknownGesture, '/lstm/unknown', qos)
        self.sub_kp = self.create_subscription(KeypointsWindow, '/lstm/keypoints_window', self.on_kp, qos)

        self.get_logger().info(f'lstm_node up. conf_threshold={self.conf_threshold:.2f}, min_frames={self.min_frames}, normalize={self.normalize}')

    def on_kp(self, msg: KeypointsWindow):
        if msg.frames < self.min_frames:
            return
        data = list(msg.data)
        if len(data) == 0:
            return

        # Heuristic confidence on motion
        conf = self._conf_from_motion(data, msg.frames, msg.joints_per_frame, self.normalize)

        if conf >= self.conf_threshold:
            it = Intent()
            it.stamp = now_stamp(self)
            it.label = self.label
            it.confidence = float(conf)
            it.latency_ms = 0
            it.source = 'lstm_skeleton'
            self.pub_intent.publish(it)
            self.get_logger().info(f'Intent: {it.label} ({it.confidence:.2f})')
        else:
            unk = UnknownGesture()
            unk.stamp = now_stamp(self)
            unk.confidence = float(conf)
            unk.window_frames = msg.frames
            unk.source = 'lstm_skeleton'
            unk.hint = 'low_conf'
            self.pub_unknown.publish(unk)
            self.get_logger().info(f'Unknown (conf={conf:.2f})')

    def _conf_from_motion(self, flat: list, frames: int, joints_per_frame: int, normalize: bool) -> float:
        # flat = [f0_j0x, f0_j0y, f0_j0z, ..., fF_jJz]
        dims = 3
        if joints_per_frame == 0 or frames == 0:
            return 0.0
        points_per_frame = joints_per_frame * dims
        if len(flat) < frames * points_per_frame:
            frames = len(flat) // points_per_frame

        if np:
            arr = np.array(flat[:frames * points_per_frame], dtype=np.float32).reshape(frames, joints_per_frame, dims)
            if normalize:
                mn = arr.mean(axis=(0, 1), keepdims=True)
                sd = arr.std(axis=(0, 1), keepdims=True) + 1e-6
                arr = (arr - mn) / sd
            vel = np.diff(arr, axis=0)  # (frames-1, joints, 3)
            mag = np.linalg.norm(vel, axis=2)  # (frames-1, joints)
            motion = float(mag.mean()) if mag.size else 0.0
            # Map motion -> confidence in [0,1] with soft saturation
            conf = 1.0 - math.exp(-motion)
            return max(0.0, min(1.0, conf))
        else:
            # Fallback without numpy
            motion_sum = 0.0
            count = 0
            for f in range(1, frames):
                base_a = (f - 1) * points_per_frame
                base_b = f * points_per_frame
                for j in range(joints_per_frame):
                    i = j * dims
                    ax, ay, az = flat[base_a + i], flat[base_a + i + 1], flat[base_a + i + 2]
                    bx, by, bz = flat[base_b + i], flat[base_b + i + 1], flat[base_b + i + 2]
                    if normalize:
                        # Simple mean/std-free scale guard
                        pass
                    dx, dy, dz = (bx - ax), (by - ay), (bz - az)
                    motion_sum += math.sqrt(dx * dx + dy * dy + dz * dz)
                    count += 1
            motion = (motion_sum / count) if count else 0.0
            conf = 1.0 - math.exp(-motion)
            return max(0.0, min(1.0, conf))


def main():
    rclpy.init()
    node = LstmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


