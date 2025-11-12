#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
import math
from rclpy.node import Node
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from time import monotonic

class FrameSimplifier(Node):
    def __init__(self):
        super().__init__('frame_simplifier')

        # Params
        self.declare_parameter('target_fps', 30.0)   # double so "30" or "30.0" both work
        self._target_fps = float(self.get_parameter('target_fps').value)
        self._min_dt = 1.0 / max(1.0, self._target_fps)

        self._paused = False
        self._last_pub_t = 0.0

        # Topics (use relative names so CLI remaps are clean)
        in_topic  = '/image_in'
        out_topic = '/image_out'

        # I/O
        self._sub_img = self.create_subscription(
            Image, in_topic, self._on_image, qos_profile_sensor_data
        )
        self._pub_img = self.create_publisher(
            Image, out_topic, qos_profile_sensor_data
        )

        # Optional hold/pause line
        self._sub_hold = self.create_subscription(
            Bool, '/pipeline/hold', self._on_hold, 10
        )

        self.get_logger().info(
            f"Simplifier up: {in_topic} → {out_topic} @ {self._target_fps:.0f} fps (no KPX/LSTM links)"
        )

    def _on_hold(self, msg: Bool):
        self._paused = bool(msg.data)
        self.get_logger().info(f"pause={self._paused}")

    def _on_image(self, msg: Image):
        if self._paused:
            return
        t = monotonic()
        if (t - self._last_pub_t) < self._min_dt:
            return
        self._last_pub_t = t
        # Straight passthrough of the frame
        self._pub_img.publish(msg)

def main():
    rclpy.init()
    node = FrameSimplifier()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
