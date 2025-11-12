#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Frame Simplifier + Keypoints XY Bridge (single node, no extra processes)

- Subscribes:  /image_raw   (sensor_msgs/Image)
- Publishes:   /image_raw_30hz  (same Image, rate-limited)

- Subscribes:  /lstm/keypoints_window        (vlm_interfaces/KeypointsWindow)
- Publishes:   /lstm/keypoints_window_xy     (vlm_interfaces/KeypointsWindow)  # x,y only

- Listens:     /pipeline/hold (std_msgs/Bool) to pause/resume the pipeline

Parameters:
  fps (double, default: 10.0)         → output rate for the image relay
  paused (bool, default: false)       → start paused or not
  emit_xy (bool, default: true)       → turn on the XY bridge
  image_in (string, default: '/image_raw')
  image_out (string, default: '/image_raw_30hz')
  kpx_in (string, default: '/lstm/keypoints_window')
  kpx_out (string, default: '/lstm/keypoints_window_xy')
"""

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Bool
from sensor_msgs.msg import Image

# Adjust this import if your message lives elsewhere,
# but the fields used below must exist:
#   stamp (builtin_interfaces/Time)
#   frames (int32)
#   joints_per_frame (int32)
#   data (sequence<float>)
#   source (string)
from vlm_interfaces.msg import KeypointsWindow


class FrameSimplifierNode(Node):
    def __init__(self):
        super().__init__('frame_simplifier')

        # --- Parameters ------------------------------------------------------
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('paused', False)
        self.declare_parameter('emit_xy', True)
        self.declare_parameter('image_in', '/image_raw')
        self.declare_parameter('image_out', '/image_raw_30hz')
        self.declare_parameter('kpx_in', '/lstm/keypoints_window')
        self.declare_parameter('kpx_out', '/lstm/keypoints_window_xy')

        self._fps = float(self.get_parameter('fps').value)
        self._paused = bool(self.get_parameter('paused').value)
        self._emit_xy = bool(self.get_parameter('emit_xy').value)

        self._image_in = str(self.get_parameter('image_in').value)
        self._image_out = str(self.get_parameter('image_out').value)
        self._kpx_in = str(self.get_parameter('kpx_in').value)
        self._kpx_out = str(self.get_parameter('kpx_out').value)

        self.add_on_set_parameters_callback(self._on_set_params)

        # --- QoS -------------------------------------------------------------
        # Images: best effort, small queue (typical camera QoS)
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Lightweight QoS for control/keys
        ctrl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        kpx_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # --- Image downsampler ----------------------------------------------
        self._img_pub = self.create_publisher(Image, self._image_out, image_qos)
        self._img_sub = self.create_subscription(Image, self._image_in, self._on_image, image_qos)

        # pause control via /pipeline/hold
        self.create_subscription(Bool, '/pipeline/hold', self._on_hold, ctrl_qos)

        # --- Keypoints XY bridge (optional) ----------------------------------
        self._kpx_sub = None
        self._kpx_pub = None
        if self._emit_xy:
            self._kpx_pub = self.create_publisher(KeypointsWindow, self._kpx_out, kpx_qos)
            self._kpx_sub = self.create_subscription(KeypointsWindow, self._kpx_in, self._on_kpx, kpx_qos)

        # --- Timing ----------------------------------------------------------
        self._last_pub_time = self.get_clock().now()

        self.get_logger().info(
            f"Simplifier up: {self._image_in} → {self._image_out} @ {self._fps:.1f} fps"
        )
        self.get_logger().info(f"KPX bridge: {self._emit_xy} ({self._kpx_in} → {self._kpx_out})")
        self.get_logger().info(f"pause={self._paused}")

    # ----------------------- Parameter handling ------------------------------
    def _on_set_params(self, params: List[Parameter]):
        for p in params:
            if p.name == 'fps' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                self._fps = float(p.value)
                self.get_logger().info(f"fps -> {self._fps:.2f}")
            elif p.name == 'paused' and p.type_ == Parameter.Type.BOOL:
                self._paused = bool(p.value)
                self.get_logger().info(f"paused -> {self._paused}")
            elif p.name == 'emit_xy' and p.type_ == Parameter.Type.BOOL:
                new_emit = bool(p.value)
                if new_emit != self._emit_xy:
                    self._emit_xy = new_emit
                    # (re)wire the KPX bridge according to toggle
                    self._rewire_kpx()
                    self.get_logger().info(f"emit_xy -> {self._emit_xy}")
        return SetParametersResult(successful=True)

    def _rewire_kpx(self):
        # Destroy existing endpoints
        if self._kpx_sub is not None:
            self.destroy_subscription(self._kpx_sub)
            self._kpx_sub = None
        if self._kpx_pub is not None:
            self.destroy_publisher(self._kpx_pub)
            self._kpx_pub = None

        if self._emit_xy:
            kpx_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self._kpx_pub = self.create_publisher(KeypointsWindow, self._kpx_out, kpx_qos)
            self._kpx_sub = self.create_subscription(KeypointsWindow, self._kpx_in, self._on_kpx, kpx_qos)

    # --------------------------- Callbacks -----------------------------------
    def _on_hold(self, msg: Bool):
        prev = self._paused
        self._paused = bool(msg.data)
        if self._paused != prev:
            self.get_logger().info(f"pause={self._paused}")

    def _on_image(self, msg: Image):
        if self._paused or self._fps <= 0.0:
            return
        now = self.get_clock().now()
        elapsed = (now - self._last_pub_time).nanoseconds / 1e9
        period = 1.0 / self._fps
        if elapsed >= period:
            # pass-through: no copy, just republish the message as-is
            self._img_pub.publish(msg)
            self._last_pub_time = now

    def _on_kpx(self, msg: KeypointsWindow):
        # Expect flat layout (T * J * 3)
        T = int(msg.frames)
        J = int(msg.joints_per_frame)
        flat = list(msg.data) if msg.data is not None else []

        expected = T * J * 3
        if len(flat) != expected:
            # Unexpected size — publish as-is (avoid breaking downstream)
            self.get_logger().warn(
                f"KeypointsWindow length {len(flat)} != T*J*3 ({T}*{J}*3={expected}); passing through."
            )
            out = msg
        else:
            # Take x,y (drop every 3rd value: z)
            xy = []
            # layout: [x,y,z, x,y,z, ...]
            for i in range(0, len(flat), 3):
                xy.append(flat[i])       # x
                xy.append(flat[i + 1])   # y

            out = KeypointsWindow()
            out.stamp = msg.stamp
            out.frames = T
            out.joints_per_frame = J
            out.data = xy
            # preserve source and annotate
            try:
                src = (msg.source or '').strip()
            except Exception:
                src = ''
            out.source = (src if src else 'mediapipe') + '_xy'

        self._kpx_pub.publish(out)


def main():
    rclpy.init()
    node = FrameSimplifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

