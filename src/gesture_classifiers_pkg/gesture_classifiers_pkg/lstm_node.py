#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from amr_interfaces.msg import UnknownGesture
from builtin_interfaces.msg import Time

class LSTMNode(Node):
    def __init__(self):
        super().__init__(
            'lstm_node',
            # optional: uncomment if you also want to allow YAML/launch to auto-declare:
            # automatically_declare_parameters_from_overrides=True
        )

        # Declare parameters with defaults
        self.declare_parameter('tau', 0.60)
        self.declare_parameter('window_frames', 40)

        # Cache values
        self.tau = float(self.get_parameter('tau').value)
        self.window_frames = int(self.get_parameter('window_frames').value)

        # React to runtime updates
        self.add_on_set_parameters_callback(self._on_param_set)

        # Publisher(s)
        self.unknown_pub = self.create_publisher(UnknownGesture, '/lstm/unknown', 10)

        self.get_logger().info(f'lstm_node up. τ={self.tau:.2f}, window={self.window_frames} frames')

        # TODO: your subscriptions / timers that compute confidence go here
        # Example timer that does nothing but demonstrate publishing “unknown”
        # self.create_timer(5.0, self._demo_tick)

    def _on_param_set(self, params):
        for p in params:
            if p.name == 'tau':
                try:
                    self.tau = float(p.value)
                    self.get_logger().info(f'param updated: tau={self.tau:.2f}')
                except Exception as e:
                    self.get_logger().warn(f'bad tau: {e}')
                    return SetParametersResult(successful=False)
            elif p.name == 'window_frames':
                try:
                    self.window_frames = int(p.value)
                    self.get_logger().info(f'param updated: window_frames={self.window_frames}')
                except Exception as e:
                    self.get_logger().warn(f'bad window_frames: {e}')
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    # Optional demo publisher (remove if not needed)
    def _demo_tick(self):
        msg = UnknownGesture(
            stamp=self.get_clock().now().to_msg(),
            confidence=0.42,
            window_frames=self.window_frames,
            source='lstm',
            hint='demo'
        )
        self.unknown_pub.publish(msg)

def main():
    rclpy.init()
    node = LSTMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
