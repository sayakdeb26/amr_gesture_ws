#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from amr_interfaces.msg import Intent, TelemetryCommand
from geometry_msgs.msg import Twist
import time

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/AMR/cmd_vel')
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Motion mappings: gesture -> (vx, wz, duration)
        # SWIPE_UP -> move forward at +0.20 m/s for 1.0s
        # SWIPE_DOWN -> move backward at -0.20 m/s for 1.0s
        # SWIPE_LEFT -> rotate left +0.5 rad/s for 0.5s
        # SWIPE_RIGHT -> rotate right -0.5 rad/s for 0.5s
        self.motion_profiles = {
            "SWIPE_UP": (0.20, 0.0, 1.0),
            "SWIPE_DOWN": (-0.20, 0.0, 1.0),
            "SWIPE_LEFT": (0.0, 0.5, 0.5),
            "SWIPE_RIGHT": (0.0, -0.5, 0.5),
            "THUMB_UP": (0.0, 0.0, 0.0), # Stop/Confirm (or maybe move forward?) - User context implies approval, maybe no motion?
            "THUMB_DOWN": (0.0, 0.0, 0.0), # Stop/Reject
            "STOP_SIGN": (0.0, 0.0, 0.0)   # Explicit Stop
        }

        # Command text mapping for TelemetryCommand
        self.command_text = {
            "SWIPE_UP": "move_forward_1s",
            "SWIPE_DOWN": "move_backward_1s",
            "SWIPE_LEFT": "turn_left_15deg",
            "SWIPE_RIGHT": "turn_right_15deg",
            "THUMB_UP": "confirm_action",
            "THUMB_DOWN": "reject_action",
            "STOP_SIGN": "stop_robot"
        }

        # Active motion state
        self.active_motion = None  # (vx, wz, end_time)

        # Subscribers
        self.sub_intent = self.create_subscription(
            Intent,
            '/intents_raw',
            self.intent_callback,
            10)

        # Publishers
        self.pub_telemetry = self.create_publisher(TelemetryCommand, '/telemetry/command', 10)
        self.pub_twist = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # 20 Hz timer for Twist publishing
        self.timer = self.create_timer(0.05, self.timer_callback)  # 50ms = 20Hz

        self.get_logger().info('Telemetry Node started (with Create3 motion control)')

    def intent_callback(self, msg):
        label = msg.label

        # Only process if label is in our mapping
        if label in self.motion_profiles:
            # Get motion profile
            vx, wz, duration = self.motion_profiles[label]

            # Publish TelemetryCommand (existing functionality)
            cmd = TelemetryCommand()
            cmd.stamp = msg.stamp
            cmd.session_id = msg.session_id
            cmd.label = label
            cmd.command_text = self.command_text[label]
            cmd.category = "MOVEMENT"
            cmd.source = "telemetry_node"
            self.pub_telemetry.publish(cmd)

            # Start timed motion
            self.active_motion = (vx, wz, time.time() + duration)

            # Log motion start
            self.get_logger().info(
                f"telemetry: {label} -> {self.command_text[label]} "
                f"(vx={vx:.2f}, wz={wz:.2f}, duration={duration:.1f}s)"
            )

    def timer_callback(self):
        """20 Hz timer to publish Twist commands during active motions"""
        if self.active_motion is None:
            return

        vx, wz, end_time = self.active_motion

        if time.time() < end_time:
            # Motion still active, publish velocity
            twist = Twist()
            twist.linear.x = vx
            twist.angular.z = wz
            self.pub_twist.publish(twist)
        else:
            # Motion completed, send stop command
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            # Log motion stop
            self.get_logger().info("Motion completed, robot stopped")

            # Clear active motion
            self.active_motion = None

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
