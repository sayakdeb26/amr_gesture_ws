# AMR Telemetry Package

This package handles the translation of high-level gesture intents into robot commands.

## Features

- **Telemetry Command**: Publishes `TelemetryCommand` messages to `/telemetry/command` for UI feedback.
- **Motion Control**: Publishes `geometry_msgs/Twist` messages to drive the robot (real or simulated).

## Simulation Support

The telemetry node can drive a simulated Create3 robot in Gazebo.

### Configuration

The command velocity topic is configurable via the `cmd_vel_topic` parameter.

- Default: `/AMR/cmd_vel` (matches standard Create3 namespace)

To change it (e.g., for a different simulation setup):

```bash
ros2 run amr_telemetry_pkg telemetry_node --ros-args -p cmd_vel_topic:=/my_robot/cmd_vel
```

### Motion Mappings

| Gesture | Motion | Speed | Duration |
|---------|--------|-------|----------|
| SWIPE_UP | Forward | 0.20 m/s | 1.0 s |
| SWIPE_DOWN | Backward | -0.20 m/s | 1.0 s |
| SWIPE_LEFT | Rotate Left | 0.5 rad/s | 0.5 s |
| SWIPE_RIGHT | Rotate Right | -0.5 rad/s | 0.5 s |
