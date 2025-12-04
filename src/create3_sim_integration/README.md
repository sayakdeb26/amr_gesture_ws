# Create3 Simulation Integration

This package provides a simplified launch file to spawn the iRobot Create3 robot in Gazebo (Ignition) for simulation.

## 1. Setup & Installation

### Prerequisites
Ensure you have the necessary system dependencies installed (requires sudo):
```bash
sudo apt-get update
sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-ros-ign-bridge \
                     ros-humble-ros2-control ros-humble-ros2-controllers \
                     ros-humble-joint-state-publisher-gui
```

### Build Instructions
Build the workspace with the necessary CMake flags:
```bash
cd ~/amr_gesture_ws
colcon build --symlink-install --packages-select create3_sim_integration --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source install/setup.bash
```

## 2. Normal Operation

### Launching the Simulation
To launch Gazebo with the Create3 robot spawned in the `/AMR` namespace:

```bash
ros2 launch create3_sim_integration create3_gazebo_sim.launch.py
```

**Note on GPU Usage:**
This launch file is configured to force GPU acceleration:
- `IGN_RENDER_ENGINE=ogre2` (Uses Ogre2 rendering engine)
- `LIBGL_ALWAYS_SOFTWARE=0` (Disables software rendering)

### Controlling the Robot
Once the simulation is running, you can control the robot using ROS 2 topics.

**Move Forward:**
```bash
ros2 topic pub /AMR/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

**Rotate:**
```bash
ros2 topic pub /AMR/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

### Visualizing in RViz
The launch file automatically starts RViz. You should see the robot model and can visualize sensor data like:
- **Odom**: `/AMR/odom`
- **TF**: `/AMR/tf`
- **Lidar/Scan**: `/AMR/scan` (if equipped/simulated)

## 3. Troubleshooting

*   **"No module named yaml"**: Do not run the launch file with `python`. Always use `ros2 launch`.
*   **Black Screen / No Rendering**: Ensure your GPU drivers are up to date. The launch file forces `ogre2`; if this fails, you may need to check your Ignition Gazebo installation.
