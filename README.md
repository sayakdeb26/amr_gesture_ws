# AMR Gesture Control Workspace

This workspace contains a complete pipeline for controlling an iRobot Create3 robot using hand gestures detected by a VLM (Video-LLaVA). It supports both real hardware and Gazebo simulation.

## 1. Simulation with iRobot Create3 + Gesture Control

This section explains how to set up and run the full simulation on a new machine.

### A) Environment Setup

1.  **Install Ubuntu 22.04 & ROS 2 Humble**
    *   Follow the official [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2.  **Install Gazebo & Integration Packages**
    ```bash
    sudo apt-get update
    sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-ros-ign-bridge
    ```
    *Note: If using Gazebo Classic, install `ros-humble-gazebo-ros-pkgs`. If using Ignition/Gazebo Sim, ensure `ign` or `gz` is installed.*

3.  **Install iRobot Create3 Simulation**
    ```bash
    # Create workspace directory
    mkdir -p ~/amr_gesture_ws/src
    cd ~/amr_gesture_ws/src
    
    # Clone repositories
    git clone https://github.com/iRobotEducation/create3_sim.git -b humble
    git clone https://github.com/iRobotEducation/irobot_create_msgs.git -b humble
    
    # Install dependencies
    cd ~/amr_gesture_ws
    rosdep install --from-paths src -y --ignore-src
    ```

4.  **Clone This Workspace**
    *   Copy/clone the contents of this repo into `~/amr_gesture_ws/src`.

5.  **Build Everything**
    ```bash
    cd ~/amr_gesture_ws
    colcon build --symlink-install --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    ```

### B) Paths That May Need Adjustment

Some nodes use absolute paths that might need updating on a new machine:

*   **VLM Model**: `src/vlm_videollava_pkg/vlm_videollava_pkg/video_llava_node.py`
    *   Check `model_path` variable.
*   **LSTM Model**: `src/gesture_classifiers_pkg/config/lstm_params.yaml`
    *   Check `model_path`.
*   **UI Kiosk Media**: `src/ui_kiosk_pkg/config/kiosk_params.yaml`
    *   Check `media_dir` (default: `~/amr_gesture_ws/data/runtime_clips`).
*   **Recorder Output**: `src/vlm_recorder_pkg/config/recorder_params.yaml`
    *   Check `save_dir`.

### C) Sourcing and Running

**1. Source the Workspace**
Always run this in every new terminal:
```bash
source /opt/ros/humble/setup.bash
cd ~/amr_gesture_ws
source install/setup.bash
```

**2. Run JUST the Simulation**
To verify the robot spawns in Gazebo:
```bash
ros2 launch create3_sim_integration create3_gazebo_sim.launch.py
```

**3. Run JUST the Gesture Pipeline**
To verify camera and VLM without simulation:
```bash
# Set domain ID if connecting to real robot (e.g., 95)
# export ROS_DOMAIN_ID=95 
ros2 launch vlm_ros pipeline.launch.py
```

**4. Run EVERYTHING (Sim + Gestures)**
To control the simulated robot with gestures:
```bash
ros2 launch vlm_ros amr_gesture_create3_sim.launch.py
```

### D) Using Antigravity

If you are setting this up on a new device with Antigravity (Google's AI coding assistant), you can simply ask it to:
> "Fix paths, reinstall dependencies, and regenerate missing configs for this workspace."

It can parse this README and automate the setup steps for you.

## 2. Real Robot Usage

To control a real iRobot Create3:

1.  Connect to the same Wi-Fi network.
2.  Set the correct Domain ID (e.g., 95):
    ```bash
    export ROS_DOMAIN_ID=95
    ```
3.  Run the pipeline:
    ```bash
    ros2 launch vlm_ros pipeline.launch.py
    ```
