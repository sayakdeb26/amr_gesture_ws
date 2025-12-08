# AMR Gesture Control Workspace

A complete ROS 2 Humble pipeline for controlling an iRobot Create3 robot using hand gestures. Supports both Gazebo simulation and real hardware.

---

## Quick Start

```bash
# Source workspace
source /opt/ros/humble/setup.bash
cd ~/amr_gesture_ws
source install/setup.bash

# Run simulation + gesture control (FastVLM - faster)
ros2 launch vlm_ros amr_gesture_create3_sim.launch.py

# OR with Video-LLaVA (more accurate)
ros2 launch vlm_ros amr_gesture_create3_sim_videollava.launch.py
```

---

## Supported Gestures

| Gesture | Action | Description |
|---------|--------|-------------|
| 👍 **THUMB_UP** | Activate | Enable gesture control |
| 👎 **THUMB_DOWN** | Deactivate | Disable gesture control |
| ⬆️ **SWIPE_UP** | Move Forward | 0.2 m/s for 1 second |
| ⬇️ **SWIPE_DOWN** | Move Backward | -0.2 m/s for 1 second |
| ⬅️ **SWIPE_LEFT** | Turn Left | Rotate at +0.5 rad/s |
| ➡️ **SWIPE_RIGHT** | Turn Right | Rotate at -0.5 rad/s |
| 🔄 **ROLL_BACK** | 180° Turn | Full turnaround |

---

## VLM Options

| VLM | Speed | Accuracy | Model | Launch File |
|-----|-------|----------|-------|-------------|
| **FastVLM** | ~1s | Good | `apple/FastVLM-1.5B` | `amr_gesture_create3_sim.launch.py` |
| **Video-LLaVA** | ~15s | Better | `LanguageBind/Video-LLaVA-7B-hf` | `amr_gesture_create3_sim_videollava.launch.py` |

---

## Installation

### Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo (Ignition Fortress)
- Python 3.10+

### Step 1: Install ROS 2 Humble
Follow the official [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

### Step 2: Install Gazebo & Integration Packages
```bash
sudo apt-get update
sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-ros-ign-bridge
```

### Step 3: Install iRobot Create3 Simulation
```bash
mkdir -p ~/amr_gesture_ws/src
cd ~/amr_gesture_ws/src

# Clone iRobot packages
git clone https://github.com/iRobotEducation/create3_sim.git -b humble
git clone https://github.com/iRobotEducation/irobot_create_msgs.git -b humble

# Install dependencies
cd ~/amr_gesture_ws
rosdep install --from-paths src -y --ignore-src
```

### Step 4: Clone This Repository
```bash
cd ~/amr_gesture_ws/src
git clone <this-repo-url> .
```

### Step 5: Build Everything
```bash
cd ~/amr_gesture_ws
colcon build --symlink-install
```

---

## Architecture

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Webcam    │───▶│  LSTM Node  │───▶│  Telemetry  │───▶│  /cmd_vel   │
└─────────────┘    └──────┬──────┘    └─────────────┘    └─────────────┘
                          │
                   Unknown Gesture?
                          │
                          ▼
                   ┌─────────────┐    ┌─────────────┐
                   │ VLM Bridge  │───▶│ FastVLM or  │
                   │             │    │ Video-LLaVA │
                   └──────┬──────┘    └─────────────┘
                          │
                          ▼
                   ┌─────────────┐
                   │  UI Kiosk   │ ◀── http://localhost:8008
                   │(Confirmation)│
                   └─────────────┘
```

---

## Key Packages

| Package | Description |
|---------|-------------|
| `gesture_classifiers_pkg` | LSTM gesture recognition with MediaPipe + face detection filter |
| `vlm_ros` | FastVLM integration (HuggingFace) |
| `vlm_videollava_pkg` | Video-LLaVA integration |
| `vlm_bridge_pkg` | Handles unknown gestures, triggers VLM |
| `vlm_recorder_pkg` | Records video clips for VLM analysis |
| `amr_telemetry_pkg` | Converts gestures to robot velocities |
| `ui_kiosk_pkg` | Web UI for operator confirmation |
| `create3_sim_integration` | Gazebo simulation for iRobot Create3 |
| `frame_simplifier_pkg` | Downsamples video for efficient processing |

---

## Models Required

| Model | Path | Notes |
|-------|------|-------|
| **LSTM Gesture Model** | `models/lstm/jester20b_12cls/final_jester_model.onnx` | Pre-trained on Jester dataset |
| **MediaPipe Hands** | `models/mediapipe/hand_landmarker.task` | Hand landmark detection |
| **VLM Models** | Auto-download | Downloaded from HuggingFace on first run |

---

## Running Options

### 1. Full Simulation + Gestures (Main Demo)
```bash
ros2 launch vlm_ros amr_gesture_create3_sim.launch.py
```
This launches:
- Gazebo simulation with iRobot Create3
- Webcam gesture detection
- VLM fallback for unknown gestures
- Web UI at http://localhost:8008

### 2. Just the Simulation
```bash
ros2 launch create3_sim_integration create3_gazebo_sim.launch.py
```

### 3. Just the Gesture Pipeline (No Sim)
```bash
ros2 launch vlm_ros pipeline.launch.py
```

### 4. Real Robot
```bash
export ROS_DOMAIN_ID=95  # Match your robot's domain
ros2 launch vlm_ros pipeline.launch.py
```

---

## Configuration

### Paths That May Need Adjustment

| File | Variable | Default |
|------|----------|---------|
| `src/vlm_videollava_pkg/.../video_llava_node.py` | `model_id` | `LanguageBind/Video-LLaVA-7B-hf` |
| `src/vlm_ros/launch/pipeline.launch.py` | `model_path` | `/home/sayak/amr_gesture_ws/models/lstm/...` |
| `src/vlm_recorder_pkg/.../recorder_node.py` | `save_dir` | `~/amr_gesture_ws/data/runtime_clips` |

### VLM CPU vs GPU Mode
Edit `src/vlm_videollava_pkg/vlm_videollava_pkg/video_llava_node.py`:

```python
# CPU Mode (Default) - Slower but stable
self.device = "cpu"
dtype = torch.float32

# GPU Mode - Faster, requires 12GB+ VRAM
self.device = "cuda"
dtype = torch.float16
```

---

## UI Kiosk

Access at **http://localhost:8008** when the pipeline is running.

Features:
- Live camera feed
- Current gesture display
- VLM confirmation dialogs
- Robot status indicators

---

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| **"Switch controller timed out"** | Old URDF plugin | Ensure `gz_ros2_control` in URDFs, rebuild `irobot_create_description` |
| **Robot won't move backward** | Safety system active | Set `safety_override: full` in `create3_nodes.launch.py` |
| **Pipeline not resuming after confirm** | Topic mismatch | Ensure `ui_kiosk_node.py` publishes to `/ui/confirm_reply` |
| **VLM returns "ERROR"** | OOM on GPU | Switch to CPU mode |
| **No camera feed** | Device path wrong | Check `/dev/video0` exists |
| **Face detected as hand** | False positive | Face filter should reject - check logs |
| **Robot doesn't respond** | Not activated | Do THUMB_UP gesture first |

---

## License

Apache 2.0
