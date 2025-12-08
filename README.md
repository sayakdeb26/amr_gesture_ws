# AMR Gesture Control Workspace

A complete ROS 2 Humble pipeline for controlling an iRobot Create3 robot using hand gestures. Supports both Gazebo simulation and real hardware with integrated VLM fallback for unknown gestures.

---

## Features

✅ **Real-time hand gesture recognition** using MediaPipe + LSTM  
✅ **Face detection filter** to prevent false hand detections  
✅ **FastVLM integration** for unknown gesture classification  
✅ **THUMB_UP/THUMB_DOWN** activation/deactivation  
✅ **cv2.imshow debug window** with real-time visualization  
✅ **Smart cooldown** (only for movement gestures)  
✅ **Gazebo simulation** with iRobot Create3  
✅ **Web UI** for operator confirmation at http://localhost:8008

---

## Quick Start

```bash
# Source workspace
source /opt/ros/humble/setup.bash
cd ~/amr_gesture_ws
source install/setup.bash

# Run simulation + gesture control
ros2 launch vlm_ros amr_gesture_create3_sim.launch.py
```

**What to expect:**
1. Gazebo simulation window launches with Create3 robot
2. "LSTM Debug Feed" OpenCV window appears showing hand tracking
3. Wait ~15 seconds for VLM model to load
4. Do **THUMB_UP** gesture to activate control
5. Perform gestures to move the robot

---

## Supported Gestures

### Control Gestures (No Cooldown)
| Gesture | Action | Description |
|---------|--------|-------------|
| 👍 **THUMB_UP** | Activate | Enable gesture control system |
| 👎 **THUMB_DOWN** | Deactivate | Disable gesture control system |

### Movement Gestures (2.5s Cooldown)
| Gesture | Motion | Velocity |
|---------|--------|----------|
| ⬆️ **SWIPE_UP** | Move Forward | 0.2 m/s for 1.0s |
| ⬇️ **SWIPE_DOWN** | Move Backward | -0.2 m/s for 1.0s |
| ⬅️ **SWIPE_LEFT** | Turn Left | +0.5 rad/s for 0.5s |
| ➡️ **SWIPE_RIGHT** | Turn Right | -0.5 rad/s for 0.5s |
| ✋ **STOP_SIGN** | Emergency Stop | Stop all motion |

### Ignored Gestures (Filtered Out)
- `NO_GESTURE` - No hand detected
- `IGNORE` - Model's background class

---

## Architecture

```
┌─────────────┐    ┌──────────────────┐    ┌─────────────┐    ┌─────────────┐
│   Webcam    │───▶│   LSTM Node      │───▶│  Telemetry  │───▶│  /cmd_vel   │
│ /image_raw  │    │ + MediaPipe      │    │    Node     │    │   (robot)   │
└─────────────┘    │ + Face Filter    │    └─────────────┘    └─────────────┘
                   └────────┬─────────┘
                            │
                     Unknown Gesture?
                            │
                            ▼
                   ┌─────────────────┐    ┌─────────────┐
                   │  VLM Bridge     │───▶│  FastVLM    │
                   │  + Recorder     │    │  (1.5B)     │
                   └────────┬────────┘    └─────────────┘
                            │
                            ▼
                   ┌─────────────────┐
                   │   UI Kiosk      │ ◀── http://localhost:8008
                   │ (Confirmation)  │
                   └─────────────────┘
```

---

## Installation

### Prerequisites
- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill
- **Gazebo:** Ignition Fortress (Gazebo 6)
- **Python:** 3.10+
- **GPU:** NVIDIA GPU with 8GB+ VRAM (optional, for faster VLM)

### Step 1: Install ROS 2 Humble
```bash
# Follow official guide
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
```

### Step 2: Install Dependencies
```bash
sudo apt-get update
sudo apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-ign-bridge \
    python3-pip \
    python3-opencv \
    ffmpeg

# Python dependencies
pip3 install \
    mediapipe \
    onnxruntime \
    numpy \
    torch torchvision \
    transformers \
    accelerate \
    flask
```

### Step 3: Clone Workspace
```bash
mkdir -p ~/amr_gesture_ws/src
cd ~/amr_gesture_ws/src
git clone <this-repo-url> .

# Clone iRobot dependencies
git clone https://github.com/iRobotEducation/create3_sim.git -b humble
git clone https://github.com/iRobotEducation/irobot_create_msgs.git -b humble
```

### Step 4: Install ROS Dependencies
```bash
cd ~/amr_gesture_ws
rosdep install --from-paths src -y --ignore-src
```

### Step 5: Build Workspace
```bash
cd ~/amr_gesture_ws
colcon build --symlink-install
```

### Step 6: Download Models
```bash
# LSTM gesture model (required)
mkdir -p ~/amr_gesture_ws/models/lstm/jester20b_12cls
# Copy your final_jester_model.onnx here

# MediaPipe hand model (required)
mkdir -p ~/amr_gesture_ws/models/mediapipe
# Download hand_landmarker.task from MediaPipe
# https://developers.google.com/mediapipe/solutions/vision/hand_landmarker

# VLM models auto-download on first run from HuggingFace
```

---

## Key Packages

| Package | Description | Key Features |
|---------|-------------|--------------|
| `gesture_classifiers_pkg` | LSTM gesture recognition | MediaPipe hands, face filter, debug window |
| `vlm_ros` | FastVLM integration | Apple FastVLM-1.5B via HuggingFace |
| `vlm_bridge_pkg` | Unknown gesture handler | Triggers VLM, manages sessions |
| `vlm_recorder_pkg` | Video clip recorder | Records 3s clips for VLM input |
| `amr_telemetry_pkg` | Gesture → velocity mapping | Publishes to `/cmd_vel` |
| `ui_kiosk_pkg` | Web UI server | Operator confirmation interface |
| `create3_sim_integration` | Gazebo simulation | iRobot Create3 in depot world |
| `frame_simplifier_pkg` | Video downsampler | 320x240 @ 10fps for efficiency |

---

## Debug Window

The LSTM node displays a real-time debug feed with:
- ✅ Hand landmarks with connection lines
- ✅ Gesture label and confidence
- ✅ FPS counter
- ✅ Frame buffer status (Frames: X/30)
- ✅ Motion status (Hand: MOVING/STATIONARY)
- ✅ Activation status (ACTIVE/IDLE)

**Keyboard Controls:**
- `Q` - Quit node
- `R` - Reset buffers

---

## Running Options

### 1. Full Simulation (Main Demo)
```bash
ros2 launch vlm_ros amr_gesture_create3_sim.launch.py
```
Launches:
- Gazebo with Create3 robot in depot environment
- Webcam gesture recognition with debug window
- VLM fallback for unknown gestures
- UI kiosk at http://localhost:8008

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
# Set domain to match your robot (check robot's web interface)
export ROS_DOMAIN_ID=95

# Launch pipeline
ros2 launch vlm_ros pipeline.launch.py
```

---

## Configuration

### Camera Settings
Edit `src/vlm_ros/launch/pipeline.launch.py`:
```python
DeclareLaunchArgument('camera_dev', default_value='/dev/video0'),
DeclareLaunchArgument('width', default_value='640'),
DeclareLaunchArgument('height', default_value='480'),
DeclareLaunchArgument('fps', default_value='30'),
```

### LSTM Model Path
Edit `src/vlm_ros/launch/pipeline.launch.py`:
```python
'model_path': '/home/sayak/amr_gesture_ws/models/lstm/jester20b_12cls/final_jester_model.onnx'
```

### Gesture Motion Profiles
Edit `src/amr_telemetry_pkg/amr_telemetry_pkg/telemetry_node.py`:
```python
self.motion_profiles = {
    "SWIPE_UP": (0.20, 0.0, 1.0),      # (vx, wz, duration)
    "SWIPE_DOWN": (-0.20, 0.0, 1.0),
    "SWIPE_LEFT": (0.0, 0.5, 0.5),
    "SWIPE_RIGHT": (0.0, -0.5, 0.5),
}
```

### Disable RViz
Already disabled by default. To re-enable:
```python
# In amr_gesture_create3_sim.launch.py
DeclareLaunchArgument('use_rviz', default_value='true')
```

---

## Face Detection Filter

The LSTM node includes a lightweight face detection filter to prevent MediaPipe from misidentifying faces or ears as hands.

### Rejection Rules
A hand detection is rejected if:
1. **Face overlap > 20%** - Bounding box overlaps with detected face
2. **Too small** - Width or height < 40 pixels
3. **Bad aspect ratio** - Aspect ratio < 0.2 (too compressed)
4. **Upper-center region** - y < 25% AND x between 30-70% (likely face area)

### Performance
- **Compute cost:** < 1ms per frame (negligible)
- **Model:** MediaPipe Face Detection (short-range, model 0)

---

## Cooldown Behavior

| Gesture Type | Cooldown | Rationale |
|--------------|----------|-----------|
| **Movement gestures** (SWIPE_*) | 2.5 seconds | Allow robot to complete motion |
| **Control gestures** (THUMB_UP/DOWN) | None | Instant response |
| **NO_GESTURE, IGNORE** | None | Filtered out |

---

## Topics Reference

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/image_raw` | `sensor_msgs/Image` | v4l2_camera | lstm_node | Raw webcam feed |
| `/frames/simplified` | `sensor_msgs/Image` | simplifier_node | recorder_node | Downsampled 320x240 @ 10fps |
| `/intents_raw` | `vlm_interfaces/Intent` | lstm_node | telemetry_node | Recognized gestures |
| `/cmd_vel` | `geometry_msgs/Twist` | telemetry_node | robot | Velocity commands |
| `/lstm/unknown` | `vlm_interfaces/UnknownGesture` | lstm_node | bridge_node | Unknown gestures |
| `/lstm/debug_feed` | `sensor_msgs/Image` | lstm_node | (optional) | Debug visualization |
| `/ui/confirm_reply` | `vlm_interfaces/ConfirmReply` | ui_kiosk | bridge/lstm | Operator feedback |

---

## Troubleshooting

### LSTM Node Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| **"Model not found"** | Wrong path | Check `model_path` parameter points to `final_jester_model.onnx` |
| **Face detected as hand** | False positive | Check logs for "Rejected false hand detection" messages |
| **No debug window** | Display issue | Ensure X11 forwarding or local display |
| **Low FPS (<20)** | Slow inference | Reduce camera resolution or use faster model |

### Robot Control Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| **Robot doesn't move** | Not activated | Do THUMB_UP gesture first |
| **Only turns, no fwd/back** | Safety override | Set `safety_override: full` in Create3 config |
| **Gestures ignored after movement** | Cooldown active | Wait 2.5 seconds between movement gestures |
| **No `/cmd_vel` topic** | Namespace mismatch | Check topics with `ros2 topic list` |

### VLM Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| **"Out of memory"** | GPU VRAM | Switch VLM to CPU mode (edit vlm_node.py) |
| **VLM takes >30s** | CPU inference | Use GPU or switch to FastVLM |
| **Model download fails** | Network/HuggingFace | Check internet, retry, or download manually |

### Gazebo Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| **"Waiting for world names"** | Slow startup | Wait 10-15 seconds, check Gazebo GUI opens |
| **Robot falls through floor** | Physics not loaded | Restart Gazebo, ensure depot.sdf loads |
| **Black screen in Gazebo** | GPU driver | Update NVIDIA drivers, check `IGN_RENDER_ENGINE=ogre2` |

### General Debugging

```bash
# Check active nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor gesture detections
ros2 topic echo /intents_raw

# Monitor robot commands
ros2 topic echo /cmd_vel

# Check node logs
ros2 run rqt_console rqt_console

# Kill stuck processes
pkill -f ros2
pkill -f gazebo
fuser -k 8008/tcp
```

---

## UI Kiosk

Access at **http://localhost:8008** when pipeline is running.

### Features
- 📹 Live camera feed preview
- 🎯 Current detected gesture display
- ✅ VLM confirmation dialogs for unknown gestures
- 📊 System status indicators
- ⏱️ 30-second countdown timer for operator response

### Configuration
Edit `src/vlm_ros/launch/pipeline.launch.py`:
```python
DeclareLaunchArgument('ui_port', default_value='8008'),
DeclareLaunchArgument('auto_approve', default_value='false'),  # Auto-approve VLM (testing only)
```

---

## Development

### Adding New Gestures

1. **Train LSTM model** with new gesture class
2. **Update label_map** in `src/gesture_classifiers_pkg/gesture_classifiers_pkg/lstm_node.py`:
```python
self.label_map = {
    "NEW_GESTURE": "NEW_GESTURE",
    # ...existing mappings...
}
```
3. **Add motion profile** in `src/amr_telemetry_pkg/amr_telemetry_pkg/telemetry_node.py`:
```python
self.motion_profiles = {
    "NEW_GESTURE": (vx, wz, duration),
    # ...existing profiles...
}
```
4. **Rebuild:**
```bash
colcon build --symlink-install --packages-select gesture_classifiers_pkg amr_telemetry_pkg
```

### Tuning Face Detection Filter

Edit `src/gesture_classifiers_pkg/gesture_classifiers_pkg/lstm_node.py`:
```python
# Adjust thresholds in _is_invalid_hand() method
overlap > 0.20    # Face overlap threshold
w_px < 40         # Minimum size threshold  
aspect < 0.2      # Aspect ratio threshold
center_y < 0.25   # Upper region threshold
```

---

## Performance

| Metric | Value | Hardware |
|--------|-------|----------|
| **LSTM Inference** | ~30 FPS | CPU (Intel i7) |
| **MediaPipe Hands** | ~25 FPS | CPU |
| **Face Detection** | < 1ms | CPU |
| **Total Latency** | ~50ms | End-to-end |
| **FastVLM** | ~1s | NVIDIA RTX 5070 |

---

## License

Apache 2.0

---

## Credits

- **iRobot Create3 Simulation:** https://github.com/iRobotEducation/create3_sim
- **MediaPipe:** https://developers.google.com/mediapipe
- **FastVLM:** https://huggingface.co/apple/FastVLM-1.5B
- **Jester Dataset:** https://20bn.com/datasets/jester

---

## Contact

For issues and questions, please open an issue on the GitHub repository.
