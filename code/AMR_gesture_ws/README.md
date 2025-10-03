This workspace contains the **AMR Gesture Recognition stack**, integrating multimodal perception (VLM-based), gesture classification, intent messaging, and UI support for training and diagnostics.  
It is organized as a modular ROS 2 Humble workspace with dedicated packages for perception, context handling, repo management, LED feedback, and UI training.

---

# AMR Gesture Control Workspace

### Current Status
- **Build**: Uses `--merge-install` + `--symlink-install`
- **Interfaces**: `amr_interfaces`, `vlm_interfaces` rebuilt cleanly
- **Core Components**:
  - `frame_simplifier_pkg` – downsample camera feed
  - `keypoint_extractor_pkg` – Mediapipe keypoints to ROS topic
  - `gesture_classifiers_pkg` – LSTM ONNX inference
  - `vlm_ros` – Video-LLaVA service (stub or live)
  - `vlm_bridge_pkg` – Connects unknown gestures → VLM → UI Kiosk → Intents
  - `ui_kiosk_pkg` – Web-based manual/auto approval (toggle from UI)
  - `central_db_pkg` – Writes approved clips to `~/amr_gesture_ws/data/training`

### Setup

```bash
# First time only
sudo apt install python3-ament-package python3-colcon-common-extensions ros-humble-image-view

# Build everything
rm -rf build install log
colcon build --merge-install --symlink-install

# Source in every new terminal
source /opt/ros/humble/setup.bash
source ~/amr_gesture_ws/install/setup.bash


~/amr_gesture_ws/launch_all.sh  ----- Not working yet...

# Gesture Recognition Pipeline (ROS2 + LSTM + VLM)

This repository contains the gesture recognition pipeline for human-robot interaction (HRI).  
It integrates ROS 2 nodes for keypoint extraction, frame simplification, LSTM gesture classification, and optional VLM escalation.

---

## Dependencies

Ensure you have the following installed:

- Ubuntu 22.04 (Jammy)
- ROS 2 Humble
- colcon build tools
- Python ≥ 3.10
- PyTorch ≥ 2.0
- ONNX Runtime
- OpenCV, NumPy
- MediaPipe (for keypoints)
- v4l2 (for camera input)

Install base tools:

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions v4l-utils
pip install torch onnxruntime opencv-python mediapipe numpy
```

---

## Workspace Setup

```bash
# Create workspace
mkdir -p ~/amr_gesture_ws/src && cd ~/amr_gesture_ws/src

# Clone this repository (adjust URL if needed)
git clone https://git.faps.uni-erlangen.de/Dokumentation/robotik/human-roboter-interaction-kolamero.git kolamero_repo

# Build workspace
cd ~/amr_gesture_ws
colcon build --merge-install

# Source overlay
source /opt/ros/humble/setup.bash
source ~/amr_gesture_ws/install/setup.bash
```

---

## Running the Pipeline

Open **3 terminals** (source in each):

### 1. Camera Input
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args   -p image_size:="[640,480]" -p fps:=30 -r image_raw:=/image_raw_30hz
```

### 2. Keypoint Extraction + Frame Simplifier
```bash
ros2 run keypoint_extractor_pkg keypoint_extractor_node --ros-args   -r image:=/image_raw_30hz -r /keypoints:=/keypoints_30hz

ros2 run frame_simplifier_pkg simplifier_node --ros-args   -p window:=30 -p stride:=1 -p features:=84   -r /keypoints_in:=/keypoints_30hz   -r /lstm/window:=/lstm/window
```

### 3. LSTM Gesture Classifier
For **5-class model**:
```bash
ros2 run gesture_classifiers_pkg lstm_onnx_node --ros-args   -p weights_path:="$HOME/amr_gesture_ws/models/lstm/current/latest.onnx"   -p labels_path:="$HOME/amr_gesture_ws/models/lstm/current/labels.txt"   -p normalizer_path:="$HOME/amr_gesture_ws/models/lstm/current/normalizer.json"   -p conf_threshold:=0.35 -p min_frames:=30 -p model_frames:=30 -p allow_stub:=false
```

For **27-class model**:
```bash
ros2 run gesture_classifiers_pkg lstm_onnx_node --ros-args   -p weights_path:="$HOME/amr_gesture_ws/models/lstm/current_27/latest.onnx"   -p labels_path:="$HOME/amr_gesture_ws/models/lstm/current_27/labels.txt"   -p normalizer_path:="$HOME/amr_gesture_ws/models/lstm/current_27/normalizer.json"   -p conf_threshold:=0.35 -p min_frames:=30 -p model_frames:=30 -p allow_stub:=false
```

---

## Monitoring

Check outputs:

```bash
ros2 topic list
ros2 topic echo /intents_raw
```

Optional visualization:

```bash
sudo apt install ros-humble-rqt-plot
rqt_plot /intents_raw/confidence
```

---

## Notes

- Camera calibration file is optional; warnings can be ignored if no intrinsic calibration is provided.
- Confidence threshold can be tuned (e.g., 0.35 for testing, 0.8 for production).
- Use the **5-class model** for HRI (wave_stop, follow_me, go_ahead, point, no_gesture).

---
