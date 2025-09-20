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


~/amr_gesture_ws/launch_all.sh
