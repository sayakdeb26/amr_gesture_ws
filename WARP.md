# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Architecture Overview

This is a **ROS 2 Humble workspace** for AMR (Autonomous Mobile Robot) **gesture recognition** using multimodal perception with VLM (Vision-Language Model) integration. The system processes camera input through a pipeline of gesture classification, intent messaging, and UI support for training.

### Core Pipeline Flow
1. **Camera Input** → **Frame Simplifier** → **Keypoint Extraction** (MediaPipe)
2. **Feature Bridge** → **LSTM ONNX Classifier** → **Intent Recognition**
3. **Unknown Gestures** → **VLM Bridge** → **UI Kiosk** → **Manual Approval**
4. **Approved Examples** → **Central DB** → **Training Data**

### Key ROS 2 Packages
- **`gesture_classifiers_pkg`**: LSTM ONNX inference for gesture classification
- **`keypoint_extractor_pkg`**: MediaPipe keypoints extraction 
- **`frame_simplifier_pkg`**: Camera feed downsampling and rate control
- **`vlm_bridge_pkg`**: Connects unknown gestures to VLM service
- **`ui_kiosk_pkg`**: Web-based manual approval interface
- **`central_db_pkg`**: Training data storage and management
- **`amr_interfaces`**: Custom message definitions (Intent, Gesture, etc.)

### Critical Topics
- `/image_raw_30hz`: Simplified camera frames
- `/keypoints_30hz`: MediaPipe keypoints
- `/features_84`: 84-dimensional features for LSTM
- `/intents_raw`: Raw gesture classifications
- `/intents_approved`: Human-approved intents
- `/lstm/unknown`: Unknown/low-confidence gestures

## Common Development Commands

### Build System
```bash
# Clean build (recommended for interface changes)
rm -rf build install log
colcon build --merge-install --symlink-install

# Quick build
make ros-build

# Source environment (required in every new terminal)
source /opt/ros/humble/setup.bash
source ~/amr_gesture_ws/install/setup.bash
```

### Launch Complete System
```bash
# Full pipeline via tmux (multi-window)
./launch_all.sh

# Or via launch file
ros2 launch central_db_pkg amr_pipeline.launch.py
```

### Individual Components
```bash
# Camera input only
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" -p fps:=30

# LSTM classifier (5-class model)
ros2 run gesture_classifiers_pkg lstm_onnx_node --ros-args \
  -p weights_path:="$HOME/amr_gesture_ws/weights/lstm.onnx" \
  -p labels_path:="$HOME/amr_gesture_ws/weights/labels.txt" \
  -p conf_threshold:=0.35 -p min_frames:=30

# UI Kiosk for manual approval
ros2 run ui_kiosk_pkg ui_kiosk_node --ros-args -p port:=8008
```

### Development Tools
```bash
# Code formatting
make fmt  # black formatter
make lint # pre-commit hooks (black + flake8)

# Topic monitoring
ros2 topic list
ros2 topic echo /intents_raw
ros2 topic hz /keypoints_30hz

# VLM service (development)
make vlm-run  # uvicorn server on localhost:8000
```

### Testing
```bash
# Monitor pipeline performance
python3 tools/pipeline_monitor.py

# Process training data
python3 tools/train_lstm_from_npz.py
python3 tools/preprocess_jester.py
```

## Development Guidelines

### Model Configuration
- **5-class model**: Standard HRI gestures (wave_stop, follow_me, go_ahead, point, no_gesture)
- **27-class model**: Extended Jester dataset support
- Models expect **30 frames** at **84-dim features** (42 joints × 2 hands, x/y coordinates)
- Confidence threshold: 0.35 (testing) → 0.80+ (production)

### System Modes
- **operation**: Normal runtime mode
- **teaching**: Training data collection mode
- Toggle via `config/system_mode.yaml`

### Environment Variables
- `AMR_DB_DIR`: Override database location (default: `~/amr_db`)
- `OMP_NUM_THREADS=3`: Limit threading for ONNX inference
- `PYTHONNOUSERSITE=1`: Isolate from conda environments

### Data Management
- Training clips: `~/amr_gesture_ws/data/training/`
- Model weights: `~/amr_gesture_ws/weights/`
- Configuration: `config/` and `configs/` directories
- Large files (*.onnx, *.pt, data/) are gitignored

### Interface Dependencies
When modifying custom messages in `amr_interfaces`, **always clean rebuild**:
```bash
rm -rf build install log
colcon build --merge-install --symlink-install
```

### Performance Tuning
- Frame rates: Camera 30Hz → Simplified 10-30Hz → LSTM inference
- Thread limits: Set `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS` to 2-3
- QoS: Use BEST_EFFORT/KEEP_LAST for image topics
- Buffer management: 5-second video clips for VLM analysis

## Architecture Notes

This system implements a **hybrid symbolic-neural approach**:
- **Fast path**: ONNX LSTM for known gestures (< 120ms latency)
- **Slow path**: VLM analysis for unknown gestures (< 400ms timeout)
- **Human-in-loop**: Manual approval via web UI for safety-critical decisions
- **Continuous learning**: Approved examples feed back into training pipeline

The modular ROS 2 design allows independent scaling of perception, classification, and decision components.