# AMR Gesture Control - Quick Start Guide

## Launch (Simulation)
```bash
cd ~/amr_gesture_ws
source install/setup.bash
ros2 launch vlm_ros amr_gesture_create3_sim.launch.py
```

## Gesture Commands

### Control Gestures (No Cooldown)
| Gesture | Action |
|---------|--------|
| **THUMB_UP** | Activate gesture control |
| **THUMB_DOWN** | Deactivate gesture control |

### Movement Gestures (2.5s Cooldown)
| Gesture | Motion |
|---------|--------|
| **SWIPE_UP** | Forward 0.20 m/s (1.0s) |
| **SWIPE_DOWN** | Backward -0.20 m/s (1.0s) |
| **SWIPE_LEFT** | Rotate left 0.5 rad/s (0.5s) |
| **SWIPE_RIGHT** | Rotate right -0.5 rad/s (0.5s) |
| **STOP** | Emergency stop |

### Ignored Gestures (No Action)
- `NO_GESTURE` - No hand detected
- `IGNORE` - Gesture filtered out

## Debug Window Controls
- **Q** - Quit
- **R** - Reset buffers

## Features
- ✅ THUMB_UP/DOWN activation/deactivation
- ✅ Face detection filter (prevents false hand detections)
- ✅ cv2.imshow debug window with HUD
- ✅ MediaPipe HandLandmarker visualization
- ✅ FastVLM integration for unknown gestures

## Topics
- `/cmd_vel` - Robot velocity commands
- `/intents_raw` - Gesture intents
- `/lstm/debug_feed` - Debug image stream
- `/lstm/unknown` - Unknown gestures for VLM

## Troubleshooting
```bash
# Kill stuck processes
pkill -f ros2
pkill -f gazebo

# Rebuild after changes
colcon build --symlink-install --packages-select gesture_classifiers_pkg
```
