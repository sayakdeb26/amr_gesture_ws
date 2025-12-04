#!/bin/bash
# Wrapper to run video_llava_node with rosgpu_isolated venv

# Activate the venv
source /home/sayak/venvs/rosgpu_isolated/bin/activate

# Source ROS 2
source /opt/ros/humble/setup.bash
source /home/sayak/amr_gesture_ws/install/setup.bash

# Run the actual node
exec python3 /home/sayak/amr_gesture_ws/install/vlm_videollava_pkg/lib/vlm_videollava_pkg/video_llava_node "$@"
