#!/usr/bin/env bash

# Activate GPU venv
source /home/sayak/venvs/rosgpu_isolated/bin/activate

# ROS + workspace
source /opt/ros/humble/setup.bash
cd /home/sayak/amr_gesture_ws
source install/setup.bash

# Launch the full webcam pipeline (GPU FastVLM + everything else)
ros2 launch vlm_ros amr_pipeline_webcam_30hz.launch.py
