#!/usr/bin/env bash
set -e

# 1) GPU venv
source ~/venvs/rosgpu_isolated/bin/activate

# 2) ROS 2
source /opt/ros/humble/setup.bash

# 3) Workspace
cd ~/amr_gesture_ws
source install/setup.bash

# 4) Launch the full AMR pipeline with the ZED
ros2 launch vlm_ros amr_pipeline_zed.launch.py \
  camera_model:=zed2i camera_name:=zed2i \
  resolution:=HD720 framerate:=30 \
  gpu_id:=0 publish_tf:=false depth_mode:=NONE
