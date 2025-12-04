#!/bin/bash
# Setup script for VLM pipeline with Create3 robot

# Set domain ID for Create3 robot at 192.168.137.61
export ROS_DOMAIN_ID=95

# Source workspace
source /home/sayak/amr_gesture_ws/install/setup.bash

echo "✅ Environment configured for Create3 (Domain ID: $ROS_DOMAIN_ID)"
echo "Ready to run: ros2 launch vlm_ros pipeline.launch.py"
