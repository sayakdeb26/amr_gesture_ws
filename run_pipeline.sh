#!/bin/bash

# Function to handle cleanup
cleanup() {
    echo ""
    echo "🛑 Caught SIGINT/SIGTERM. Shutting down..."
    
    # Kill the launch process group
    # (This script's children)
    pkill -P $$

    echo "🧹 Force killing ROS nodes to ensure clean exit..."
    # List of node executables to kill
    NODES=(
        "ui_kiosk_node"
        "bridge_node"
        "recorder_node"
        "simplifier_node"
        "lstm_node"
        "video_llava_node"
        "v4l2_camera_node"
        "central_db_node"
        "telemetry_node"
    )

    for node in "${NODES[@]}"; do
        pkill -f "$node" > /dev/null 2>&1
    done

    # Also kill by port just in case
    fuser -k 8008/tcp > /dev/null 2>&1

    echo "✅ Shutdown complete."
    exit 0
}

# Trap signals
trap cleanup SIGINT SIGTERM

# Configure for Create3 robot
export ROS_DOMAIN_ID=95

# Source environment
source install/setup.bash

# Run the launch file
echo "🚀 Launching VLM Pipeline..."
ros2 launch vlm_ros pipeline.launch.py &

# Wait for the background process
PID=$!
wait $PID
