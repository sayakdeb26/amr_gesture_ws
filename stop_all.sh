#!/usr/bin/env bash
set -Eeuo pipefail
SESSION="amr_stack"
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
  echo "Killed tmux session: $SESSION"
fi
# Best-effort kill any leftover nodes
pkill -f "ros2 run v4l2_camera v4l2_camera_node"      || true
pkill -f "frame_simplifier_pkg simplifier_node"       || true
pkill -f "keypoint_extractor_pkg keypoint_extractor"  || true
pkill -f "gesture_classifiers_pkg lstm_onnx_node"     || true
pkill -f "vlm_ros vlm_node"                           || true
pkill -f "vlm_bridge_pkg vlm_bridge_node"             || true
pkill -f "ui_kiosk_pkg ui_kiosk_node"                 || true
pkill -f "central_db_pkg central_db_node"             || true
echo "Done."
