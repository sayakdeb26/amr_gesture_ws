#!/usr/bin/env bash
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-}
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
export COLCON_TRACE=${COLCON_TRACE:-}

set -euo pipefail

# ---------- env ----------
conda deactivate >/dev/null 2>&1 || true
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
source /opt/ros/humble/setup.bash
[ -f "$HOME/amr_gesture_ws/install/setup.bash" ] && source "$HOME/amr_gesture_ws/install/setup.bash"

SESSION="monitor"

# kill any old session to avoid layout conflicts
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

tmux new-session -d -s "$SESSION" -n topics

make_topics () {
  local n="$1"
  tmux kill-window -t "$SESSION":0 2>/dev/null || true
  tmux new-window -t "$SESSION":0 -n topics

  # Start by creating 2 panes, then branch
  tmux split-window -h -t "$SESSION":0 || return 1

  case "$n" in
    8)
      tmux split-window -v -t "$SESSION":0.0 || return 1
      tmux split-window -v -t "$SESSION":0.1 || return 1
      tmux split-window -h -t "$SESSION":0.2 || return 1
      tmux split-window -h -t "$SESSION":0.3 || return 1
      tmux split-window -v -t "$SESSION":0.4 || return 1
      tmux split-window -v -t "$SESSION":0.5 || return 1
      tmux select-layout -t "$SESSION":0 tiled || return 1
      ;;

    6)
      tmux split-window -v -t "$SESSION":0.0 || return 1
      tmux split-window -v -t "$SESSION":0.1 || return 1
      tmux split-window -h -t "$SESSION":0.2 || return 1
      tmux split-window -h -t "$SESSION":0.3 || return 1
      tmux select-layout -t "$SESSION":0 tiled || return 1
      ;;

    4|*)
      tmux split-window -v -t "$SESSION":0.0 || return 1
      tmux split-window -v -t "$SESSION":0.1 || return 1
      tmux select-layout -t "$SESSION":0 tiled || return 1
      ;;
  esac

  # Fill panes with commands (use as many as fit for n)
  # Pane numbering depends on layout order after 'tiled'

  # common first four
  tmux send-keys -t "$SESSION":0.0 'ros2 topic echo /intents_raw' C-m
  tmux send-keys -t "$SESSION":0.1 'ros2 topic echo /vlm/confirm_request' C-m
  tmux send-keys -t "$SESSION":0.2 'ros2 topic echo /ui/confirm_reply' C-m
  tmux send-keys -t "$SESSION":0.3 'ros2 topic echo /db/training_example' C-m

  if [ "$n" -ge 6 ]; then
    tmux send-keys -t "$SESSION":0.4 'ros2 topic hz /image_raw_30hz' C-m
    tmux send-keys -t "$SESSION":0.5 'ros2 topic hz /lstm/keypoints_window' C-m
  fi

  if [ "$n" -ge 8 ]; then
    tmux send-keys -t "$SESSION":0.6 'ros2 topic echo /lstm/keypoints_window | egrep "^(frames:|joints_per_frame:|source:)"' C-m
    tmux send-keys -t "$SESSION":0.7 'watch -n 2 '"'"'echo "=== Nodes ==="; ros2 node list; echo; echo "=== Topics (first 50) ==="; ros2 topic list -t | sed -n "1,50p"'"'" C-m
  fi
}

# Try 8 → 6 → 4 panes
if ! make_topics 8 2>/dev/null; then
  if ! make_topics 6 2>/dev/null; then
    make_topics 4
    tmux display-message -t "$SESSION" "monitor: fell back to 4-pane layout"
  else
    tmux display-message -t "$SESSION" "monitor: using 6-pane layout"
  fi
else
  tmux display-message -t "$SESSION" "monitor: using 8-pane layout"
fi

# ---------- Window 2: camera viewer ----------
tmux new-window -t "$SESSION":1 -n camera
tmux send-keys -t "$SESSION":1 '
if command -v ros2 >/dev/null && ros2 pkg executables image_view >/dev/null 2>&1; then
  ros2 run image_view image_view --ros-args -p image:=/image_raw_30hz
else
  echo "image_view not installed. Try: sudo apt install ros-humble-image-view"
  echo "Fallback: rqt_image_view (sudo apt install ros-humble-rqt-image-view) then run: rqt_image_view"
  bash
fi' C-m

# ---------- Window 3: system ----------
tmux new-window -t "$SESSION":2 -n sys
tmux split-window -h -t "$SESSION":2
tmux send-keys -t "$SESSION":2.0 'htop' C-m
tmux send-keys -t "$SESSION":2.1 'watch -n 1 '\''echo "TIME: $(date)"; echo; free -h; echo; df -h --output=source,size,used,avail,target | sed -n "1,12p"'\''' C-m

tmux select-window -t "$SESSION":0
tmux attach-session -t "$SESSION"

