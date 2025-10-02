# ~/amr_gesture_ws/launch_all.sh
#!/usr/bin/env bash
set -u  # (no -e, so a single failing window won’t kill the script)

SESSION=amr_stack

# Clean Python/conda noise, then source ROS + your WS
SRC_ROS='unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PROMPT_MODIFIER; \
         unset PYTHONPATH; export PYTHONNOUSERSITE=1; \
         source /opt/ros/humble/setup.bash; \
         source "$HOME/amr_gesture_ws/install/setup.bash"'

# Start clean
tmux kill-session -t "$SESSION" 2>/dev/null || true

# cam: frame simplifier
tmux new-session -d -s "$SESSION" -n cam \
  "bash -lc '$SRC_ROS; \
   export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 NUMEXPR_NUM_THREADS=2; \
   ros2 run frame_simplifier_pkg simplifier_node --ros-args \
     -p in_topic:=/image_raw -p out_topic:=/image_raw_30hz \
     -p width:=320 -p height:=240 -p fps:=30.0; \
   exec bash'"

# live image viewer
tmux new-window -t "$SESSION" -n img \
  "bash -lc '$SRC_ROS; ros2 run image_view image_view --ros-args -r image:=/image_raw_30hz; exec bash'"

# keypoints
tmux new-window -t "$SESSION" -n keypts \
  "bash -lc '$SRC_ROS; \
   export OMP_NUM_THREADS=3 OPENBLAS_NUM_THREADS=3 MKL_NUM_THREADS=3 NUMEXPR_NUM_THREADS=3; \
   ros2 run keypoint_extractor_pkg keypoint_extractor_node --ros-args \
     -p source:=ros -p image_topic:=/image_raw_30hz -p stride:=1 -p debug:=true; \
   exec bash'"

# ONNX LSTM
tmux new-window -t "$SESSION" -n onnx \
  "bash -lc '$SRC_ROS; \
   export OMP_NUM_THREADS=3 OPENBLAS_NUM_THREADS=3 MKL_NUM_THREADS=3 NUMEXPR_NUM_THREADS=3; \
   ros2 run gesture_classifiers_pkg lstm_onnx_node --ros-args \
     -p weights_path:=$HOME/amr_gesture_ws/weights/lstm.onnx \
     -p normalizer_path:=$HOME/amr_gesture_ws/weights/normalizer.json \
     -p labels_path:=$HOME/amr_gesture_ws/weights/labels.txt \
     -p conf_threshold:=0.80 -p min_frames:=30 -p model_frames:=30 -p intra_threads:=3 -p allow_stub:=false; \
   exec bash'"

# VLM service (stub)
tmux new-window -t "$SESSION" -n vlm \
  "bash -lc '$SRC_ROS; ros2 run vlm_ros vlm_node; exec bash'"

# Bridge
tmux new-window -t "$SESSION" -n bridge \
  "bash -lc '$SRC_ROS; \
   export PERCEPTION_TEST_CLIP=$HOME/amr_gesture_ws/data/test_clip.mp4; \
   ros2 run vlm_bridge_pkg vlm_bridge_node --ros-args \
     -p default_clip:=$PERCEPTION_TEST_CLIP -p confirm_timeout_s:=20.0; \
   exec bash'"

# UI kiosk
tmux new-window -t "$SESSION" -n ui \
  "bash -lc '$SRC_ROS; \
   ros2 run ui_kiosk_pkg ui_kiosk_node --ros-args \
     -p host:=127.0.0.1 -p port:=8008 -p auto_approve:=false; \
   exec bash'"

# Central DB
tmux new-window -t "$SESSION" -n db \
  "bash -lc '$SRC_ROS; \
   mkdir -p \"$HOME/amr_gesture_ws/data/training\"; \
   ros2 run central_db_pkg central_db_node --ros-args \
     -p data_root:=$HOME/amr_gesture_ws/data/training \
     -p write_format:=mp4 -p max_keep_per_label:=5000; \
   exec bash'"

tmux select-window -t "$SESSION:onnx"
tmux attach -t "$SESSION"

# allow ~/.local packages (onnxruntime for system Python 3.10)
unset PYTHONNOUSERSITE
export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:${PYTHONPATH:-}"

# keep Conda out of the PATH entirely in tmux windows
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL _CE_CONDA _CE_M
PATH="$(echo "$PATH" | tr ':' '\n' | grep -v -E 'conda|mamba' | paste -sd: -)"



