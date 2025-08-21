“This workspace contains the AMR Gesture Recognition stack: perception (VLM), context bus, gesture classifiers, intent interfaces, repo manager, and trainer UI.”

# amr_gesture_ws
VLM_gesture _control

amr_gesture_ws/
├─ src/ # ROS 2 packages
│ ├─ amr_interfaces/ # msgs/srvs (Keypoints, Gesture, Intent, Diagnostics; UpsertGesture, SaveMapping, SwitchProfile, ReloadRepo)
│ ├─ repo_manager_pkg/ # JSON repo + profiles manager (services + Diagnostics)
│ ├─ ... # (arbiter_pkg, perception_pkg, etc. stubs)
├─ cloud/
│ └─ video_llava_service/ # FastAPI sidecar (local VLM) → /infer
├─ configs/ # topics.yaml, policy.yaml (includes repo path + profile)
├─ data/
│ └─ repo/profiles/base/ # gestures.json, mappings.json (persisted)
├─ models/ # (place model weights here; Git LFS-recommended)
├─ Makefile # ros-build, vlm-run, lint, fmt
└─ README.md
