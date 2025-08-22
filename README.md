This workspace contains the **AMR Gesture Recognition stack**, integrating multimodal perception (VLM-based), gesture classification, intent messaging, and UI support for training and diagnostics.  
It is organized as a modular ROS 2 Humble workspace with dedicated packages for perception, context handling, repo management, LED feedback, and UI training.

---

amr_gesture_ws/
├── src/                  # ROS 2 packages
│   ├── amr_interfaces        # Custom msg/srv definitions (e.g., Intent.msg)
│   ├── arbiter_pkg           # Arbitration logic for multi-intent handling
│   ├── context_bus_pkg       # Publishes context/state across nodes
│   ├── gesture_classifiers_pkg # Lightweight classifiers for gesture mapping
│   ├── led_manager_pkg       # LED feedback + control logic
│   ├── perception_pkg        # VLM/Vision pipeline for gesture detection
│   ├── repo_manager_pkg      # Profiles repository manager (JSON/SQL-backed)
│   └── trainer_ui_pkg        # Training/diagnostics user interface
├── configs/              # YAML configs for models, bus, and profiles
├── cloud/                # (Optional) Cloud integration hooks
├── data/                 # Sample datasets / logs
├── models/               # Pretrained model weights (gitignored)
├── tools/                # Utility scripts
├── build/ install/ log/  # Colcon build outputs
├── README.md             # This file
└── LICENSE
