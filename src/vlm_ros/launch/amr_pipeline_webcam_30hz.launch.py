#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    #
    # 0) Webcam (v4l2_camera)  →  /image_raw  @ ~30 Hz
    #
    v4l2_camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        output="screen",
        parameters=[{
            # these are fine as defaults; adjust if you like
            "image_size": [640, 480],
            "time_per_frame": [1, 30],  # 30 fps
        }],
    )

    #
    # 1) Frame simplifier: /image_raw → /image_raw_30hz  (no KPX bridge)
    #
    frame_simplifier = Node(
        package="frame_simplifier_pkg",
        executable="simplifier_node",
        name="frame_simplifier",
        output="screen",
        parameters=[{
            "target_fps": 30.0,
        }],
        remappings=[
            ("/image_in", "/image_raw"),
            ("/image_out", "/image_raw_30hz"),
        ],
    )

    #
    # 2) Keypoint extractor: /image_raw_30hz → /lstm/keypoints_window
    #
    keypoint_extractor = Node(
        package="keypoint_extractor_pkg",
        executable="keypoint_extractor_node",
        name="keypoint_extractor_node",
        output="screen",
        parameters=[{
            "window_size": 30,
            "drop_z": True,
        }],
        remappings=[
            ("/image_raw_10hz", "/image_raw_30hz"),
            ("/keypoints_window", "/lstm/keypoints_window"),
        ],
    )

    #
    # 3) LSTM ONNX: /lstm/keypoints_window → /intents_raw + /lstm/unknown
    #
    lstm_onnx = Node(
        package="gesture_classifiers_pkg",
        executable="lstm_onnx_node",
        name="lstm_onnx_node",
        output="screen",
        parameters=[{
            "weights_path":
                "/home/sayak/amr_gesture_ws/models/lstm/jester20b_12cls/model_30x84.onnx",
            "normalizer_path":
                "/home/sayak/amr_gesture_ws/models/lstm/jester20b_12cls/normalizer.json",
            "labels_path":
                "/home/sayak/amr_gesture_ws/models/lstm/jester20b_12cls/labels.txt",
            "n_frames": 30,
            "use_cuda": True,
            "cuda_device": 0,
            "conf_threshold": 0.8,
            "background_label": "NO_GESTURE",
            "treat_background_as_unknown": True,
        }],
        remappings=[
            ("/keypoints_window", "/lstm/keypoints_window"),
            ("/intents_raw", "/intents_raw"),
            ("/unknown", "/lstm/unknown"),
        ],
    )

    #
    # 4) Recorder: /image_raw_30hz → files under amr_kiosk_media/tmp
    #
    recorder = Node(
        package="vlm_recorder_pkg",
        executable="recorder_node",
        name="recorder_node",
        output="screen",
        parameters=[{
            "image_topic": "/image_raw_30hz",
            "out_dir": "/home/sayak/amr_kiosk_media/tmp",
            "buffer_seconds": 6.0,
            "target_fps": 30.0,
        }],
    )

    #
    # 5) Real VLM node (FastVLM on GPU) – service /vlm/infer
    #
    vlm_node = Node(
        package="vlm_ros",
        executable="vlm_node",
        name="vlm_node",
        output="screen",
        # venv + ROS env will be handled by the wrapper script below
    )

    #
    # 6) Bridge: /lstm/unknown + recorder → /vlm/infer → kiosk + /intents
    #
    vlm_bridge = Node(
        package="vlm_bridge_pkg",
        executable="bridge_node",
        name="bridge_node",
        output="screen",
        parameters=[{
            "confirm_timeout_s": 20.0,
            "kiosk_url": "http://127.0.0.1:8008",
        }],
        remappings=[
            ("/lstm/unknown", "/lstm/unknown"),
            ("/vlm/confirm_request", "/vlm/confirm_request"),
            ("/ui/confirm_reply", "/ui/confirm_reply"),
            ("/intents", "/intents"),
        ],
    )

    #
    # 7) UI kiosk (serves clips from /home/sayak/amr_kiosk_media)
    #
    ui_kiosk = Node(
        package="ui_kiosk_pkg",
        executable="ui_kiosk_node",
        name="ui_kiosk_node",
        output="screen",
        parameters=[
            "/home/sayak/amr_gesture_ws/src/ui_kiosk_pkg/config/kiosk_params.yaml",
        ],
    )

    #
    # 8) Central DB: subscribes /intents and logs to /home/sayak/amr_db
    #
    central_db = Node(
        package="central_db_pkg",
        executable="central_db_node",
        name="central_db_node",
        output="screen",
    )

    return LaunchDescription([
        v4l2_camera,
        frame_simplifier,
        keypoint_extractor,
        lstm_onnx,
        recorder,
        vlm_node,
        vlm_bridge,
        ui_kiosk,
        central_db,
    ])

