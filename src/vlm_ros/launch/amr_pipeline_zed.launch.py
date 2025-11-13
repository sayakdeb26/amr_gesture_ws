import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # --------- ZED launch args ----------
    camera_model = LaunchConfiguration("camera_model", default="zed2i")
    camera_name  = LaunchConfiguration("camera_name",  default="zed2i")
    resolution   = LaunchConfiguration("resolution",   default="HD720")
    framerate    = LaunchConfiguration("framerate",    default="30")
    gpu_id       = LaunchConfiguration("gpu_id",       default="0")
    publish_tf   = LaunchConfiguration("publish_tf",   default="false")
    depth_mode   = LaunchConfiguration("depth_mode",   default="NEURAL")

    # ZED wrapper launch file (from zed_wrapper package)
    zed_wrapper_share = get_package_share_directory("zed_wrapper")
    zed_camera_launch = os.path.join(zed_wrapper_share, "launch", "zed_camera.launch.py")

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_camera_launch),
        launch_arguments={
            "camera_model": camera_model,
            "camera_name":  camera_name,
            "resolution":   resolution,
            "frame_rate":   framerate,
            "gpu_id":       gpu_id,
            "publish_tf":   publish_tf,
            "depth_mode":   depth_mode,
        }.items(),
    )

    # RGB topic from ZED (we keep it hardcoded to zed2i namespace)
    zed_rgb_topic = "/zed2i/zed_node/rgb/image_rect_color"

    # --------- Frame simplifier (10 Hz) ----------
    frame_simplifier = Node(
        package="frame_simplifier_pkg",
        executable="frame_simplifier",
        name="frame_simplifier",
        output="screen",
        parameters=[{"target_fps": 10.0}],
        remappings=[
            ("/image_in", zed_rgb_topic),
            ("/image_out", "/image_raw_10hz"),
        ],
    )

    # --------- MediaPipe keypoint extractor ----------
    keypoint_extractor = Node(
        package="keypoint_extractor_pkg",
        executable="keypoint_extractor_node",
        name="keypoint_extractor_node",
        output="screen",
        parameters=[{"window_size": 30, "drop_z": True}],
        remappings=[
            ("/image_raw", "/image_raw_10hz"),
            ("/keypoints_window", "/lstm/keypoints_window"),
        ],
    )

    # --------- LSTM ONNX gesture classifier (GPU) ----------
    lstm_onnx = Node(
        package="gesture_classifiers_pkg",
        executable="lstm_onnx_node",
        name="lstm_onnx_node",
        output="screen",
        parameters=[
            {
                "weights_path":    "/home/sayak/amr_gesture_ws/models/lstm/jester20b_12cls/model_30x84.onnx",
                "normalizer_path": "/home/sayak/amr_gesture_ws/models/lstm/jester20b_12cls/normalizer.json",
                "labels_path":     "/home/sayak/amr_gesture_ws/models/lstm/jester20b_12cls/labels.txt",
                "publish_bg":      False,
                "min_confidence":  0.6,
            }
        ],
        remappings=[
            ("/keypoints_window", "/lstm/keypoints_window"),
            ("/intents_raw", "/intents_raw"),
            ("/unknown", "/lstm/unknown"),
        ],
    )

    # --------- Recorder (circular buffer, 6 s @ 10 Hz) ----------
    recorder = Node(
        package="vlm_recorder_pkg",
        executable="recorder_node",
        name="recorder_node",
        output="screen",
        parameters=[
            {
                "image_topic":    "/image_raw_10hz",
                "out_dir":        "/home/sayak/amr_gesture_ws/data/training/samples",
                "buffer_seconds": 6.0,
                "target_fps":     10.0,
            }
        ],
    )

    # --------- VLM node (FastVLM on CPU) ----------
    vlm_node = Node(
        package="vlm_ros",
        executable="vlm_node",
        name="vlm_node",
        output="screen",
        # Force CPU for VLM
        env={
            "VLM_MODEL_ID":   "apple/FastVLM-1.5B",
            "VLM_DEVICE":     "cpu",
            "VLM_FRAMES":     "5",
            "VLM_MAX_TOKENS": "24",
        },
    )

    # --------- VLM bridge (session-locked pipeline brain) ----------
    vlm_bridge = Node(
        package="vlm_bridge_pkg",
        executable="bridge_node",
        name="pipeline",
        output="screen",
        parameters=[
            {
                "confirm_timeout_s": 20.0,
                "resume_on_timeout": True,
            }
        ],
    )

    # --------- UI Kiosk ----------
    ui_kiosk = Node(
        package="ui_kiosk_pkg",
        executable="ui_kiosk_node",
        name="ui_kiosk_node",
        output="screen",
        parameters=[
            {
                "media_dir":           "/home/sayak/amr_kiosk_media",
                "decision_timeout_s":  20,
                "auto_approve_default": False,
                "janitor_enabled":      True,
                "keep_days_approved":   1,
                "dev_mode":             True,
            }
        ],
    )

    # --------- Central DB ----------
    central_db = Node(
        package="central_db_pkg",
        executable="central_db_node",
        name="central_db_node",
        output="screen",
        parameters=[
            {
                "db_path": "/home/sayak/amr_db",
            }
        ],
    )

    # --------- Image view (for ZED RGB stream) ----------
    image_view = Node(
        package="image_view",
        executable="image_view",
        name="zed_rgb_view",
        output="screen",
        remappings=[("/image", zed_rgb_topic)],
    )

    # --------- LaunchDescription ----------
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_model", default_value="zed2i"),
            DeclareLaunchArgument("camera_name",  default_value="zed2i"),
            DeclareLaunchArgument("resolution",   default_value="HD720"),
            DeclareLaunchArgument("framerate",    default_value="30"),
            DeclareLaunchArgument("gpu_id",       default_value="0"),
            DeclareLaunchArgument("publish_tf",   default_value="false"),
            DeclareLaunchArgument("depth_mode",   default_value="NEURAL"),
            zed_launch,
            frame_simplifier,
            keypoint_extractor,
            lstm_onnx,
            recorder,
            vlm_node,
            vlm_bridge,
            ui_kiosk,
            central_db,
            image_view,
        ]
    )
