import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get arguments as strings
    camera_dev = LaunchConfiguration('camera_dev').perform(context)
    width = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    fps = int(LaunchConfiguration('fps').perform(context))
    pixel_format = LaunchConfiguration('pixel_format').perform(context)
    output_encoding = LaunchConfiguration('output_encoding').perform(context)
    ui_port = int(LaunchConfiguration('ui_port').perform(context))
    auto_approve = LaunchConfiguration('auto_approve').perform(context).lower() == 'true'

    # VLM Node - FastVLM (Apple's efficient VLM via HuggingFace)
    vlm_node = Node(
        package='vlm_ros',
        executable='vlm_node',
        name='vlm_node',
        output='screen'
    )
    
    # Camera Node - delayed start
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        parameters=[{
            'video_device': camera_dev,
            'image_size': [width, height],
            'time_per_frame': [1, fps],
            'pixel_format': pixel_format,
            'output_encoding': output_encoding
        }]
    )

    # Simplifier
    simplifier_node = Node(
        package='frame_simplifier_pkg',
        executable='simplifier_node',
        name='simplifier_node',
        parameters=[{
            'input_topic': '/image_raw',
            'output_topic': '/frames/simplified',
            'output_width': 320,
            'output_height': 240,
            'target_fps': 10.0
        }]
    )

    # LSTM
    lstm_node = Node(
        package='gesture_classifiers_pkg',
        executable='lstm_node',
        name='lstm_node',
        parameters=[{
            'min_frames': 30,
            'conf_threshold': 0.80,
            'model_path': '/home/sayak/amr_gesture_ws/models/lstm/jester20b_12cls/final_jester_model.onnx'
        }],
        remappings=[('/camera/image_raw', '/image_raw')]
    )

    # Recorder
    recorder_node = Node(
        package='vlm_recorder_pkg',
        executable='recorder_node',
        name='recorder_node',
        parameters=[{
            'input_topic': '/frames/simplified',
            'save_dir': '/home/sayak/amr_gesture_ws/data/runtime_clips'
        }]
    )

    # Bridge
    bridge_node = Node(
        package='vlm_bridge_pkg',
        executable='bridge_node',
        name='bridge_node',
        parameters=[{
            'confirm_timeout_s': 900.0,
            'wait_clip_timeout_s': 5.0,
            'default_clip': ''
        }]
    )

    # Central DB
    db_node = Node(
        package='central_db_pkg',
        executable='central_db_node',
        name='central_db_node'
    )

    # UI Kiosk
    ui_node = Node(
        package='ui_kiosk_pkg',
        executable='ui_kiosk_node',
        name='ui_kiosk_node',
        parameters=[{
            'host': '0.0.0.0',
            'port': ui_port,
            'auto_approve': auto_approve,
            'deadline_s': 900.0
        }]
    )

    # Telemetry
    telemetry_node = Node(
        package='amr_telemetry_pkg',
        executable='telemetry_node',
        name='telemetry_node'
    )
    
    # Start VLM first, then wait 15 seconds before starting other nodes
    # No "SYSTEM READY" message - VLM will announce when ready
    return [
        vlm_node,
        TimerAction(
            period=15.0,
            actions=[
                camera_node,
                simplifier_node,
                lstm_node,
                recorder_node,
                bridge_node,
                db_node,
                ui_node,
                telemetry_node
            ]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='\n\n=== INITIALIZING GESTURE CONTROL PIPELINE ===\nLoading VLM model (takes ~15 seconds)...\nWait for "Video-LLaVA Service Ready" message\n'),
        DeclareLaunchArgument('camera_dev', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('output_encoding', default_value='rgb8'),
        DeclareLaunchArgument('ui_port', default_value='8008'),
        DeclareLaunchArgument('auto_approve', default_value='false'),
        OpaqueFunction(function=launch_setup)
    ])
