from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    frame_simplifier = Node(
        package='frame_simplifier_pkg',
        executable='simplifier_node',
        name='frame_simplifier',
        output='screen',
        parameters=[{'target_fps': 30}],
        remappings=[('image_in','/image_raw'),
                    ('image_out','/image_raw_30hz')],
    )

    keypoints = Node(
        package='keypoint_extractor_pkg',
        executable='keypoint_extractor_node',
        name='keypoint_extractor_node',
        output='screen',
        parameters=[{'window_size': 30, 'drop_z': True}],
        remappings=[('image_raw','/image_raw_30hz'),
                    ('keypoints_window','/lstm/keypoints_window')],
    )

    lstm = Node(
        package='gesture_classifiers_pkg',
        executable='lstm_onnx_node',
        name='lstm_onnx_node',
        output='screen',
        parameters=[{
            'n_frames': 30,
            'use_cuda': True,
            'cuda_device': 0,
            'conf_threshold': 0.80,
            'background_label': 'NO_GESTURE',
            'treat_background_as_unknown': True,
        }],
        remappings=[('keypoints_window','/lstm/keypoints_window'),
                    ('intents_raw','/intents_raw'),
                    ('unknown','/lstm/unknown')],
    )

    recorder = Node(
        package='vlm_recorder_pkg',
        executable='recorder_node',
        name='recorder_node',
        output='screen',
        parameters=[{
            'image_topic': '/image_raw_30hz',
            'out_dir': '/home/sayak/amr_kiosk_media/tmp',
            'buffer_seconds': 6.0,
            'target_fps': 30,
        }],
    )

    bridge = Node(
        package='vlm_bridge_pkg',
        executable='bridge_node',
        name='vlm_bridge_node',
        output='screen',
        parameters=[{'confirm_timeout_s': 20.0,
                     'kiosk_url': 'http://127.0.0.1:8008'}],
        remappings=[('lstm/unknown','/lstm/unknown'),
                    ('vlm/confirm_request','/vlm/confirm_request'),
                    ('ui/confirm_reply','/ui/confirm_reply'),
                    ('intents','/intents')],
    )

    kiosk = Node(
        package='ui_kiosk_pkg',
        executable='ui_kiosk_node',
        name='ui_kiosk_node',
        output='screen',
        parameters=[{
            'http_host': '0.0.0.0',
            'http_port': 8008,
            'media_dir': '/home/sayak/amr_kiosk_media',
            'history_size': 5,
            'video_width': 960,
            'video_height': 540,
            'decision_timeout_s': 20.0,
            'auto_approve_default': False,
            'janitor_enabled': True,
            'janitor_interval_s': 3600,
            'keep_days_approved': 1,
            'dev_mode': True,
        }],
    )

    central_db = Node(
        package='central_db_pkg',
        executable='central_db_node',
        name='central_db_node',
        output='screen',
    )

    return LaunchDescription([
        frame_simplifier,
        keypoints,
        lstm,
        recorder,
        bridge,
        kiosk,
        central_db,
    ])
