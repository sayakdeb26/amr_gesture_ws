from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(_ctx, **_kwargs):
    camera_dev   = LaunchConfiguration('camera_dev')
    width        = LaunchConfiguration('width')
    height       = LaunchConfiguration('height')
    fps          = LaunchConfiguration('fps')
    pixel_format = LaunchConfiguration('pixel_format')   # YUYV is safest
    out_enc      = LaunchConfiguration('output_encoding')# rgb8 works for your graph
    ui_port      = LaunchConfiguration('ui_port')
    auto_ok      = LaunchConfiguration('auto_approve')

    nodes = []

    # Webcam (v4l2_camera)
    nodes.append(Node(
        package='v4l2_camera', executable='v4l2_camera_node', name='v4l2_camera',
        parameters=[{
            'video_device': camera_dev,
            'image_size': [width, height],
            'fps': fps,
            'pixel_format': pixel_format,        # force YUYV to avoid MJPG conversion woes
            'output_encoding': out_enc,          # rgb8 for downstream OpenCV
        }],
        remappings=[],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    ))

    # Frame simplifier
    nodes.append(Node(
        package='frame_simplifier_pkg', executable='simplifier_node', name='frame_simplifier',
        parameters=[{
            'input_topic': '/camera/image_raw',
            'output_topic': '/frames/simplified',
        }],
        output='screen'
    ))

    # LSTM gesture classifier
    nodes.append(Node(
        package='gesture_classifiers_pkg', executable='lstm_node', name='lstm_node',
        parameters=[{
            'input_topic': '/frames/simplified',
            'conf_threshold': 0.80,
            'min_frames': 30,
            'normalize': True,
        }],
        output='screen'
    ))

    # Recorder
    nodes.append(Node(
        package='vlm_recorder_pkg', executable='recorder_node', name='recorder',
        parameters=[{
            'input_topic': '/frames/simplified',
            'save_dir':  '~/amr_gesture_ws/data/runtime_clips',
            'window_seconds': 2.0,
        }],
        output='screen'
    ))

    # VLM bridge
    nodes.append(Node(
        package='vlm_bridge_pkg', executable='bridge_node', name='vlm_bridge',
        parameters=[{
            'confirm_timeout_s': 20.0,
            'wait_clip_timeout_s': 5.0,
            'default_clip': '',
        }],
        output='screen'
    ))

    # Central DB
    nodes.append(Node(
        package='central_db_pkg', executable='central_db_node', name='central_db_node',
        output='screen'
    ))

    # UI kiosk
    nodes.append(Node(
        package='ui_kiosk_pkg', executable='ui_kiosk_node', name='ui_kiosk',
        parameters=[{
            'host': '0.0.0.0',
            'port': ui_port,
            'auto_approve': auto_ok,
            'deadline_s': 20.0,
        }],
        output='screen'
    ))

    # Lightweight monitors (no new code): topic statistics + image viewer UI
    # rqt_image_view is handy for visual; comment out if you don't want a GUI window
    try:
        nodes.append(Node(package='rqt_image_view', executable='rqt_image_view', name='viewer'))
    except Exception:
        pass

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_dev', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('pixel_format', default_value='YUYV'),   # <- important
        DeclareLaunchArgument('output_encoding', default_value='rgb8'),
        DeclareLaunchArgument('ui_port', default_value='8008'),
        DeclareLaunchArgument('auto_approve', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
