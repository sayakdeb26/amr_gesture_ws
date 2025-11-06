from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(ctx, **_kwargs):
    # LaunchConfiguration Werte auswerten und in korrekte Typen konvertieren
    camera_dev   = LaunchConfiguration('camera_dev').perform(ctx)
    width        = int(LaunchConfiguration('width').perform(ctx))
    height       = int(LaunchConfiguration('height').perform(ctx))
    fps          = int(LaunchConfiguration('fps').perform(ctx))
    pixel_format = LaunchConfiguration('pixel_format').perform(ctx)
    out_enc      = LaunchConfiguration('output_encoding').perform(ctx)
    ui_port      = int(LaunchConfiguration('ui_port').perform(ctx))

    # auto_approve String zu Boolean konvertieren
    auto_ok_str = LaunchConfiguration('auto_approve').perform(ctx)
    auto_ok = True if auto_ok_str.lower() == 'true' else False

    nodes = []

    # Webcam (v4l2_camera)
    nodes.append(Node(
        package='v4l2_camera', executable='v4l2_camera_node', name='v4l2_camera',
        parameters=[{
            'video_device': camera_dev,
            'image_size': [width, height],  # echte Integer-Liste
            'fps': fps,
            'pixel_format': pixel_format,        
            'output_encoding': out_enc,        
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

    # Keypoint Identfikation
    nodes.append(Node(
        package='keypoint_extractor_pkg', executable='keypoint_extractor_node', name='keypoint_extractor_node',
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

    #VLM service node
    nodes.append(Node(
        package='vlm_ros', executable='vlm_node', name='vlm_node',  
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
            'auto_approve': auto_ok,  # Boolean korrekt übergeben
            'deadline_s': 20.0,
        }],
        output='screen'
    ))

    # Optional: rqt_image_view für Visualisierung
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
        DeclareLaunchArgument('pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('output_encoding', default_value='rgb8'),
        DeclareLaunchArgument('ui_port', default_value='8008'),
        DeclareLaunchArgument('auto_approve', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
