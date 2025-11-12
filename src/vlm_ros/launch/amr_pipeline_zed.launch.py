from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    camera_model = LaunchConfiguration('camera_model', default='zed2i')
    camera_name  = LaunchConfiguration('camera_name',  default='zed2i')
    resolution   = LaunchConfiguration('resolution',   default='HD720')
    framerate    = LaunchConfiguration('framerate',    default='30')
    gpu_id       = LaunchConfiguration('gpu_id',       default='0')
    publish_tf   = LaunchConfiguration('publish_tf',   default='false')
    depth_mode   = LaunchConfiguration('depth_mode',   default='NONE')

    # ZED wrapper (expects plain arg names: camera_model, resolution, frame_rate, publish_tf, depth_mode, gpu_id)
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('zed_wrapper') + '/launch/zed_camera.launch.py'
        ),
        launch_arguments={
            'camera_model': camera_model,
            'camera_name':  camera_name,
            'resolution':   resolution,
            'frame_rate':   framerate,
            'gpu_id':       gpu_id,
            'publish_tf':   publish_tf,
            'depth_mode':   depth_mode,
        }.items()
    )

    # Hardcode topic for now (we pass camera_name:=zed2i)
    zed_rgb_topic = '/zed2i/zed_node/rgb/image_rect_color'

    frame_simplifier = Node(
        package='frame_simplifier_pkg',
        executable='frame_simplifier',
        name='frame_simplifier',
        output='screen',
        parameters=[{'target_fps': 10.0}],
        remappings=[('/image_in', zed_rgb_topic),
                    ('/image_out','/image_raw_10hz')],
    )

    keypoint_extractor = Node(
        package='keypoint_extractor_pkg',
        executable='keypoint_extractor_node',
        name='keypoint_extractor_node',
        output='screen',
        parameters=[{'window_size': 30, 'drop_z': True}],
        remappings=[('/image_raw','/image_raw_10hz'),
                    ('/keypoints_window','/lstm/keypoints_window')],
    )

    lstm_onnx = Node(
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
        remappings=[('/keypoints_window','/lstm/keypoints_window'),
                    ('/intents_raw','/intents_raw'),
                    ('/unknown','/lstm/unknown')],
    )

    recorder = Node(
        package='vlm_recorder_pkg',
        executable='recorder_node',
        name='recorder_node',
        output='screen',
        parameters=[{
            'image_topic': '/image_raw_10hz',
            'out_dir':     '/home/sayak/amr_kiosk_media/tmp',
            'buffer_seconds': 6.0,
            'target_fps': 10.0,
        }],
    )

    vlm_bridge = Node(
        package='vlm_bridge_pkg',
        executable='vlm_bridge_node',
        name='vlm_bridge_node',
        output='screen',
        parameters=[{'confirm_timeout_s': 20,
                     'kiosk_url': 'http://127.0.0.1:8008'}],
        remappings=[('/lstm/unknown','/lstm/unknown'),
                    ('/vlm/confirm_request','/vlm/confirm_request'),
                    ('/ui/confirm_reply','/ui/confirm_reply'),
                    ('/intents','/intents')],
    )

    ui_kiosk = Node(
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
            'decision_timeout_s': 20,
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

    image_view = Node(
        package='image_view',
        executable='image_view',
        name='zed_rgb_view',
        output='screen',
        remappings=[('image', zed_rgb_topic)],
        emulate_tty=True
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_model',  default_value='zed2i'),
        DeclareLaunchArgument('camera_name',   default_value='zed2i'),
        DeclareLaunchArgument('resolution',    default_value='HD720'),
        DeclareLaunchArgument('framerate',     default_value='30'),
        DeclareLaunchArgument('gpu_id',        default_value='0'),
        DeclareLaunchArgument('publish_tf',    default_value='false'),
        DeclareLaunchArgument('depth_mode',    default_value='NONE'),
        zed_launch,
        frame_simplifier,
        keypoint_extractor,
        lstm_onnx,
        recorder,
        vlm_bridge,
        ui_kiosk,
        central_db,
        image_view,
    ])
