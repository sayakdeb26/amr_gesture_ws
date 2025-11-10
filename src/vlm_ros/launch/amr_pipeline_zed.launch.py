import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    camera_model = LaunchConfiguration('camera_model', default='zed2i')
    camera_name  = LaunchConfiguration('camera_name',  default='zed2i')
    resolution   = LaunchConfiguration('resolution',   default='HD720')
    framerate    = LaunchConfiguration('framerate',    default='30')    # some tags call it 'framerate'
    gpu_id       = LaunchConfiguration('gpu_id',       default='0')
    publish_tf   = LaunchConfiguration('publish_tf',   default='false')
    depth_mode   = LaunchConfiguration('depth_mode',   default='NONE')

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            # these names work on humble-v4.2.5; if in doubt: `ros2 launch zed_wrapper zed_camera.launch.py --show-args`
            'camera_model': camera_model,
            'camera_name':  camera_name,
            'resolution':   resolution,
            'framerate':    framerate,
            'gpu_id':       gpu_id,
            'publish_tf':   publish_tf,
            'depth_mode':   depth_mode,
        }.items()
    )

    # We’ll use the default camera_name 'zed2i' in the viewer/simplifier topic below.
    zed_rect_rgb = '/zed2i/zed_node/rgb/image_rect_color'

    simplifier = Node(
        package='frame_simplifier_pkg',
        executable='simplifier_node',
        name='frame_simplifier',
        output='screen',
        parameters=[{
            'image_in':  zed_rect_rgb,
            'image_out': '/image_raw_10hz',
            'fps':       10.0
        }]
    )

    keypoints = Node(
        package='keypoint_extractor_pkg',
        executable='keypoint_extractor_node',
        name='keypoint_extractor_node',
        output='screen',
        parameters=[{
            'image_topic':       '/image_raw_10hz',
            'drop_z':            True,
            'publish_annotated': True
        }]
    )

    lstm = Node(
        package='gesture_classifiers_pkg',
        executable='lstm_onnx_node',
        name='lstm_onnx_node',
        output='screen',
        parameters=[{
            'weights_path':    os.path.expanduser('~/amr_gesture_ws/models/lstm/jester20b_12cls/model_30x84.onnx'),
            'normalizer_path': os.path.expanduser('~/amr_gesture_ws/models/lstm/jester20b_12cls/normalizer.json'),
            'labels_path':     os.path.expanduser('~/amr_gesture_ws/models/lstm/jester20b_12cls/labels.txt'),
            'conf_threshold':  0.80,
            'model_frames':    30,
            'min_frames':      12,
            'use_cuda':        True
        }]
    )

    recorder = Node(
        package='vlm_recorder_pkg',
        executable='recorder_node',
        name='recorder_node',
        output='screen',
        parameters=[{
            'image_topic':    '/image_raw_10hz',
            'data_root':      os.path.expanduser('~/amr_gesture_ws/data/training'),
            'buffer_seconds': 6.0,    # DOUBLE (fixes your earlier crash)
            'target_fps':     10      # INTEGER
        }]
    )

    vlm = Node(
        package='vlm_ros',
        executable='vlm_node',
        name='vlm_node',
        output='screen'
    )

    bridge = Node(
        package='vlm_bridge_pkg',
        executable='vlm_bridge_node',
        name='vlm_bridge_node',
        output='screen'
    )

    rgb_view = Node(
        package='image_view',
        executable='image_view',
        name='view_raw_10hz',
        output='screen',
        remappings=[('image', '/image_raw_10hz')]
    )

    annotated_view = Node(
        package='image_view',
        executable='image_view',
        name='view_annotated',
        output='screen',
        remappings=[('image', '/image_annotated')]
    )

    return LaunchDescription([
        # Make sure ZED SDK + CUDA are visible
        SetEnvironmentVariable('LD_LIBRARY_PATH', '/usr/local/zed/lib'),
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),

        DeclareLaunchArgument('camera_model', default_value=camera_model),
        DeclareLaunchArgument('camera_name',  default_value=camera_name),
        DeclareLaunchArgument('resolution',   default_value=resolution),
        DeclareLaunchArgument('framerate',    default_value=framerate),
        DeclareLaunchArgument('gpu_id',       default_value=gpu_id),
        DeclareLaunchArgument('publish_tf',   default_value=publish_tf),
        DeclareLaunchArgument('depth_mode',   default_value=depth_mode),

        zed_launch,
        simplifier,
        keypoints,
        lstm,
        recorder,
        vlm,
        bridge,
        rgb_view,
        annotated_view,
    ])
