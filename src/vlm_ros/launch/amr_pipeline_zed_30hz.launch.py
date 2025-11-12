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
    frame_rate   = LaunchConfiguration('frame_rate',   default='30')
    gpu_id       = LaunchConfiguration('gpu_id',       default='0')
    publish_tf   = LaunchConfiguration('publish_tf',   default='false')
    depth_mode   = LaunchConfiguration('depth_mode',   default='NONE')

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('zed_wrapper') + '/launch/zed_camera.launch.py'
        ),
        launch_arguments={
            'camera_model': camera_model,
            'camera_name':  camera_name,
            'resolution':   resolution,
            'frame_rate':   frame_rate,
            'gpu_id':       gpu_id,
            'publish_tf':   publish_tf,
            'depth_mode':   depth_mode,
        }.items()
    )

    zed_rgb_topic = '/zed2i/zed_node/rgb/image_rect_color'

    frame_simplifier = Node(
        package='frame_simplifier_pkg',
        executable='simplifier_node',
        name='frame_simplifier',
        output='screen',
        parameters=[{
            'target_fps': 30,
            'kpx_bridge': False,   # <-- disables any /lstm/* subscribers/publishers
        }],
        remappings=[
            ('/image_raw',      zed_rgb_topic),   # input from ZED
            ('/image_raw_30hz', '/image_raw_30hz')# output @30Hz
        ],
    )

    keypoint_extractor = Node(
        package='keypoint_extractor_pkg',
        executable='keypoint_extractor_node',
        name='keypoint_extractor_node',
        output='screen',
        parameters=[{'window_size': 30, 'drop_z': True}],
        remappings=[
            ('/image_raw',        '/image_raw_30hz'),      # <-- subscribe to 30Hz
            ('/keypoints_window', '/lstm/keypoints_window')
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_model',  default_value='zed2i'),
        DeclareLaunchArgument('camera_name',   default_value='zed2i'),
        DeclareLaunchArgument('resolution',    default_value='HD720'),
        DeclareLaunchArgument('frame_rate',    default_value='30'),
        DeclareLaunchArgument('gpu_id',        default_value='0'),
        DeclareLaunchArgument('publish_tf',    default_value='false'),
        DeclareLaunchArgument('depth_mode',    default_value='NONE'),
        zed_launch,
        frame_simplifier,
        keypoint_extractor,
    ])
