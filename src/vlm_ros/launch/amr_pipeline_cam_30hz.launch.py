from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    frame_simplifier = Node(
        package='frame_simplifier_pkg',
        executable='simplifier_node',
        name='frame_simplifier',
        output='screen',
        parameters=[{'target_fps': 30, 'kpx_bridge': False}],
        remappings=[
            ('/image_raw',      '/image_raw'),      # webcam input
            ('/image_raw_30hz', '/image_raw_30hz')  # 30Hz out
        ],
    )

    keypoint_extractor = Node(
        package='keypoint_extractor_pkg',
        executable='keypoint_extractor_node',
        name='keypoint_extractor_node',
        output='screen',
        parameters=[{'window_size': 30, 'drop_z': True}],
        remappings=[
            ('/image_raw',        '/image_raw_30hz'),
            ('/keypoints_window', '/lstm/keypoints_window')
        ],
    )

    return LaunchDescription([frame_simplifier, keypoint_extractor])
