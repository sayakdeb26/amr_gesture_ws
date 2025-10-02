from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='vlm_ros', executable='vlm_node', name='vlm_node', output='screen')
    ])
