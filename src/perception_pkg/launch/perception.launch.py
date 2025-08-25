from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    vlm_url = os.environ.get('VLM_URL', 'http://127.0.0.1:8000')
    return LaunchDescription([
        Node(
            package='perception_pkg',
            executable='perception_node',
            name='perception_node',
            output='screen',
            env={'VLM_URL': vlm_url},
        )
    ])
