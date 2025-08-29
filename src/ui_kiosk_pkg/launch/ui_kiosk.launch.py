from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='ui_kiosk_pkg', executable='ui_kiosk_node', name='ui_kiosk_node'),
    ])
