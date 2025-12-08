#!/usr/bin/env python3
"""
Create3 Simulation + Gesture Pipeline with Video-LLaVA VLM

Usage:
    ros2 launch vlm_ros amr_gesture_create3_sim_videollava.launch.py
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    pkg_create3_sim = get_package_share_directory('create3_sim_integration')
    pkg_vlm_ros = get_package_share_directory('vlm_ros')

    sim_launch = PathJoinSubstitution(
        [pkg_create3_sim, 'launch', 'create3_gazebo_sim.launch.py'])

    pipeline_launch = PathJoinSubstitution(
        [pkg_vlm_ros, 'launch', 'pipeline_videollava.launch.py'])

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )

    sim_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sim_launch]),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items()
    )

    pipeline_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pipeline_launch])
    )

    ld = LaunchDescription()
    ld.add_action(use_rviz_arg)
    ld.add_action(sim_action)
    ld.add_action(pipeline_action)

    return ld
