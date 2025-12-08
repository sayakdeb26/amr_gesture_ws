#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    pkg_create3_ignition_bringup = get_package_share_directory('irobot_create_ignition_bringup')

    # Path to the standard Create3 Ignition launch file
    create3_ignition_launch = PathJoinSubstitution(
        [pkg_create3_ignition_bringup, 'launch', 'create3_ignition.launch.py'])

    # Declare use_rviz argument
    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz')

    # Include the standard launch file with our configuration
    create3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_ignition_launch]),
        launch_arguments=[
            ('namespace', ''),          # No namespace - topics at root
            ('use_rviz', use_rviz),    # Use argument
            ('spawn_dock', 'true'),    # Spawn the dock
            ('x', '0.0'),              # Initial X position
            ('y', '0.0'),              # Initial Y position
            ('z', '0.0'),              # Initial Z position
            ('yaw', '0.0')             # Initial Yaw orientation
        ]
    )

    # Set environment variables for GPU usage and Plugin Path
    env_render = SetEnvironmentVariable(name='IGN_RENDER_ENGINE', value='ogre2')
    env_gl = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='0')
    
    # Ensure Ignition can find the ros2_control plugin
    import os
    plugin_path = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    if '/opt/ros/humble/lib' not in plugin_path:
        if plugin_path:
            plugin_path += ':' + '/opt/ros/humble/lib'
        else:
            plugin_path = '/opt/ros/humble/lib'
            
    env_plugin = SetEnvironmentVariable(name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH', value=plugin_path)

    ld = LaunchDescription()
    ld.add_action(declare_use_rviz)
    ld.add_action(env_render)
    ld.add_action(env_gl)
    ld.add_action(env_plugin)
    ld.add_action(create3_sim)

    return ld
