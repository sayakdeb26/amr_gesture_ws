from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("amr_sim_pkg")
    default_world = os.path.join(pkg_share, "worlds", "amr_lab.world")

    world_arg = DeclareLaunchArgument(
        "world", default_value=default_world, description="Full path to the world file"
    )

    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", LaunchConfiguration("world")], output="screen"
    )

    return LaunchDescription([world_arg, gazebo])
