from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch-time parameters with sensible defaults
    vlm_url = LaunchConfiguration("vlm_url")
    intent_topic = LaunchConfiguration("intent_topic")
    period_sec = LaunchConfiguration("period_sec")

    return LaunchDescription(
        [
            # Arguments you can override at launch time
            DeclareLaunchArgument(
                "vlm_url",
                default_value="http://127.0.0.1:8000",
                description="Base URL for the local VLM FastAPI service.",
            ),
            DeclareLaunchArgument(
                "intent_topic",
                default_value="/intents_raw",
                description="Topic to publish amr_interfaces/Intent messages to.",
            ),
            DeclareLaunchArgument(
                "period_sec",
                default_value="2.0",
                description="Seconds between VLM polls.",
            ),

            # Perception node
            Node(
                package="perception_pkg",
                executable="perception_node",
                name="perception_node",
                output="screen",
                parameters=[
                    {
                        "vlm_url": vlm_url,
                        "publish_topic": intent_topic,
                        "period_sec": period_sec,
                    }
                ],
            ),
        ]
    )

