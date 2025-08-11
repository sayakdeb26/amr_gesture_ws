from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    # Behavior Tree und Screen
    behaviorTree = Node(
        package='behavior_tree',
        executable='behavior_tree'
    )

    screen = Node(
        package='screen',
        executable='display_on_screen'
    )

    gesture_initial_training_action= Node(
        package='gesture_detection',
        executable='initial_training_action'
    )

    gesture_keypoints= Node(
        package='gesture_detection',
        executable='keypoint_detection'
    )

    gesture_retrain_action= Node(
        package='gesture_detection',
        executable='retrain_model'
    )

    gesture_get_gestures_service= Node(
        package='gesture_detection',
        executable='get_gestures_service'
    )
    # Aktionen hinzufügen

    ld.add_action(behaviorTree)
    ld.add_action(screen)
    ld.add_action(gesture_initial_training_action)
    ld.add_action(gesture_keypoints)
    ld.add_action(gesture_retrain_action)
    ld.add_action(gesture_get_gestures_service)

    return ld
