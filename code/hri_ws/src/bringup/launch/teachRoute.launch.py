from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Micro-ROS-Agent starten
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0'
        ],
        output='screen',
        respawn=True  # Optional: startet Agent neu bei Absturz
    )

    # LED Nodes
    ledMerger = Node(
        package='led_controller',
        executable='LEDMerger',
    )

    personLED = Node(
        package='led_controller',
        executable='personLocaterLED'
    )

    directionLED = Node(
        package='led_controller',
        executable='directionLED'
    )

    stoppingLED = Node(
        package='led_controller',
        executable='stoppingLED'
    )

    virtualLedStrip = Node(
        package='led_controller',
        executable='LEDvisualizer'
    )

    # Behavior Tree und Screen
    behaviorTree = Node(
        package='behavior_tree',
        executable='behavior_tree'
    )

    screen = Node(
        package='screen',
        executable='display_on_screen'
    )

    # Aktionen hinzufügen
    ld.add_action(micro_ros_agent)
    ld.add_action(ledMerger)
    ld.add_action(personLED)
    ld.add_action(directionLED)
    ld.add_action(stoppingLED)
    ld.add_action(virtualLedStrip)
    ld.add_action(behaviorTree)
    ld.add_action(screen)

    return ld
