from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

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

    virtualLedStrip = Node(
        package='led_controller',
        executable='LEDvisualizer'
    )

    stoppingLED = Node(
        package='led_controller',
        executable='stoppingLED'
    )

    # micro-ROS Agent Node
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0'],
        output='screen'
    )

    # Aktionen zum LaunchDescription hinzufügen
    ld.add_action(ledMerger)
    ld.add_action(personLED)
    ld.add_action(directionLED)
    ld.add_action(virtualLedStrip)
    ld.add_action(stoppingLED)
    ld.add_action(micro_ros_agent)  # <-- hier wird der Agent hinzugefügt

    return ld
