from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ledMerger = Node(
        package ='led_controller',
        executable='LEDMerger',
    )

    personLED = Node(
        package = 'led_controller',
        executable='personLocaterLED'
    )

    directionLED = Node(
        package = 'led_controller',
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

    behviorTree = Node(
        package='behavior_tree',
        executable='behavior_tree'
    )

    screen = Node(
        package='screen',
        executable='display_on_screen'
    )

    ld.add_action(ledMerger)
    ld.add_action(personLED)
    ld.add_action(directionLED)
    ld.add_action(virtualLedStrip)
    ld.add_action(behviorTree)
    ld.add_action(screen)
    ld.add_action(stoppingLED)

    return ld



