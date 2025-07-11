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

    virtualLedStrip = Node(
        package='led_controller',
        executable='LEDvisualizer'
    )

    ld.add_action(ledMerger)
    ld.add_action(personLED)
    ld.add_action(directionLED)
    ld.add_action(virtualLedStrip)

    return ld



