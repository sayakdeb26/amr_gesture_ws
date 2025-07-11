from setuptools import find_packages, setup

package_name = 'led_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy'],
    zip_safe=True,
    maintainer='faps',
    maintainer_email='faps@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
     entry_points={
    'console_scripts': [
        "ledPublisher=led_controller.ledPublisher:main",
        "LEDMerger=led_controller.LEDMerger:main",
        "personLocaterLED=led_controller.personLocaterLED:main",
        "directionLED=led_controller.directionLED:main",
        "LEDvisualizer = led_controller.virtualLEDstrip:main",
        "stoppingLED = led_controller.stoppingLED:main"
    ],
},

)
