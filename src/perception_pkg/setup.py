from setuptools import setup

package_name = 'perception_pkg'

setup(
    name='perception_pkg',  # <-- underscore, not dash
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/perception_pkg']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'requests'],
    zip_safe=False,
    maintainer='Sayak',
    maintainer_email='sayak.deb26@gmail.com',
    description='Perception node that calls the local VLM HTTP service and publishes amr_interfaces/Intent.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perception_node = perception_pkg.perception_node:main',
        ],
    },
)
