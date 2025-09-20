from setuptools import setup

package_name = 'mp_gesture_pkg'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='sayak',
    maintainer_email='sayak.deb26@gmail.com',
    description='Rule-based hand gesture recognizer (MediaPipe Hands) → /intents_raw',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mp_gesture_node = mp_gesture_pkg.mp_gesture_node:main',
        ],
    },
)
