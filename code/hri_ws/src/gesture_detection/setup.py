from setuptools import find_packages, setup

package_name = 'gesture_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faps',
    maintainer_email='jakob.hartmann@faps.fau.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'retrain_model = gesture_detection.retrain_model_action:main',
            'record_gesture = gesture_detection.record_gesture:main',
            'initial_training_action = gesture_detection.initial_training_action:main',
            'keypoint_detection = gesture_detection.keypoint_detection:main',
            'get_gestures_service = gesture_detection.get_gestures_service:main'
        ],
    },
)
