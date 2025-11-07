from setuptools import setup
from glob import glob
package_name = 'vlm_recorder_pkg'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sayak',
    maintainer_email='sayak.deb26@gmail.com',
    description='Recorder: 4s@10fps ring buffer → MP4 on /lstm/unknown',
    license='MIT',
    entry_points={'console_scripts': ['recorder_node = vlm_recorder_pkg.recorder_node:main']},
)
