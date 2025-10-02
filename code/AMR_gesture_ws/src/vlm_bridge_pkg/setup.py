from setuptools import setup
from glob import glob

package_name = 'vlm_bridge_pkg'

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
    description='Bridge: /lstm/unknown → wait for /recorder/clip_ready → call /vlm/infer → /vlm/confirm_request',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 👇 now matches your actual file name
            'bridge_node = vlm_bridge_pkg.vlm_bridge_node:main',
        ],
    },
)

