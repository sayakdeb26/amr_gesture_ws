# setup.py (inside vlm_videollava_pkg)

from setuptools import setup

package_name = 'vlm_videollava_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sayak',
    maintainer_email='you@example.com',
    description='Video-LLaVA backend for /vlm/infer service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlm_node_videollava = vlm_videollava_pkg.vlm_node_videollava:main',
        ],
    },
)

