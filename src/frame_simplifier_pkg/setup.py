from setuptools import setup

package_name = 'frame_simplifier_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sayak',
    maintainer_email='sayak.deb26@gmail.com',
    description='Lightweight frame simplifier: resizes, converts to bgr8, and republishes at fixed FPS.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simplifier_node = frame_simplifier_pkg.simplifier_node:main',
        ],
    },
)
