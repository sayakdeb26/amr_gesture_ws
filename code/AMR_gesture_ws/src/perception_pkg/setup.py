from setuptools import setup

package_name = 'perception_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # folder perception_pkg/ is the Python package
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/perception_pkg']),
        ('share/' + package_name, ['package.xml']),
        # add launch files later if you want them installed:
        # ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=False,
    maintainer='Sayak',
    maintainer_email='sayak.deb26@gmail.com',
    description='Perception node that calls VLM and publishes intents/unknowns.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perception_node = perception_pkg.perception_node:main',
        ],
    },
)
