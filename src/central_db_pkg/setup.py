from glob import glob
from setuptools import setup
package_name = 'central_db_pkg'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # Python package folder
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/central_db_pkg']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/central_db.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Sayak',
    maintainer_email='sayak.deb26@gmail.com',
    description='Central database node (JSON-based).',
    license='MIT',
    entry_points={'console_scripts': [
        'central_db_node = central_db_pkg.central_db_node:main',
    ]},
)
