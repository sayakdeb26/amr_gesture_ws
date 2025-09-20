from setuptools import setup

package_name = 'vlm_bridge_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sayak',
    maintainer_email='sayak.deb26@gmail.com',
    description='Bridge low-conf LSTM → VLM service → ConfirmRequest → Intent',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vlm_bridge_node = vlm_bridge_pkg.vlm_bridge_node:main',
        ],
    },
)
