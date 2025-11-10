from setuptools import setup, find_packages

package_name = "frame_simplifier_pkg"

setup(
    name=package_name,
    version="0.0.2",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sayak",
    maintainer_email="sayak.deb26@gmail.com",
    description="Frame downsampler + Keypoints XY bridge (single node).",
    license="Apache-2.0",
    entry_points={
        'console_scripts': [
            'simplifier_node = frame_simplifier_pkg.simplifier_node:main',
        ],
    },
)

