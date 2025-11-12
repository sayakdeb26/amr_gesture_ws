from setuptools import setup, find_packages
package_name = "frame_simplifier_pkg"

setup(
    name=package_name,
    version="0.0.3",
    packages=find_packages(include=[package_name+"*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sayak",
    maintainer_email="",
    description="Frame downsampler node",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "simplifier_node = frame_simplifier_pkg.simplifier_node:main",
            "frame_simplifier = frame_simplifier_pkg.simplifier_node:main",
        ],
    },
)
