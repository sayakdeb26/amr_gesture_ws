from setuptools import setup

package_name = "repo_manager_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="JSON repo manager for gestures & mappings with profiles",
    entry_points={
        "console_scripts": [
            "repo_manager = repo_manager_pkg.repo_manager_node:main",
        ],
    },
)
