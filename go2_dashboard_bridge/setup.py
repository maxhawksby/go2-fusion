from setuptools import find_packages, setup

package_name = "go2_dashboard_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UConn Team 48",
    maintainer_email="team48@uconn.edu",
    description="Stable local API between the Oak Control dashboard and ROS 2.",
    license="BSD-2-Clause",
    entry_points={
        "console_scripts": [
            "dashboard_bridge = go2_dashboard_bridge.bridge:main",
        ],
    },
)
