from setuptools import setup
import os
from glob import glob

package_name = "a3_mqtt_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="a3_arm",
    maintainer_email="dev@example.com",
    description="ROS2-to-MQTT telemetry bridge for A3 arm (rk3588)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ros2mqtt_bridge = a3_mqtt_bridge.ros2mqtt_bridge:main",
        ],
    },
)
