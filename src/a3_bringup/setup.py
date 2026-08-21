from setuptools import setup
import os
from glob import glob

package_name = "a3_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="a3_arm",
    maintainer_email="dev@example.com",
    description="A3 arm bringup",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "trajectory_bridge = a3_bringup.trajectory_bridge:main",
            "rebot_remap_info = a3_bringup.rebot_remap_info:main",
        ],
    },
)
