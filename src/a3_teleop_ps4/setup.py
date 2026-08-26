from setuptools import setup
import os
from glob import glob

package_name = "a3_teleop_ps4"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "config", "mappings"),
            glob("config/mappings/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="a3_arm",
    maintainer_email="dev@example.com",
    description="PS4 teleop for A3 arm",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ps4_arm_teleop = a3_teleop_ps4.ps4_arm_teleop:main",
            "ps4_mapper = a3_teleop_ps4.ps4_mapper:main",
            "joy_dump = a3_teleop_ps4.joy_dump:main",
            "ds4_hid_node = a3_teleop_ps4.ds4_hid_node:main",
        ],
    },
)
