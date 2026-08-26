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
            "sim_executor = a3_bringup.sim_executor:main",
            "zero_to_work_publisher = a3_bringup.zero_to_work_publisher:main",
            "gravity_torque_node = a3_bringup.gravity_torque_node:main",
            "follow_joint_trajectory_action = a3_bringup.follow_joint_trajectory_action:main",
            "move_to_pose_ik_node = a3_bringup.move_to_pose_ik_node:main",
            "draw_rectangle_demo = a3_bringup.draw_rectangle_demo:main",
            "servo_mode_bridge = a3_bringup.servo_mode_bridge:main",
        ],
    },
)
