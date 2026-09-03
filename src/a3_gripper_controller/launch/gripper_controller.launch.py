#!/usr/bin/env python3
"""Launch a3_gripper_controller（L7 夹爪力控节点，需求 F24-F26）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("a3_gripper_controller")
    config_file = LaunchConfiguration("config_file")
    require_gate = LaunchConfiguration("require_gate")

    default_cfg = os.path.join(share, "config", "gripper_config.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_cfg,
                description="gripper_controller 参数文件绝对路径",
            ),
            DeclareLaunchArgument(
                "require_gate",
                default_value="false",
                description="力控是否要求 gate_open（真机电源序列在环时设 true）",
            ),
            Node(
                package="a3_gripper_controller",
                executable="gripper_controller",
                name="gripper_controller_node",
                output="screen",
                parameters=[config_file, {"require_gate": require_gate}],
            ),
        ]
    )
