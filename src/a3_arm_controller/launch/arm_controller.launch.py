#!/usr/bin/env python3
"""Launch a3_arm_controller (orchestration facade)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("a3_arm_controller")
    config_file = LaunchConfiguration("config_file")
    require_gate = LaunchConfiguration("require_gate")

    default_cfg = os.path.join(share, "config", "arm_controller.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_cfg,
                description="a3_arm_controller 参数文件绝对路径",
            ),
            DeclareLaunchArgument(
                "require_gate",
                default_value="false",
                description="运动命令是否要求 gate_open",
            ),
            Node(
                package="a3_arm_controller",
                executable="arm_controller",
                name="a3_arm_controller",
                output="screen",
                parameters=[config_file, {"require_gate": require_gate}],
            ),
        ]
    )
