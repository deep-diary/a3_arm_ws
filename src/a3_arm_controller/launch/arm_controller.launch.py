#!/usr/bin/env python3
"""Launch a3_arm_controller (orchestration facade) + a3_arm_monitor (F50 看门狗)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("a3_arm_controller")
    config_file = LaunchConfiguration("config_file")
    require_gate = LaunchConfiguration("require_gate")
    enable_monitor = LaunchConfiguration("enable_monitor")

    default_cfg = os.path.join(share, "config", "arm_controller.yaml")
    monitor_cfg = os.path.join(share, "config", "arm_monitor.yaml")

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
            DeclareLaunchArgument(
                "enable_monitor",
                default_value="true",
                description="是否启动 F50 故障监视看门狗（a3_arm_monitor）",
            ),
            Node(
                package="a3_arm_controller",
                executable="arm_controller",
                name="a3_arm_controller",
                output="screen",
                parameters=[config_file, {"require_gate": require_gate}],
            ),
            Node(
                package="a3_arm_controller",
                executable="arm_monitor",
                name="a3_arm_monitor",
                output="screen",
                parameters=[monitor_cfg],
                condition=IfCondition(enable_monitor),
            ),
        ]
    )
