#!/usr/bin/env python3
"""Launch a3_arm_controller (orchestration facade) + a3_arm_monitor (F50 看门狗)."""

import os

from a3_robot_model import robot_model
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

    # F174：关节形态随系统级模型配置（arm_controller.yaml 里 joint_names 为 7 关节，
    # 6 轴 + 连杆形态下需覆盖，否则 F52 档位配对校验报错；switch_controllers 同理
    # 去掉不存在的 gripper_controller，避免 STRICT 切换被拒）。
    cfg = robot_model.load()
    include_gripper = bool(cfg.get("include_gripper"))
    fsm_overrides = {
        "joint_names": list(cfg["joint_names"]),
        "named_poses_pkg": "" if include_gripper else str(cfg["description_package"]),
    }
    if not include_gripper:
        fsm_overrides["switch_controllers"] = ["arm_controller"]

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
                parameters=[config_file, {"require_gate": require_gate}, fsm_overrides],
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
