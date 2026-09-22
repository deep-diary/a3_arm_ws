#!/usr/bin/env python3
"""F74 验收附加层：在 F70 mock 栈之上启动编排 FSM（FJT action 后端）+ retime 节点。

直接以路径方式运行：
  ROS_DOMAIN_ID=60 ros2 launch scripts/a3_test/f74_extra.launch.py
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, file_path), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    desc_share = get_package_share_directory("a3_description")
    moveit_share = get_package_share_directory("a3_moveit_config")
    ctrl_share = get_package_share_directory("a3_arm_controller")

    xacro_file = os.path.join(desc_share, "urdf", "el_a3.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_mock_hardware:=true"]),
        value_type=str,
    )
    with open(os.path.join(moveit_share, "config", "el_a3.srdf"), "r", encoding="utf-8") as f:
        srdf = f.read()

    arm_cfg = os.path.join(ctrl_share, "config", "arm_controller.yaml")

    fsm = Node(
        package="a3_arm_controller",
        executable="arm_controller",
        name="a3_arm_controller",
        output="screen",
        parameters=[
            arm_cfg,
            {
                "require_gate": False,
                "control_backend": "fjt_action",
            },
        ],
    )

    jl_map = (load_yaml("a3_moveit_config", "config/joint_limits.yaml") or {}).get(
        "joint_limits", {}
    )
    velocity_limits = {
        name: float(d["max_velocity"])
        for name, d in jl_map.items()
        if d.get("has_velocity_limits") and d.get("max_velocity") is not None
    }
    acceleration_limits = {
        name: float(d["max_acceleration"])
        for name, d in jl_map.items()
        if d.get("has_acceleration_limits") and d.get("max_acceleration") is not None
    }
    retime = Node(
        package="a3_trajectory_processing",
        executable="retime_trajectory_node",
        name="a3_trajectory_processing",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": srdf},
            {"group_name": "arm_with_gripper"},
            {"velocity_limits": velocity_limits},
            {"acceleration_limits": acceleration_limits},
        ],
    )

    return LaunchDescription([fsm, retime])
