#!/usr/bin/env python3
"""MoveIt Servo → joint trajectory (sim / bypass path)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    desc_share = get_package_share_directory("a3_description")
    moveit_share = get_package_share_directory("a3_moveit_config")

    xacro = os.path.join(desc_share, "urdf", "el_a3.urdf.xacro")
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro, " use_mock_hardware:=true"]),
            value_type=str,
        )
    }
    with open(os.path.join(moveit_share, "config", "el_a3.srdf"), "r", encoding="utf-8") as f:
        robot_description_semantic = {
            "robot_description_semantic": f.read()
        }
    with open(os.path.join(moveit_share, "config", "kinematics.yaml"), "r", encoding="utf-8") as f:
        kinematics = {"robot_description_kinematics": yaml.safe_load(f)}
    with open(os.path.join(moveit_share, "config", "servo_config.yaml"), "r", encoding="utf-8") as f:
        servo_yaml = yaml.safe_load(f)
    # Flatten: moveit_servo expects params at top level in some versions,
    # and under moveit_servo in others — provide both.
    servo_params = {"moveit_servo": servo_yaml}
    servo_params.update(servo_yaml)

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
            ),
            Node(
                package="a3_bringup",
                executable="sim_executor",
                name="a3_sim_executor",
                parameters=[{"trajectory_interpolation_method": "auto"}],
            ),
            Node(
                package="a3_bringup",
                executable="servo_mode_bridge",
                name="a3_servo_mode_bridge",
                parameters=[
                    {"twist_topic": "/servo_node/delta_twist_cmds"},
                    {"timeout_s": 0.5},
                ],
            ),
            Node(
                package="moveit_servo",
                executable="servo_node_main",
                name="servo_node",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics,
                    servo_params,
                ],
                remappings=[
                    ("~/delta_twist_cmds", "/servo_node/delta_twist_cmds"),
                    (
                        "~/command_out",
                        "/joint_group_effort_controller/joint_trajectory",
                    ),
                ],
                output="screen",
            ),
        ]
    )
