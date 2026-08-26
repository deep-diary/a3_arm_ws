#!/usr/bin/env python3
"""Description + can bridge only (no teleop) for headless board bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("a3_description")
    bridge_share = get_package_share_directory("a3_can_bridge")
    can0_name = LaunchConfiguration("can0_name")
    use_power_sequence = LaunchConfiguration("use_power_sequence")
    use_gravity_compensation = LaunchConfiguration("use_gravity_compensation")

    xacro_file = os.path.join(desc_share, "urdf", "el_a3.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_mock_hardware:=true"]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("can0_name", default_value="can0"),
        DeclareLaunchArgument("use_power_sequence", default_value="true"),
        DeclareLaunchArgument(
            "use_gravity_compensation",
            default_value="false",
            description="启动 gravity_torque_node 并打开 motor_protocol 重力 MIT 前馈",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bridge_share, "launch", "can_bridge.launch.py")
            ),
            launch_arguments={
                "can0_name": can0_name,
                "use_power_sequence": use_power_sequence,
                "enable_gravity_compensation": use_gravity_compensation,
            }.items(),
        ),
        Node(
            package="a3_bringup",
            executable="trajectory_bridge",
            name="a3_trajectory_bridge",
            output="screen",
        ),
        Node(
            package="a3_bringup",
            executable="gravity_torque_node",
            name="a3_gravity_torque",
            output="screen",
            parameters=[{"enabled": True}],
            condition=IfCondition(use_gravity_compensation),
        ),
    ])
