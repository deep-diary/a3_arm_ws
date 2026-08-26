#!/usr/bin/env python3
"""PS4 teleop simulation: MoveIt Servo + sim_executor + YAML mapper + optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("a3_bringup")
    teleop_share = get_package_share_directory("a3_teleop_ps4")
    desc_share = get_package_share_directory("a3_description")

    use_rviz = LaunchConfiguration("use_rviz")
    enable_hid = LaunchConfiguration("enable_hid")
    device_name = LaunchConfiguration("device_name")
    mapping = LaunchConfiguration("mapping")

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "servo.launch.py")
        ),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_share, "launch", "ps4_teleop.launch.py")
        ),
        launch_arguments={
            "auto_start_servo": "true",
            "enable_hid": enable_hid,
            "device_name": device_name,
            "mapping": mapping,
        }.items(),
    )

    rviz_cfg = os.path.join(desc_share, "config", "el_a3_view.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("enable_hid", default_value="false"),
            DeclareLaunchArgument("device_name", default_value=""),
            DeclareLaunchArgument(
                "mapping",
                default_value="simple",
                description="手柄映射：simple（默认，无组合键）或 default（L1 死人开关）",
            ),
            servo,
            teleop,
            rviz,
        ]
    )
