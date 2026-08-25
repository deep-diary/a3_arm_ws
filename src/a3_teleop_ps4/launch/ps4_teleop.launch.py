#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("device_name", default_value=""),
        DeclareLaunchArgument("deadzone", default_value="0.12"),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[{
                "device_name": LaunchConfiguration("device_name"),
                "deadzone": LaunchConfiguration("deadzone"),
                "autorepeat_rate": 30.0,
            }],
            output="screen",
        ),
        Node(
            package="a3_teleop_ps4",
            executable="ps4_arm_teleop",
            name="ps4_arm_teleop",
            output="screen",
        ),
    ])
