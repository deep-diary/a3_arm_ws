#!/usr/bin/env python3
"""Launch melodion player node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    score_name = LaunchConfiguration("score_name")
    autoplay = LaunchConfiguration("autoplay")

    return LaunchDescription([
        DeclareLaunchArgument("score_name", default_value="scale"),
        DeclareLaunchArgument("autoplay", default_value="true"),
        Node(
            package="app_melodion",
            executable="melodion_player",
            name="a3_melodion_player",
            output="screen",
            parameters=[
                {"score_name": score_name},
                {"autoplay": autoplay},
            ],
        ),
    ])
