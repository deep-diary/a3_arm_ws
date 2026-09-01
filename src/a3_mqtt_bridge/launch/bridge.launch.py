#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # 本机 .bashrc 设了 PYTHONNOUSERSITE=1，会屏蔽 pip --user 安装的
            # paho-mqtt；此处解除以加载 ~/.local 的依赖。
            SetEnvironmentVariable("PYTHONNOUSERSITE", ""),
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="可选：覆盖 config/bridge.yaml 的绝对路径（空则用包内默认）",
            ),
            Node(
                package="a3_mqtt_bridge",
                executable="ros2mqtt_bridge",
                name="ros2mqtt_bridge",
                output="screen",
            ),
        ]
    )
