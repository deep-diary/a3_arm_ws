#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("a3_cloud_edge")
    port = LaunchConfiguration("port")
    verbose = LaunchConfiguration("verbose")

    declare_port = DeclareLaunchArgument("port", default_value="8888", description="micro-ROS Agent UDP port")
    declare_verbose = DeclareLaunchArgument("verbose", default_value="4", description="Agent verbosity level")

    agent_script = PathJoinSubstitution([pkg, "scripts", "run_micro_ros_agent.sh"])

    agent = ExecuteProcess(
        cmd=[agent_script, port, verbose],
        output="screen",
    )

    return LaunchDescription([declare_port, declare_verbose, agent])
