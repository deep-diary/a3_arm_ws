#!/usr/bin/env python3
"""Full A3 arm bringup: robot_state_publisher + can bridge + trajectory bridge + optional PS4."""

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

    use_rviz = LaunchConfiguration("use_rviz")
    use_teleop = LaunchConfiguration("use_teleop")
    use_power_sequence = LaunchConfiguration("use_power_sequence")
    can0_name = LaunchConfiguration("can0_name")

    xacro_file = os.path.join(desc_share, "urdf", "el_a3.urdf.xacro")
    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_mock_hardware:=true",
            " can_interface:=", can0_name,
        ]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    can_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "can_bridge.launch.py")
        ),
        launch_arguments={
            "can0_name": can0_name,
            "use_power_sequence": use_power_sequence,
        }.items(),
    )

    traj_bridge = Node(
        package="a3_bringup",
        executable="trajectory_bridge",
        name="a3_trajectory_bridge",
        output="screen",
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("a3_teleop_ps4"),
                "launch",
                "ps4_teleop.launch.py",
            )
        ),
        condition=IfCondition(use_teleop),
    )

    rviz_cfg = os.path.join(desc_share, "config", "el_a3_view.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_teleop", default_value="true"),
        DeclareLaunchArgument("use_power_sequence", default_value="true"),
        DeclareLaunchArgument("can0_name", default_value="can0"),
        rsp,
        can_bridge,
        traj_bridge,
        teleop,
        rviz,
    ])
