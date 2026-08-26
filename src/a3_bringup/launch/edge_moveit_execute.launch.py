#!/usr/bin/env python3
"""Unified MoveIt-ish execute stack: sim executor + FJT + IK (+ optional gravity)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    use_gravity = LaunchConfiguration("use_gravity")
    use_ik = LaunchConfiguration("use_ik")
    run_demo = LaunchConfiguration("run_demo")
    use_rviz = LaunchConfiguration("use_rviz")
    desc_share = get_package_share_directory("a3_description")
    urdf = os.path.join(desc_share, "urdf", "el_a3.urdf")

    with open(urdf, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="true"),
            DeclareLaunchArgument("use_gravity", default_value="true"),
            DeclareLaunchArgument("use_ik", default_value="true"),
            DeclareLaunchArgument("run_demo", default_value="false"),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="启动 RViz（el_a3_view.rviz）；无屏/脚本验收保持 false",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                condition=IfCondition(use_sim),
            ),
            Node(
                package="a3_bringup",
                executable="sim_executor",
                name="a3_sim_executor",
                parameters=[{"trajectory_interpolation_method": "auto"}],
                condition=IfCondition(use_sim),
            ),
            Node(
                package="a3_bringup",
                executable="trajectory_bridge",
                name="a3_trajectory_bridge",
            ),
            Node(
                package="a3_bringup",
                executable="follow_joint_trajectory_action",
                name="a3_fjt_action",
                parameters=[{"require_gate": False}],
            ),
            Node(
                package="a3_bringup",
                executable="gravity_torque_node",
                name="a3_gravity_torque",
                condition=IfCondition(use_gravity),
            ),
            Node(
                package="a3_bringup",
                executable="move_to_pose_ik_node",
                name="a3_move_to_pose_ik",
                condition=IfCondition(use_ik),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="a3_bringup",
                        executable="draw_rectangle_demo",
                        name="a3_draw_rectangle_demo",
                        condition=IfCondition(run_demo),
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", os.path.join(desc_share, "config", "el_a3_view.rviz")],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
