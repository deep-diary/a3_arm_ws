#!/usr/bin/env python3
"""Edge Wave A simulation: sim_executor + gravity + zero→work trajectory (no CAN)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    duration_s = LaunchConfiguration("duration_s")

    desc_share = get_package_share_directory("a3_description")
    urdf_path = os.path.join(desc_share, "urdf", "el_a3.urdf")
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        output="screen",
    )

    sim_exec = Node(
        package="a3_bringup",
        executable="sim_executor",
        name="a3_sim_executor",
        output="screen",
        parameters=[
            {
                "trajectory_topic": "/joint_group_effort_controller/joint_trajectory",
                "joint_states_topic": "/joint_states",
                "rate_hz": 50.0,
            }
        ],
    )

    gravity = Node(
        package="a3_bringup",
        executable="gravity_torque_node",
        name="a3_gravity_torque",
        output="screen",
        parameters=[{"enabled": True}],
    )

    traj_pub = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="a3_bringup",
                executable="zero_to_work_publisher",
                name="zero_to_work_publisher",
                output="screen",
                parameters=[
                    {
                        "start_pose": "zero",
                        "goal_pose": "work",
                        "duration_s": duration_s,
                        "num_waypoints": 11,
                        "delay_s": 0.5,
                        "publish_once": True,
                    }
                ],
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(desc_share, "config", "el_a3_view.rviz")],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="启动 RViz（el_a3_view.rviz）；无屏/脚本验收保持 false",
            ),
            DeclareLaunchArgument("duration_s", default_value="3.0"),
            rsp,
            sim_exec,
            gravity,
            traj_pub,
            rviz,
        ]
    )
