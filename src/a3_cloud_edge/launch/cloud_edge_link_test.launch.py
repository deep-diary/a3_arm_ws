#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def moveit_overlay_env():
    extra = {}
    roots = []
    for p in os.environ.get("COLCON_PREFIX_PATH", "").split(os.pathsep):
        if p:
            roots.append(os.path.join(os.path.dirname(p), "src", "third_party", "moveit_debs"))
    roots.append(os.path.expanduser("~/a3_arm_ws/src/third_party/moveit_debs"))
    for root in roots:
        humble = os.path.join(root, "opt", "ros", "humble")
        if not os.path.isdir(os.path.join(humble, "share", "moveit_ros_move_group")):
            continue
        usr_lib = os.path.join(root, "usr", "lib", "aarch64-linux-gnu")
        lib_paths = [
            os.path.join(humble, "lib"),
            os.path.join(humble, "lib", "aarch64-linux-gnu"),
            usr_lib,
            os.environ.get("LD_LIBRARY_PATH", ""),
        ]
        extra["AMENT_PREFIX_PATH"] = humble + os.pathsep + os.environ.get(
            "AMENT_PREFIX_PATH", ""
        )
        extra["LD_LIBRARY_PATH"] = os.pathsep.join([p for p in lib_paths if p])
        py = os.path.join(humble, "local", "lib", "python3.10", "dist-packages")
        if os.path.isdir(py):
            extra["PYTHONPATH"] = py + os.pathsep + os.environ.get("PYTHONPATH", "")
        return extra
    return extra


def generate_launch_description():
    pkg = FindPackageShare("a3_cloud_edge")

    agent_ip = LaunchConfiguration("agent_ip")
    agent_port = LaunchConfiguration("agent_port")
    use_mock = LaunchConfiguration("use_mock")
    use_moveit = LaunchConfiguration("use_moveit")
    traj_mode = LaunchConfiguration("traj_mode")
    delta_x_m = LaunchConfiguration("delta_x_m")
    gravity_ff_scale = LaunchConfiguration("gravity_ff_scale")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_agent_ip = DeclareLaunchArgument("agent_ip", default_value="127.0.0.1")
    declare_agent_port = DeclareLaunchArgument("agent_port", default_value="8888")
    declare_use_mock = DeclareLaunchArgument(
        "use_mock",
        default_value="true",
        description="Linux XRCE mock. Set false to leave room for an ESP32 Client.",
    )
    declare_use_moveit = DeclareLaunchArgument(
        "use_moveit",
        default_value="true",
        description="Plan with MoveIt (S1). false = PyKDL/sine test publisher.",
    )
    declare_traj_mode = DeclareLaunchArgument(
        "traj_mode",
        default_value="forward_dx",
        description="sine | forward_dx (only when use_moveit:=false)",
    )
    declare_delta_x = DeclareLaunchArgument("delta_x_m", default_value="0.10")
    declare_grav_scale = DeclareLaunchArgument("gravity_ff_scale", default_value="1.0")
    declare_rviz = DeclareLaunchArgument("use_rviz", default_value="false")

    agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "micro_ros_agent.launch.py"])
        ),
        launch_arguments={"port": agent_port}.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "moveit_planning.launch.py"])
        ),
        launch_arguments={"use_rviz": use_rviz}.items(),
        condition=IfCondition(use_moveit),
    )

    mock_client = Node(
        package="a3_microros_mock",
        executable="microros_mock_client",
        name="microros_mock_client",
        output="screen",
        parameters=[PathJoinSubstitution([pkg, "config", "cloud_edge.yaml"])],
        additional_env={
            "RMW_IMPLEMENTATION": "rmw_microxrcedds",
            "XRCE_AGENT_IP": agent_ip,
            "XRCE_AGENT_PORT": agent_port,
            "RMW_UXRCE_DEFAULT_UDP_IP": agent_ip,
            "RMW_UXRCE_DEFAULT_UDP_PORT": agent_port,
        },
        condition=IfCondition(use_mock),
    )

    gravity_node = Node(
        package="a3_cloud_edge",
        executable="gravity_compensation_node",
        name="gravity_compensation_node",
        output="screen",
        parameters=[
            {
                "input_topic": "/a3/planned_joint_trajectory",
                "output_topic": "/joint_group_effort_controller/joint_trajectory",
                "gravity_ff_scale": ParameterValue(gravity_ff_scale, value_type=float),
                "max_points": 32,
            }
        ],
    )

    moveit_plan = Node(
        package="a3_cloud_edge",
        executable="moveit_plan_node.py",
        name="moveit_plan_node",
        output="screen",
        additional_env=moveit_overlay_env(),
        parameters=[
            {
                "delay_sec": 8.0,
                "delta_x_m": ParameterValue(delta_x_m, value_type=float),
                "output_topic": "/a3/planned_joint_trajectory",
            }
        ],
        condition=IfCondition(use_moveit),
    )

    test_pub = Node(
        package="a3_cloud_edge",
        executable="trajectory_test_publisher.py",
        name="trajectory_test_publisher",
        output="screen",
        parameters=[
            {
                "trajectory_topic": "/a3/planned_joint_trajectory",
                "delay_sec": 5.0,
                "duration_sec": 2.0,
                "mode": ParameterValue(traj_mode, value_type=str),
                "delta_x_m": ParameterValue(delta_x_m, value_type=float),
            }
        ],
        condition=UnlessCondition(use_moveit),
    )

    return LaunchDescription([
        declare_agent_ip,
        declare_agent_port,
        declare_use_mock,
        declare_use_moveit,
        declare_traj_mode,
        declare_delta_x,
        declare_grav_scale,
        declare_rviz,
        agent_launch,
        mock_client,
        gravity_node,
        moveit_launch,
        moveit_plan,
        test_pub,
    ])
