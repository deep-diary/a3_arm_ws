#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("a3_cloud_edge")

    agent_ip = LaunchConfiguration("agent_ip")
    agent_port = LaunchConfiguration("agent_port")
    use_mock = LaunchConfiguration("use_mock")
    traj_mode = LaunchConfiguration("traj_mode")
    delta_x_m = LaunchConfiguration("delta_x_m")

    declare_agent_ip = DeclareLaunchArgument("agent_ip", default_value="127.0.0.1")
    declare_agent_port = DeclareLaunchArgument("agent_port", default_value="8888")
    declare_use_mock = DeclareLaunchArgument(
        "use_mock",
        default_value="true",
        description="Linux XRCE mock. Set false to leave room for an ESP32 Client.",
    )
    declare_traj_mode = DeclareLaunchArgument(
        "traj_mode",
        default_value="forward_dx",
        description="sine | forward_dx (TCP +X in base_link)",
    )
    declare_delta_x = DeclareLaunchArgument("delta_x_m", default_value="0.10")

    agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "micro_ros_agent.launch.py"])
        ),
        launch_arguments={"port": agent_port}.items(),
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

    test_pub = Node(
        package="a3_cloud_edge",
        executable="trajectory_test_publisher.py",
        name="trajectory_test_publisher",
        output="screen",
        parameters=[
            {
                "delay_sec": 5.0,
                "duration_sec": 2.0,
                "mode": ParameterValue(traj_mode, value_type=str),
                "delta_x_m": ParameterValue(delta_x_m, value_type=float),
            }
        ],
    )

    return LaunchDescription([
        declare_agent_ip,
        declare_agent_port,
        declare_use_mock,
        declare_traj_mode,
        declare_delta_x,
        agent_launch,
        mock_client,
        test_pub,
    ])
