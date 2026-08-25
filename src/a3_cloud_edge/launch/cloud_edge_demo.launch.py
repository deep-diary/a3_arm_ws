#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("a3_cloud_edge")
    desc_pkg = FindPackageShare("a3_description")

    agent_ip = LaunchConfiguration("agent_ip")
    agent_port = LaunchConfiguration("agent_port")
    use_mock = LaunchConfiguration("use_mock")
    use_rsp = LaunchConfiguration("use_rsp")

    declare_agent_ip = DeclareLaunchArgument("agent_ip", default_value="127.0.0.1")
    declare_agent_port = DeclareLaunchArgument("agent_port", default_value="8888")
    declare_use_mock = DeclareLaunchArgument("use_mock", default_value="true")
    declare_use_rsp = DeclareLaunchArgument("use_rsp", default_value="true")

    agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "micro_ros_agent.launch.py"])
        ),
        launch_arguments={"port": agent_port}.items(),
    )

    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "el_a3.urdf.xacro"])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_mock_hardware:=true"]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(use_rsp),
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
        },
        condition=IfCondition(use_mock),
    )

    test_pub = Node(
        package="a3_cloud_edge",
        executable="trajectory_test_publisher.py",
        name="trajectory_test_publisher",
        output="screen",
        parameters=[{"delay_sec": 5.0, "duration_sec": 2.0}],
    )

    return LaunchDescription([
        declare_agent_ip,
        declare_agent_port,
        declare_use_mock,
        declare_use_rsp,
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_microxrcedds"),
        SetEnvironmentVariable("XRCE_AGENT_IP", agent_ip),
        SetEnvironmentVariable("XRCE_AGENT_PORT", agent_port),
        agent_launch,
        robot_state_publisher,
        mock_client,
        test_pub,
    ])
