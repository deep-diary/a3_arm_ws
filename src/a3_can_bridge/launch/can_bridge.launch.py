#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_package = FindPackageShare("a3_can_bridge")

    can0_name = LaunchConfiguration("can0_name")
    can1_name = LaunchConfiguration("can1_name")
    gains_file = LaunchConfiguration("gains_file")
    bridge_file = LaunchConfiguration("bridge_file")
    power_sequence_file = LaunchConfiguration("power_sequence_file")
    use_power_sequence = LaunchConfiguration("use_power_sequence")
    enable_gravity_compensation = LaunchConfiguration("enable_gravity_compensation")

    declare_can0 = DeclareLaunchArgument(
        "can0_name", default_value="can0", description="SocketCAN interface name for front legs"
    )
    declare_can1 = DeclareLaunchArgument(
        "can1_name", default_value="can1", description="SocketCAN interface name for rear legs"
    )
    declare_gains = DeclareLaunchArgument(
        "gains_file",
        default_value=PathJoinSubstitution([bridge_package, "config", "control_gains.yaml"]),
        description="MIT gain parameters for motor_protocol_node",
    )
    declare_bridge = DeclareLaunchArgument(
        "bridge_file",
        default_value=PathJoinSubstitution([bridge_package, "config", "bridge.yaml"]),
        description="SocketCAN transport parameters",
    )
    declare_power_sequence_file = DeclareLaunchArgument(
        "power_sequence_file",
        default_value=PathJoinSubstitution([bridge_package, "config", "power_sequence.yaml"]),
        description="Power sequence state machine parameters",
    )
    declare_use_power_sequence = DeclareLaunchArgument(
        "use_power_sequence",
        default_value="true",
        description="是否启用遥控上下电状态机节点 power_sequence_node",
    )
    declare_enable_gravity = DeclareLaunchArgument(
        "enable_gravity_compensation",
        default_value="false",
        description="将 Pinocchio 重力力矩写入 MIT tau 前馈（需 /a3/gravity_torque）",
    )

    can_transport = Node(
        package="a3_can_bridge",
        executable="can_transport_node",
        name="can_transport_node",
        output="screen",
        parameters=[
            bridge_file,
            {"can0_name": can0_name},
            {"can1_name": can1_name},
        ],
    )

    motor_protocol = Node(
        package="a3_can_bridge",
        executable="motor_protocol_node",
        name="motor_protocol_node",
        output="screen",
        parameters=[
            gains_file,
            {
                "enable_power_sequence_gate": use_power_sequence,
                "enable_gravity_compensation": enable_gravity_compensation,
            },
        ],
    )

    power_sequence = Node(
        package="a3_can_bridge",
        executable="power_sequence_node",
        name="power_sequence_node",
        output="screen",
        parameters=[power_sequence_file],
        condition=IfCondition(use_power_sequence),
    )

    return LaunchDescription(
        [
            declare_can0,
            declare_can1,
            declare_gains,
            declare_bridge,
            declare_power_sequence_file,
            declare_use_power_sequence,
            declare_enable_gravity,
            can_transport,
            motor_protocol,
            power_sequence,
        ]
    )
