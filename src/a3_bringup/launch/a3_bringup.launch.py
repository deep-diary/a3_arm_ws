#!/usr/bin/env python3
"""Full A3 arm bringup: robot_state_publisher + can bridge + trajectory bridge + optional PS4."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("a3_description")
    bridge_share = get_package_share_directory("a3_can_bridge")

    use_rviz = LaunchConfiguration("use_rviz")
    use_sw_render = LaunchConfiguration("use_sw_render")
    use_teleop = LaunchConfiguration("use_teleop")
    use_power_sequence = LaunchConfiguration("use_power_sequence")
    use_gravity_compensation = LaunchConfiguration("use_gravity_compensation")
    can0_name = LaunchConfiguration("can0_name")
    gains_file = LaunchConfiguration("gains_file")
    motor_map_file = LaunchConfiguration("motor_map_file")

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
            "enable_gravity_compensation": use_gravity_compensation,
            "gains_file": gains_file,
            "motor_map_file": motor_map_file,
        }.items(),
    )

    traj_bridge = Node(
        package="a3_bringup",
        executable="trajectory_bridge",
        name="a3_trajectory_bridge",
        output="screen",
    )

    gravity = Node(
        package="a3_bringup",
        executable="gravity_torque_node",
        name="a3_gravity_torque",
        output="screen",
        parameters=[{"enabled": True}],
        condition=IfCondition(use_gravity_compensation),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("a3_teleop_ps4"),
                "launch",
                "ps4_teleop.launch.py",
            )
        ),
        launch_arguments={"auto_start_servo": "false"}.items(),
        condition=IfCondition(use_teleop),
    )

    rviz_cfg = os.path.join(desc_share, "config", "el_a3_view.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        # 软件渲染规避 RK3588 panfrost GPU hang（LL-027）
        prefix=IfElseSubstitution(use_sw_render, "env LIBGL_ALWAYS_SOFTWARE=1", ""),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "use_sw_render",
            default_value="true",
            description="RViz 软件渲染 LIBGL_ALWAYS_SOFTWARE=1（RK3588 默认开，LL-027）",
        ),
        DeclareLaunchArgument("use_teleop", default_value="true"),
        DeclareLaunchArgument("use_power_sequence", default_value="true"),
        DeclareLaunchArgument(
            "use_gravity_compensation",
            default_value="false",
            description="启动 gravity_torque_node 并打开 motor_protocol 重力 MIT 前馈",
        ),
        DeclareLaunchArgument("can0_name", default_value="can0"),
        # F52 档位：缺电机时（如事故后只剩 L1–L5）用 5J 档整套替换——
        #   gains_file:=.../control_gains_5j.yaml motor_map_file:=.../motor_map_5j.yaml
        # 两个文件必须配对（逐关节数组长度 = joint_names 长度），否则执行层报
        # 「F52 档位配置非法」并退回 7J 默认档。默认值即 7J 档，行为不变。
        DeclareLaunchArgument(
            "gains_file",
            default_value=os.path.join(bridge_share, "config", "control_gains.yaml"),
            description="MIT 增益/限位参数（缺电机时换 control_gains_5j.yaml，须与 motor_map_file 配对）",
        ),
        DeclareLaunchArgument(
            "motor_map_file",
            default_value=os.path.join(bridge_share, "config", "motor_map.yaml"),
            description="关节/电机档位清单（缺电机时换 motor_map_5j.yaml）",
        ),
        rsp,
        can_bridge,
        traj_bridge,
        gravity,
        teleop,
        rviz,
    ])
