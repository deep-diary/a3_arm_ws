#!/usr/bin/env python3
"""Web 仿真闭环整体启动（无电机 / 无 CAN / 无电池）。

用三个模拟节点替代硬件，构成「反馈≈指令 + 一阶跟随 + L7 接触弹簧」的闭环：

  * `sim_motor_node`（name=motor_protocol_node）：订阅
    `/joint_group_effort_controller/joint_trajectory`（来自 a3_arm_controller 多点
    轨迹 + gripper_controller 50 Hz 单点流），插值后回发 `/joint_states`
    （7 关节 position/velocity/effort，L7 用力矩弹簧模型）；并提供 `/a3/motor/*`
    、`/a3/motor/set_param`、`/a3/zero_torque/{start,stop}` 服务。
  * `sim_power_sequence_node`（name=power_sequence_node）：上电即
    `/power_sequence/gate_open=true`、state=Running。
  * `gravity_torque_node`（name=gravity_torque_node，对齐设备 YAML node 名）：
    Pinocchio 重力矩发布 `/a3/gravity_torque`。

链路：
  web --MQTT cmd--> ros2mqtt_bridge --/a3/arm/* 或 /a3/gripper/*--> controller
     --JointTrajectory--> sim_motor_node --/joint_states--> ros2mqtt_bridge
     --MQTT telemetry--> web（3D + 关节滑动条实时值 + 夹爪力控闭环）

注意：必须停掉 `a3_can_bridge`（真机 motor_protocol_node），否则 /joint_states
会有两个发布者冲突。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_gripper = LaunchConfiguration("use_gripper")
    use_rviz = LaunchConfiguration("use_rviz")

    desc_share = get_package_share_directory("a3_description")
    arm_share = get_package_share_directory("a3_arm_controller")
    bridge_share = get_package_share_directory("a3_mqtt_bridge")
    gripper_share = get_package_share_directory("a3_gripper_controller")

    urdf = os.path.join(desc_share, "urdf", "el_a3.urdf")
    with open(urdf, "r", encoding="utf-8") as f:
        robot_description = f.read()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    sim_motor = Node(
        package="a3_bringup",
        executable="sim_motor_node",
        name="motor_protocol_node",
        output="screen",
        parameters=[{"trajectory_interpolation_method": "auto"}],
    )

    sim_power = Node(
        package="a3_bringup",
        executable="sim_power_sequence_node",
        name="power_sequence_node",
        output="screen",
    )

    gravity = Node(
        package="a3_bringup",
        executable="gravity_torque_node",
        name="gravity_torque_node",
        output="screen",
        parameters=[{"enabled": True}],
    )

    arm_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_share, "launch", "arm_controller.launch.py")
        )
    )

    mqtt_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "bridge.launch.py")
        )
    )

    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripper_share, "launch", "gripper_controller.launch.py")
        ),
        condition=IfCondition(use_gripper),
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
                "use_gripper",
                default_value="true",
                description="是否叠起夹爪力控节点（sim 由 sim_motor_node 提供 L7 接触弹簧力矩，默认开）",
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            rsp,
            sim_motor,
            sim_power,
            gravity,
            arm_controller,
            mqtt_bridge,
            gripper,
            rviz,
        ]
    )
