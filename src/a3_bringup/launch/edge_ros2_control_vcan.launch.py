#!/usr/bin/env python3
"""F72 ros2_control 标准栈 vcan0 仿真闭环（验证真机 SystemInterface 插件）。

与 F70 mock 栈完全相同的控制器/ MoveIt 接线，唯一区别：xacro 生成
use_real_hardware:=true can_interface:=vcan0，controller_manager 加载
a3_hardware_interface/A3MITHardwareInterface（SocketCAN + MIT 协议），
由 scripts/a3_test/vcan_motor_sim.py 充当 7 个电机回 type-2 反馈。

用法（另开终端先起模拟器）：
  python3 scripts/a3_test/vcan_motor_sim.py --interface vcan0
  ROS_DOMAIN_ID=59 ros2 launch a3_bringup edge_ros2_control_vcan.launch.py
真机时把 can_interface 换成 can1（xacro 参数/插件 hardware param）。
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except (OSError, IOError):
        return None


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    adaptive_kd_enabled = LaunchConfiguration("adaptive_kd_enabled")
    motor_can_timeout_enabled = LaunchConfiguration("motor_can_timeout_enabled")

    desc_share = get_package_share_directory("a3_description")
    moveit_share = get_package_share_directory("a3_moveit_config")

    xacro_file = os.path.join(desc_share, "urdf", "el_a3.urdf.xacro")
    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file,
            " use_real_hardware:=true",
            " can_interface:=vcan0",
            " adaptive_kd_enabled:=", adaptive_kd_enabled,
            " motor_can_timeout_enabled:=", motor_can_timeout_enabled,
        ]),
        value_type=str,
    )

    controllers_yaml = os.path.join(desc_share, "config", "el_a3_controllers.yaml")

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # F73: 重力补偿自由拖动控制器以 inactive 状态常驻，
    # 示教时用标准 switch_controllers 与 arm_controller 互斥切换。
    free_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zero_torque_controller",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    with open(os.path.join(moveit_share, "config", "el_a3.srdf"), "r", encoding="utf-8") as f:
        robot_description_semantic = f.read()
    kinematics_yaml = load_yaml("a3_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("a3_moveit_config", "config/joint_limits.yaml")
    ompl_planning_yaml = load_yaml("a3_moveit_config", "config/ompl_planning.yaml")
    moveit_controllers_yaml = load_yaml("a3_moveit_config", "config/moveit_controllers.yaml")
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_planning_scene_hz": 4.0,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            {"robot_description_planning": joint_limits_yaml},
            {"robot_description_kinematics": kinematics_yaml},
            {"move_group": ompl_planning_yaml},
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
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

    # 同 F70 顺序：先 JTC 后 JSB（LL-072）。
    delay_arm = TimerAction(period=3.0, actions=[arm_spawner])
    delay_free_drive = TimerAction(period=5.0, actions=[free_drive_spawner])
    delay_jsb = TimerAction(period=7.0, actions=[jsb_spawner])

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "adaptive_kd_enabled",
            default_value="true",
            description="F85 自由拖动速度自适应 Kd（false=固定 zero_torque_kd 兜底）",
        ),
        DeclareLaunchArgument(
            "motor_can_timeout_enabled",
            default_value="true",
            description="F86 电机侧 CAN 超时 0x7028 布防（false=写 0 撤防）",
        ),
        rsp,
        controller_manager,
        move_group,
        delay_jsb,
        delay_arm,
        delay_free_drive,
        rviz,
    ])
