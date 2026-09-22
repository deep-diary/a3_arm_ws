#!/usr/bin/env python3
"""F75 全产品 mock-hardware 标准栈（无电机 / 无 CAN / 零自研 sim 节点）。

与真机 F72 栈同构的产品级 bringup，拓扑：

  mock_components/GenericSystem（xacro use_mock_hardware:=true）
    └ controller_manager @200 Hz（el_a3_controllers.yaml）
        ├ joint_state_broader（active，标准 /joint_states）
        ├ arm_controller JTC L1–L6（**inactive 启动，enable 才激活**）
        └ gripper_controller JTC L7（**inactive 启动**）
  move_group（moveit_simple_controller_manager 直连 JTC FJT action）
  a3_trajectory_processing/retime_trajectory_node（保几何重定时，arm_with_gripper）
  a3_arm_controller 编排层：
    control_backend=fjt_action（F74 双 JTC action 拆分）
    motor_service_backend=controller_switch（F75：enable/disable 经标准
    /controller_manager/switch_controller，硬件 on_activate/deactivate 内
    reset→enable / stop）
  a3_gripper_controller 产品节点：traj_topic:=/gripper_controller/joint_trajectory
    （JTC 原生话题入口，位置命令直入标准控制器）
  a3_mqtt_bridge（默认包含；broker 不可达时自行重连，节点不退出）

**全程不加载** sim_motor_node / sim_power_sequence_node / gravity_torque_node。
arm_monitor 默认关（其故障处置依赖真机栈 /a3/motor/* 服务；需要时 use_monitor
:=true）。
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    use_mqtt = LaunchConfiguration("use_mqtt")
    use_monitor = LaunchConfiguration("use_monitor")

    desc_share = get_package_share_directory("a3_description")
    moveit_share = get_package_share_directory("a3_moveit_config")
    arm_share = get_package_share_directory("a3_arm_controller")
    gripper_share = get_package_share_directory("a3_gripper_controller")
    bridge_share = get_package_share_directory("a3_mqtt_bridge")

    xacro_file = os.path.join(desc_share, "urdf", "el_a3.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_mock_hardware:=true"]),
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

    # JTC --inactive：只配置不激活（不 claim 命令接口、硬件 on_activate 不触发），
    # 等待编排层 enable → switch_controller 激活。JSB 保活提供 /joint_states。
    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "gripper_controller",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # ---- move_group（直连官方 JTC FJT action）----
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

    # ---- retime（F68 保几何重定时服务，arm_with_gripper 7 关节）----
    jl_map = (joint_limits_yaml or {}).get("joint_limits", {})
    velocity_limits = {
        name: float(d["max_velocity"])
        for name, d in jl_map.items()
        if d.get("has_velocity_limits") and d.get("max_velocity") is not None
    }
    acceleration_limits = {
        name: float(d["max_acceleration"])
        for name, d in jl_map.items()
        if d.get("has_acceleration_limits") and d.get("max_acceleration") is not None
    }
    retime_node = Node(
        package="a3_trajectory_processing",
        executable="retime_trajectory_node",
        name="a3_trajectory_processing",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            {"group_name": "arm_with_gripper"},
            {"velocity_limits": velocity_limits},
            {"acceleration_limits": acceleration_limits},
        ],
    )

    # ---- 编排层：F74 FJT 后端 + F75 controller_switch 使能后端 ----
    fsm = Node(
        package="a3_arm_controller",
        executable="arm_controller",
        name="a3_arm_controller",
        output="screen",
        parameters=[
            os.path.join(arm_share, "config", "arm_controller.yaml"),
            {
                "require_gate": False,
                "control_backend": "fjt_action",
                "motor_service_backend": "controller_switch",
            },
        ],
    )

    monitor = Node(
        package="a3_arm_controller",
        executable="arm_monitor",
        name="arm_monitor",
        output="screen",
        parameters=[os.path.join(arm_share, "config", "arm_monitor.yaml")],
        condition=IfCondition(use_monitor),
    )

    # ---- 夹爪产品节点：轨迹出口改指 JTC 原生话题 ----
    gripper = Node(
        package="a3_gripper_controller",
        executable="gripper_controller",
        name="a3_gripper_controller",
        output="screen",
        parameters=[
            os.path.join(gripper_share, "config", "gripper_config.yaml"),
            {"traj_topic": "/gripper_controller/joint_trajectory"},
        ],
    )

    mqtt_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "bridge.launch.py")
        ),
        condition=IfCondition(use_mqtt),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(desc_share, "config", "el_a3_view.rviz")],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # LL-072：JTC 先于 JSB 激活时 JSB 才能 claim 全接口；JTC 此处仅 configure
    # 不激活，JSB 在 7 s 激活。产品节点 4 s 后再启动，避开 ros2_control_node
    # mock hardware 加载窗口。
    delay_jtc = TimerAction(period=3.0, actions=[jtc_spawner])
    delay_jsb = TimerAction(period=7.0, actions=[jsb_spawner])
    delay_products = TimerAction(
        period=4.0,
        actions=[fsm, monitor, gripper, retime_node, mqtt_bridge],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_mqtt", default_value="true"),
        DeclareLaunchArgument("use_monitor", default_value="false"),
        rsp,
        controller_manager,
        move_group,
        delay_jtc,
        delay_jsb,
        delay_products,
        rviz,
    ])
