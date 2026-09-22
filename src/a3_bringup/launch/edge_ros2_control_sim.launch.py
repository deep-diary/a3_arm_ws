#!/usr/bin/env python3
"""F70 ros2_control 标准栈仿真闭环（无电机 / 无 CAN）。

工业标准执行底座，取代手搓的 a3_fjt_action + motor_protocol_node 插值 + 自研
sim 节点：

  * robot_description 由 xacro use_mock_hardware:=true 生成
    （mock_components/GenericSystem，mock_sensor_commands=false）
  * controller_manager @200 Hz + el_a3_controllers.yaml
  * joint_state_broadcaster：标准 /joint_states
  * arm_controller（joint_trajectory_controller，L1–L6）+
    gripper_controller（JTC，L7）：官方样条插值、容差监控，原生暴露
    /arm_controller/follow_joint_trajectory /gripper_controller/follow_joint_trajectory
  * move_group 经 moveit_simple_controller_manager 直连官方 JTC action
    （命名空间 action_ns 与 JTC 默认一致），全程无 a3_fjt_action

不加载 zero_torque_controller（a3_can_bridge 自研插件，属旧栈；yaml 中仅有
类型声明、不 spawn 即不加载）。与旧仿真栈并存（不同 ROS_DOMAIN_ID 可同时跑）。
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

    desc_share = get_package_share_directory("a3_description")
    moveit_share = get_package_share_directory("a3_moveit_config")

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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(desc_share, "config", "el_a3_view.rviz")],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # spawner 会等待 controller_manager 服务，但 ros2_control_node 加载
    # mock hardware 需要时间，固定延时更稳：先两个 JTC，再 JSB。
    # JSB 早于 JTC 激活时实测只 claim 位置接口（velocity/effort 漏 claim，
    # /joint_states 速度恒 0），调换顺序即正常（LL-072）。
    delay_arm = TimerAction(period=3.0, actions=[arm_spawner])
    delay_jsb = TimerAction(period=7.0, actions=[jsb_spawner])

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        rsp,
        controller_manager,
        move_group,
        delay_jsb,
        delay_arm,
        rviz,
    ])
