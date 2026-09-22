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

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except (OSError, IOError):
        return None


def generate_launch_description():
    use_gripper = LaunchConfiguration("use_gripper")
    use_rviz = LaunchConfiguration("use_rviz")
    use_target_ghost = LaunchConfiguration("use_target_ghost")
    use_moveit = LaunchConfiguration("use_moveit")
    use_servo = LaunchConfiguration("use_servo")

    desc_share = get_package_share_directory("a3_description")
    arm_share = get_package_share_directory("a3_arm_controller")
    bridge_share = get_package_share_directory("a3_mqtt_bridge")
    gripper_share = get_package_share_directory("a3_gripper_controller")
    moveit_share = get_package_share_directory("a3_moveit_config")

    urdf = os.path.join(desc_share, "urdf", "el_a3.urdf")
    with open(urdf, "r", encoding="utf-8") as f:
        robot_description = f.read()

    # 目标 ghost 模型：换色成品 URDF（橙→青蓝 / 深棕→深蓝），与主 launch、
    # urdf_dir_check.launch.py 保持一致
    target_description = (
        robot_description.replace(
            'rgba="0.972549 0.529412 0.00392157 1"',  # orange → 青蓝
            'rgba="0.0 0.65 0.85 1"'
        ).replace(
            'rgba="0.301961 0.290196 0.262745 1"',  # dark_brown → 深蓝
            'rgba="0.05 0.12 0.35 1"'
        )
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # 目标 ghost 三件套（与 a3_bringup.launch.py 同款）：target rsp（frame_prefix
    # 必须带尾斜杠，LL-028）+ 恒等静态 TF + home 注入。sim 栈自带的 rviz 是单模型
    # 配置，ghost 不可见；复用/另开 el_a3_dual_view.rviz 时才有渲染。无注入源则
    # target rsp 整树不发 TF，ghost 全白（LL-055/LL-056）。
    rsp_target = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="target_robot_state_publisher",
        output="screen",
        remappings=[
            ("joint_states", "/a3/display_target_joint_states"),
            ("robot_description", "/target_robot_description"),
        ],
        parameters=[{"robot_description": target_description, "frame_prefix": "target/"}],
        condition=IfCondition(use_target_ghost),
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="target_base_link_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "target/base_link"],
        condition=IfCondition(use_target_ghost),
    )
    # publish_traj:=false：只发 ghost 纯话题，不碰执行层轨迹；
    # backoff_on_foreign_msgs:=true：手动 ros2 topic pub 注入时自动让位
    target_pub = Node(
        package="a3_bringup",
        executable="urdf_dir_check_pub",
        name="urdf_dir_check_pub",
        output="screen",
        parameters=[{"publish_traj": False, "backoff_on_foreign_msgs": True}],
        condition=IfCondition(use_target_ghost),
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

    # ---- MoveIt：move_group + FJT action + retime（F67/F68 仿真验收，与 a3_bringup
    #      launch 同款接法；sim /joint_states 为 RELIABLE，move_group 可直接用）----
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
        condition=IfCondition(use_moveit),
    )
    fjt_action = Node(
        package="a3_bringup",
        executable="follow_joint_trajectory_action",
        name="a3_fjt_action",
        parameters=[{"require_gate": False}],
        condition=IfCondition(use_moveit),
    )

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
        condition=IfCondition(use_moveit),
    )

    # ---- MoveIt Servo（默认关；F67 起可与 move_group 同时起，编排层模式互锁）----
    servo_yaml = load_yaml("a3_moveit_config", "config/servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_params.update(servo_yaml)
    servo_mode_bridge = Node(
        package="a3_bringup",
        executable="servo_mode_bridge",
        name="a3_servo_mode_bridge",
        parameters=[{"twist_topic": "/servo_node/delta_twist_cmds"}, {"timeout_s": 0.5}],
        condition=IfCondition(use_servo),
    )
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            {"robot_description_kinematics": kinematics_yaml},
            servo_params,
        ],
        remappings=[
            ("~/delta_twist_cmds", "/servo_node/delta_twist_cmds"),
            ("~/command_out", "/a3/servo/joint_trajectory"),
        ],
        output="screen",
        condition=IfCondition(use_servo),
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
            DeclareLaunchArgument(
                "use_target_ghost",
                default_value="true",
                description="目标 ghost 三件套（target rsp + 恒等静态 TF + home 注入）；"
                "单模型 el_a3_view.rviz 下不可见，开 el_a3_dual_view.rviz 时需要",
            ),
            DeclareLaunchArgument(
                "use_moveit",
                default_value="true",
                description="move_group + FJT action + retime 服务（F67 goto / F68 回放）",
            ),
            DeclareLaunchArgument(
                "use_servo",
                default_value="false",
                description="MoveIt Servo 笛卡尔 jog（可与 use_moveit 共存）",
            ),
            rsp,
            rsp_target,
            static_tf,
            target_pub,
            sim_motor,
            sim_power,
            gravity,
            arm_controller,
            mqtt_bridge,
            gripper,
            move_group,
            fjt_action,
            retime_node,
            servo_mode_bridge,
            servo_node,
            rviz,
        ]
    )
