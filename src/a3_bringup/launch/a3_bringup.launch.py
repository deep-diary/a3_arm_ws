#!/usr/bin/env python3
"""A3 机械臂唯一产品入口（F78）：hardware:=mock|can，两种模式拓扑完全一致。

拓扑（ros2_control 标准栈，F75–F77 产品形态）：
  robot_state_publisher
  ros2_control controller_manager @200 Hz（el_a3_controllers.yaml）
    ├ joint_state_broadcaster（active，标准 /joint_states）
    ├ arm_controller JTC L1–L6（**inactive 启动，编排层 enable 才激活**）
    ├ gripper_controller JTC L7（**inactive 启动**）
    └ zero_torque_controller（inactive 常驻，F73 示教自由拖动时互斥切换）
  move_group（OMPL + Pilz PTP/LIN/CIRC 双规划管线 + Sequence，直连 JTC FJT action）
  a3_trajectory_processing/retime_trajectory_node（保几何重定时，arm_with_gripper）
  moveit_servo servo_node（TwistStamped + JointJog 双输入，输出 → arm JTC 原生话题）
    + servo_mode_bridge
  a3_arm_controller 编排层（control_backend=fjt_action，
    motor_service_backend=controller_switch，require_gate:=false）
  a3_gripper_controller 产品节点（traj_topic=/gripper_controller/joint_trajectory）
  a3_mqtt_bridge（默认包含；broker 不可达自行重连，节点不退出）
  diagnostic_aggregator（F82，默认包含；/diagnostics_agg + /diagnostics_toplevel_state）
  PS4 遥操作（默认包含）

硬件模式：
  hardware:=mock  mock_components/GenericSystem，无需 CAN/电机即可起栈
  hardware:=can   a3_hardware_interface/A3MITHardwareInterface（SocketCAN + MIT），
                  接口由 can_interface 指定（真机 can1；vcan 验收 vcan0 + vcan_motor_sim）

旧 can_bridge C++ 栈入口见 edge_legacy_stack.launch.py（F78 起 deprecated）。
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
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
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
    hardware = LaunchConfiguration("hardware")
    can_interface = LaunchConfiguration("can_interface")
    adaptive_kd_enabled = LaunchConfiguration("adaptive_kd_enabled")
    motor_can_timeout_enabled = LaunchConfiguration("motor_can_timeout_enabled")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sw_render = LaunchConfiguration("use_sw_render")
    use_mqtt = LaunchConfiguration("use_mqtt")
    use_teleop = LaunchConfiguration("use_teleop")
    teleop_mapping = LaunchConfiguration("teleop_mapping")
    use_monitor = LaunchConfiguration("use_monitor")
    use_diagnostics = LaunchConfiguration("use_diagnostics")

    bringup_share = get_package_share_directory("a3_bringup")
    desc_share = get_package_share_directory("a3_description")
    moveit_share = get_package_share_directory("a3_moveit_config")
    arm_share = get_package_share_directory("a3_arm_controller")
    gripper_share = get_package_share_directory("a3_gripper_controller")
    bridge_share = get_package_share_directory("a3_mqtt_bridge")
    teleop_share = get_package_share_directory("a3_teleop_ps4")

    xacro_file = os.path.join(desc_share, "urdf", "el_a3.urdf.xacro")
    # mock/can 仅切换 xacro 插件参数；其余拓扑共用同一份描述命令。
    hw_args = PythonExpression([
        "' use_mock_hardware:=true' if '", hardware, "' == 'mock'",
        " else ' use_mock_hardware:=false use_real_hardware:=true",
        " can_interface:=", can_interface,
        " adaptive_kd_enabled:=", adaptive_kd_enabled,
        " motor_can_timeout_enabled:=", motor_can_timeout_enabled, "'",
    ])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, hw_args]),
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
    # 等编排层 enable → switch_controller 激活。
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

    # ---- move_group（OMPL + Pilz 双管线；直连官方 JTC FJT action）----
    with open(os.path.join(moveit_share, "config", "el_a3.srdf"), "r", encoding="utf-8") as f:
        robot_description_semantic = f.read()
    kinematics_yaml = load_yaml("a3_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("a3_moveit_config", "config/joint_limits.yaml")
    ompl_planning_yaml = load_yaml("a3_moveit_config", "config/ompl_planning.yaml")
    pilz_planning_yaml = load_yaml(
        "a3_moveit_config", "config/pilz_industrial_motion_planner.yaml"
    )
    pilz_cartesian_limits_yaml = load_yaml(
        "a3_moveit_config", "config/pilz_cartesian_limits.yaml"
    )
    moveit_controllers_yaml = load_yaml("a3_moveit_config", "config/moveit_controllers.yaml")
    servo_yaml = load_yaml("a3_moveit_config", "config/servo_config.yaml")
    # Flatten: moveit_servo accepts params both top-level and under moveit_servo.
    servo_params = {"moveit_servo": servo_yaml}
    servo_params.update(servo_yaml or {})

    planning_pipelines_parameters = {
        "planning_pipelines": ["ompl", "pilz"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning_yaml,
        "pilz": pilz_planning_yaml,
    }
    robot_description_planning = dict(joint_limits_yaml or {})
    if pilz_cartesian_limits_yaml:
        robot_description_planning.update(pilz_cartesian_limits_yaml)
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
            {"robot_description_planning": robot_description_planning},
            {"robot_description_kinematics": kinematics_yaml},
            planning_pipelines_parameters,
            {
                "capabilities": (
                    "pilz_industrial_motion_planner/MoveGroupSequenceAction "
                    "pilz_industrial_motion_planner/MoveGroupSequenceService"
                )
            },
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

    # ---- servo_node 常驻（TwistStamped + JointJog 双输入），出口指 arm JTC 话题 ----
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            {"robot_description_kinematics": kinematics_yaml},
            servo_params,
            {"moveit_servo.command_out_topic": "/arm_controller/joint_trajectory"},
        ],
        remappings=[
            ("~/delta_twist_cmds", "/servo_node/delta_twist_cmds"),
            ("~/delta_joint_cmds", "/servo_node/delta_joint_cmds"),
        ],
    )

    servo_bridge = Node(
        package="a3_bringup",
        executable="servo_mode_bridge",
        name="a3_servo_mode_bridge",
        output="screen",
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

    # ---- 夹爪产品节点：轨迹出口指 JTC 原生话题 ----
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

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_share, "launch", "ps4_teleop.launch.py")
        ),
        launch_arguments={
            "auto_start_servo": "false",
            "mapping": teleop_mapping,
        }.items(),
        condition=IfCondition(use_teleop),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(desc_share, "config", "el_a3_view.rviz")],
        # 软件渲染规避 RK3588 panfrost GPU hang（LL-027）
        prefix=PythonExpression([
            "'env LIBGL_ALWAYS_SOFTWARE=1' if '", use_sw_render,
            "' in ('true', '1', 'True') else ''",
        ]),
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # ---- F82：诊断聚合（/diagnostics → /diagnostics_agg + toplevel state）----
    diagnostics_yaml = os.path.join(bringup_share, "config", "diagnostics.yaml")
    diagnostic_aggregator = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostic_aggregator",
        output="screen",
        parameters=[diagnostics_yaml],
        condition=IfCondition(use_diagnostics),
    )

    # LL-072：JTC 先 configure（3 s），zero_torque 5 s，JSB 7 s 激活；
    # 产品节点 4 s 后启动，避开 hardware 加载窗口。
    delay_jtc = TimerAction(period=3.0, actions=[jtc_spawner])
    delay_free_drive = TimerAction(period=5.0, actions=[free_drive_spawner])
    delay_jsb = TimerAction(period=7.0, actions=[jsb_spawner])
    delay_products = TimerAction(
        period=4.0,
        actions=[
            fsm,
            monitor,
            gripper,
            retime_node,
            mqtt_bridge,
            servo_node,
            servo_bridge,
            teleop,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware",
            default_value="mock",
            choices=["mock", "can"],
            description="硬件插件：mock=GenericSystem 无 CAN；can=A3MITHardwareInterface（配 can_interface）",
        ),
        DeclareLaunchArgument(
            "can_interface",
            default_value="can1",
            description="hardware:=can 时的 SocketCAN 接口名（真机 can1；vcan 验收 vcan0）",
        ),
        DeclareLaunchArgument(
            "adaptive_kd_enabled",
            default_value="true",
            description="F85 自由拖动速度自适应 Kd（false=固定 zero_torque_kd=0.3 兜底）",
        ),
        DeclareLaunchArgument(
            "motor_can_timeout_enabled",
            default_value="true",
            description="F86 电机侧 CAN 超时 0x7028 布防（false=显式写 0 撤防；与 F81 主机看门狗独立）",
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "use_sw_render",
            default_value="true",
            description="RViz 软件渲染 LIBGL_ALWAYS_SOFTWARE=1（RK3588 默认开，LL-027）",
        ),
        DeclareLaunchArgument(
            "use_mqtt",
            default_value="true",
            description="MQTT 桥（web 遥测/指令，F18/F23）",
        ),
        DeclareLaunchArgument("use_teleop", default_value="true"),
        DeclareLaunchArgument(
            "teleop_mapping",
            default_value="default",
            description="手柄映射：default（F60/F64）或 simple（legacy）",
        ),
        DeclareLaunchArgument(
            "use_monitor",
            default_value="false",
            description="arm_monitor 诊断节点（F71；故障处置依赖具体后端时再开）",
        ),
        DeclareLaunchArgument(
            "use_diagnostics",
            default_value="true",
            description="diagnostic_aggregator 诊断聚合（F82；/diagnostics_agg + toplevel state）",
        ),
        rsp,
        diagnostic_aggregator,
        controller_manager,
        move_group,
        delay_jtc,
        delay_free_drive,
        delay_jsb,
        delay_products,
        rviz,
    ])
