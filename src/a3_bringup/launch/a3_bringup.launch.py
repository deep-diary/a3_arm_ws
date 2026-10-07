#!/usr/bin/env python3
"""
A3 机械臂唯一产品入口（F78）：hardware:=mock|can，两种模式拓扑完全一致.

拓扑（ros2_control 标准栈，F75–F77 产品形态）：
  robot_state_publisher
  ros2_control controller_manager @200 Hz（el_a3_controllers.yaml）
    ├ joint_state_broadcaster（active，标准 /joint_states）
    ├ arm_controller JTC L1–L6（**inactive 启动，编排层 enable 才激活**）
    ├ gripper_controller effort GAC L7（**inactive 启动**，标准 /gripper_cmd，F87）
    └ zero_torque_controller（inactive 常驻，F73 示教自由拖动时互斥切换）
  move_group（OMPL + Pilz PTP/LIN/CIRC 双规划管线 + Sequence，直连 JTC FJT action）
  a3_trajectory_processing/retime_trajectory_node（保几何重定时，arm L1–L6，排除 L7）
  moveit_servo servo_node（TwistStamped + JointJog 双输入，输出 → arm JTC 原生话题）
    + servo_mode_bridge
  a3_arm_controller 编排层（control_backend=fjt_action，
    motor_service_backend=controller_switch，require_gate 由 yaml 决定）
  a3_mqtt_bridge（默认包含；broker 不可达自行重连，节点不退出）
  diagnostic_aggregator（F82，默认包含；/diagnostics_agg + /diagnostics_toplevel_state）
  rosbag2 黑匣子（F90，默认包含；snapshot-mode 循环缓冲，FAULT 边沿自动落盘 mcap）
  PS4 遥操作（默认包含）

硬件模式：
  hardware:=mock  mock_components/GenericSystem，无需 CAN/电机即可起栈
  hardware:=can   a3_hardware_interface/A3MITHardwareInterface（SocketCAN + MIT），
                  接口由 can_interface 指定（真机 can1；vcan 验收 vcan0 + vcan_motor_sim）

旧 can_bridge C++ 栈入口见 edge_legacy_stack.launch.py（F78 起 deprecated）。
"""

import os
from datetime import datetime

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    IfElseSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def make_spawner_stage(make_node, first_node, on_success, max_retries=2, gap=2.0):
    """
    顺序 spawner 链的一级：退出码 0 → on_success；非零 → 延迟后重启，最多 max_retries.

    重试必须新建同参数 Node：launch_ros 对同一 action 对象二次调度会抛
    "ExecuteLocal ... executed more than once"，异常直接杀死整个 launch
    （一次瞬时 load 竞态 = spawner FATAL + launch 崩溃 + 全栈 SIGINT）。
    逐次重建 spawner 并各自挂退出处理：spawner 幂等（先查 is_controller_loaded，
    已 load 的控制器会跳过 load_controller 继续 configure/activate），
    重试撞上 "already loaded" 不会再把起栈流程打死。
    """
    attempts = {"n": 0}

    def arm(node):
        def on_exit(event, _context):
            if event.returncode == 0:
                return on_success
            attempts["n"] += 1
            if attempts["n"] > max_retries:
                return []
            fresh = make_node()
            return [TimerAction(period=gap, actions=[arm(fresh), fresh])]

        return RegisterEventHandler(
            OnProcessExit(target_action=node, on_exit=on_exit)
        )

    return arm(first_node)


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
    active_report_hz = LaunchConfiguration("active_report_hz")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sw_render = LaunchConfiguration("use_sw_render")
    use_mqtt = LaunchConfiguration("use_mqtt")
    use_teleop = LaunchConfiguration("use_teleop")
    use_self_test = LaunchConfiguration("use_self_test")
    use_host_diagnostics = LaunchConfiguration("use_host_diagnostics")
    teleop_mapping = LaunchConfiguration("teleop_mapping")
    use_monitor = LaunchConfiguration("use_monitor")
    use_diagnostics = LaunchConfiguration("use_diagnostics")
    fsm_backend = LaunchConfiguration("fsm_backend")
    use_rosbag = LaunchConfiguration("use_rosbag")
    bag_dir_cfg = LaunchConfiguration("bag_dir")
    use_servo_anchor = LaunchConfiguration("use_servo_anchor")

    bringup_share = get_package_share_directory("a3_bringup")
    desc_share = get_package_share_directory("a3_description")
    moveit_share = get_package_share_directory("a3_moveit_config")
    arm_share = get_package_share_directory("a3_arm_controller")
    bridge_share = get_package_share_directory("a3_mqtt_bridge")
    teleop_share = get_package_share_directory("a3_teleop_ps4")
    can_bridge_share = get_package_share_directory("a3_can_bridge")

    xacro_file = os.path.join(desc_share, "urdf", "el_a3.urdf.xacro")
    # mock/can 仅切换 xacro 插件参数；其余拓扑共用同一份描述命令。
    hw_args = PythonExpression([
        "' use_mock_hardware:=true' if '", hardware, "' == 'mock'",
        " else ' use_mock_hardware:=false use_real_hardware:=true",
        " can_interface:=", can_interface,
        " adaptive_kd_enabled:=", adaptive_kd_enabled,
        " motor_can_timeout_enabled:=", motor_can_timeout_enabled,
        " active_report_hz:=", active_report_hz, "'",
    ])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, hw_args]),
        value_type=str,
    )

    controllers_yaml = os.path.join(desc_share, "config", "el_a3_controllers.yaml")

    # F89: gravity scales produced by scripts/gravity_scale_calibration.py.
    # Default output is now in the package config dir (same as
    # inertia_params.yaml, version-controlled).  Legacy ~/.a3 fallback for
    # backward compatibility.
    pkg_scales = os.path.join(desc_share, "config", "gravity_scales.yaml")
    legacy_scales = os.path.expanduser("~/.a3/gravity_scales.yaml")
    default_scales = (
        pkg_scales if os.path.exists(pkg_scales) else
        legacy_scales if os.path.exists(legacy_scales) else
        ""
    )

    # F89: JTC 前馈链（gravity_torque_node）同样应用标定的 tau_scale。
    # 标定文件 tau_scale 为 L1-L7 七值（前 6 有效、L7 占位 1.0），直接整体传入。
    gravity_tau_scale = [1.0] * 7
    if default_scales and os.path.exists(default_scales):
        try:
            with open(default_scales, "r", encoding="utf-8") as _f:
                _scales_doc = yaml.safe_load(_f)
            _ts = (_scales_doc.get("zero_torque_controller", {})
                   .get("ros__parameters", {}).get("tau_scale"))
            if isinstance(_ts, list) and _ts:
                gravity_tau_scale = [float(v) for v in _ts]
        except Exception:
            gravity_tau_scale = [1.0] * 7

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # F88: Humble GenericSystem prepare_command_mode_switch 不接受 effort 接口，
    # mock 栈 L7 用 position 版 GAC；真机保持 effort 版（力夹持 + stall 语义）。
    # 用独立 ParameterFile 覆盖（dotted-key dict 会被 launch 写成扁平参数名）。
    gripper_plugin_file = ParameterFile(
        PythonExpression([
            "'", desc_share, "/config/gripper_position_plugin.yaml' if '", hardware,
            "' == 'mock' else '", desc_share, "/config/gripper_effort_plugin.yaml'",
        ]),
        allow_substs=True,
    )

    gravity_scales_path = LaunchConfiguration("gravity_scales_file")
    # Empty arg resolves to the base controllers yaml (identical values),
    # keeping the parameter list static across runs.
    gravity_scales_file_param = ParameterFile(
        PythonExpression([
            "'", gravity_scales_path, "' if '", gravity_scales_path,
            "' != '' else '", controllers_yaml, "'",
        ]),
        allow_substs=True,
    )

    # F160: real-hardware arm_controller claims velocity/acceleration command
    # interfaces (JTC splines v/a feedforward). mock GenericSystem keeps
    # position-only, so fall back to the base controllers yaml (no-op) for mock.
    arm_ff_plugin_file = ParameterFile(
        PythonExpression([
            "'", desc_share, "/config/arm_controller_ff_plugin.yaml' if '", hardware,
            "' == 'can' else '", controllers_yaml, "'",
        ]),
        allow_substs=True,
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
            arm_ff_plugin_file,
            gravity_scales_file_param,
            # LL-103：rcl 参数文件后者覆盖，且 gravity_scales 留空时解析为
            # controllers_yaml 本身——gripper 覆盖文件必须在最后，否则 mock 的
            # position 插件覆写会被基础 yaml 冲掉（F88）。
            gripper_plugin_file,
        ],
        output="screen",
    )

    # JTC --inactive：只配置不激活（不 claim 命令接口、硬件 on_activate 不触发），
    # 等编排层 enable → switch_controller 激活。
    # 三个 spawner 都用工厂函数生成：make_spawner_stage 重试时必须新建同参数
    # Node（launch_ros 禁止同一 action 对象二次调度），否则一次瞬时 load 竞态
    # 会把整个 launch 打死（"ExecuteLocal ... executed more than once"）。
    def make_jtc_spawner():
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "arm_controller",
                "gripper_controller",
                "--inactive",
                "--controller-manager",
                "/controller_manager",
                # 5 s × spawner 内部 3 次重试：单次丢响应可在链内自愈；
                # 30 s 会让恢复（最长 90 s）远超验收/看门狗窗口。
                "--service-call-timeout",
                "5.0",
            ],
            output="screen",
        )

    def make_free_drive_spawner():
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "zero_torque_controller",
                "--inactive",
                "--controller-manager",
                "/controller_manager",
                "--service-call-timeout",
                "5.0",
            ],
            output="screen",
        )

    def make_jsb_spawner():
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
                "--service-call-timeout",
                "5.0",
            ],
            output="screen",
        )

    jtc0 = make_jtc_spawner()
    fd0 = make_free_drive_spawner()
    jsb0 = make_jsb_spawner()

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

    # ---- retime（F68 保几何重定时服务，arm L1–L6；L7 由编排层原始时间戳下发）----
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
            {"group_name": "arm"},
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
            # F118：use_servo_anchor:=true 时 servo 原始输出改道 /cmd，由
            # a3_servo_anchor 锚定后转发回 JTC 原生话题（绝对目标/外力回弹）。
            {
                "moveit_servo.command_out_topic": IfElseSubstitution(
                    use_servo_anchor,
                    "/a3/servo/joint_trajectory/cmd",
                    "/arm_controller/joint_trajectory",
                )
            },
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

    # F9/F49：Pinocchio 重力力矩发布（JTC/位置控制模式下的重力前馈源）。
    # 订阅 /joint_states（URDF frame，joint_direction 全 1），加载 F49 标定的
    # inertia_params.yaml（apply_calibrated_inertia=true），发布 /a3/gravity_torque。
    # motor_protocol_node 在 enable_gravity_compensation=true 时叠加到 MIT tau 前馈。
    gravity_torque_node = Node(
        package="a3_bringup",
        executable="gravity_torque_node",
        name="a3_gravity_torque",
        output="screen",
        parameters=[{
            "enabled": True,
            "apply_calibrated_inertia": True,
            "joint_direction": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            "tau_scale": gravity_tau_scale,
        }],
    )

    # F118：薄桥接累积器（绝对目标）。downstream 在 JTC 原生话题前，非 arm 关节透传。
    servo_anchor_node = Node(
        package="a3_bringup",
        executable="servo_anchor",
        name="a3_servo_anchor",
        output="screen",
        parameters=[
            {
                "in_topic": "/a3/servo/joint_trajectory/cmd",
                "out_topic": "/arm_controller/joint_trajectory",
                "joint_states_topic": "/joint_states",
                "max_joint_delta_rad": 0.05,
            }
        ],
        condition=IfCondition(use_servo_anchor),
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
                # require_gate 由 arm_controller.yaml 决定（真机 true / 仿真 false）；
                # 不再在 launch 里硬编码覆盖。
                "control_backend": fsm_backend,
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

    # ---- F95：标准只读自检（diagnostic_msgs/SelfTest；~ /a3_self_test/self_test）----
    self_test_node = Node(
        package="a3_self_test",
        executable="a3_self_test",
        name="a3_self_test",
        output="screen",
        condition=IfCondition(use_self_test),
    )

    # ---- F101：systemd 服务看门狗喂狗（仅在 NOTIFY_SOCKET 存在时工作）----
    watchdog_feed = Node(
        package="a3_bringup",
        executable="systemd_watchdog_feed",
        name="systemd_watchdog_feed",
        output="screen",
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

    # ---- F96：主机资源标准诊断（diagnostic_common_diagnostics → /diagnostics）----
    host_diag_condition = IfCondition(PythonExpression([
        "'", use_diagnostics, "' == 'true' and '",
        use_host_diagnostics, "' == 'true'",
    ]))
    cpu_monitor = Node(
        package="diagnostic_common_diagnostics",
        executable="cpu_monitor.py",
        name="cpu_monitor",
        output="screen",
        condition=host_diag_condition,
    )
    ram_monitor = Node(
        package="diagnostic_common_diagnostics",
        executable="ram_monitor.py",
        name="ram_monitor",
        output="screen",
        condition=host_diag_condition,
    )
    hd_monitor = Node(
        package="diagnostic_common_diagnostics",
        executable="hd_monitor.py",
        name="hd_monitor",
        output="screen",
        parameters=[{"path": "/"}],
        condition=host_diag_condition,
    )

    # ---- F103：CAN 物理层健康（bus-off/错误帧/链路 down → 标准诊断）----
    can_bus_condition = IfCondition(PythonExpression([
        "'", use_diagnostics, "' == 'true' and '",
        hardware, "' == 'can'",
    ]))
    can_bus_monitor = Node(
        package="a3_bringup",
        executable="can_bus_monitor",
        name="a3_can_bus",
        output="screen",
        parameters=[{"interface": can_interface}],
        condition=can_bus_condition,
    )

    # ---- F104：关键话题频率健康（退化 WARN / 停发 ERROR）----
    topic_rate_monitor = Node(
        package="a3_bringup",
        executable="topic_rate_monitor",
        name="a3_topic_rate",
        output="screen",
        condition=IfCondition(use_diagnostics),
    )

    # ---- F90：故障黑匣子（rosbag2 snapshot-mode；FAULT 边沿 FSM 触发落盘）----
    # 常驻录制器只保留 32 MiB 内存循环缓冲（不落盘、无磁盘增长）；每次 snapshot
    # 把缓冲写为一个 mcap 分片，单分片超 64 MiB 自动切，天然有界。
    blackbox_recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "--snapshot-mode",
            "--max-cache-size", "33554432",
            "--max-bag-size", "67108864",
            "--storage", "mcap",
            "-o", bag_dir_cfg,
            "/joint_states",
            "/a3/arm_status",
            "/a3/control_mode",
            "/diagnostics",
            "/diagnostics_agg",
            "/diagnostics_toplevel_state",
            "/arm_controller/joint_trajectory",
        ],
        output="screen",
        condition=IfCondition(use_rosbag),
    )

    # ---- F110：mock 栈电源序列（对齐真机 C++ power_sequence_node）----
    # 无 CAN 时执行层缺失会导致 gate 永不打开：ps4_mapper L3 5s 超时报错、
    # F61 灯效按 power_state 空 + gate 关判离线而红闪（优先于 READY）。
    # 复用 sim_power_sequence_node：上电锁存 gate_open=true / state=Running，
    # command 往返仍保留。随早期栈启动，确保 teleop 订阅前锁存已发布。
    sim_power_sequence = Node(
        package="a3_bringup",
        executable="sim_power_sequence_node",
        name="power_sequence_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", hardware, "' == 'mock'"])),
    )

    # ---- F111：真机栈电源序列（C++ power_sequence_node）----
    # 统一栈硬件插件直驱 SocketCAN，旧执行层未并入：can 模式下 gate 话题同样
    # 0 publisher，L3 必超时。启动 C++ 节点补 gate/state 时序；其 /can_tx_frames
    # 使能帧在统一栈无桥接节点自动丢弃，硬件使能由插件 controller switch 完成。
    can_power_sequence = Node(
        package="a3_can_bridge",
        executable="power_sequence_node",
        name="power_sequence_node",
        output="screen",
        parameters=[
            os.path.join(can_bridge_share, "config", "power_sequence.yaml"),
            os.path.join(can_bridge_share, "config", "motor_map.yaml"),
        ],
        condition=IfCondition(PythonExpression(["'", hardware, "' == 'can'"])),
    )

    # JTC（3 s）→ zero_torque → JSB 严格顺序执行，任一 spawner 非零退出自动
    # 重启（最多 2 次）。顺序执行消除并发 load 突发，重启兜住 rmw 丢响应竞态。
    # 产品节点必须等 JSB 成功后再启动：8 节点并发突发曾把 list_controllers
    # 响应挤丢（rmw_response.cpp timeout），导致 /joint_states 永远不出现。
    delay_jtc = TimerAction(period=3.0, actions=[jtc0])
    stage_jtc = make_spawner_stage(
        make_jtc_spawner, jtc0,
        [TimerAction(period=1.0, actions=[fd0])],
    )
    stage_free_drive = make_spawner_stage(
        make_free_drive_spawner, fd0,
        [TimerAction(period=1.0, actions=[jsb0])],
    )
    stage_jsb = make_spawner_stage(
        make_jsb_spawner, jsb0,
        [TimerAction(period=1.0, actions=[
            fsm,
            monitor,
            retime_node,
            watchdog_feed,
            mqtt_bridge,
            servo_node,
            servo_bridge,
            gravity_torque_node,
            servo_anchor_node,
            self_test_node,
            teleop,
            can_bus_monitor,
            topic_rate_monitor,
        ])],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware",
            default_value="mock",
            choices=["mock", "can"],
            description="硬件插件：mock=GenericSystem 无 CAN；"
                        "can=A3MITHardwareInterface（配 can_interface）",
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
        DeclareLaunchArgument(
            "active_report_hz",
            default_value="10.0",
            description="F123 0x18 主动上报周期（Hz；10=100ms/电机；100=回退工厂 10ms 诊断观察位）",
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
            "use_self_test",
            default_value="true",
            description="F95 只读自检服务（diagnostic_msgs/SelfTest；"
                        "ros2 service call /a3_self_test/self_test）",
        ),
        DeclareLaunchArgument(
            "use_host_diagnostics",
            default_value="true",
            description="F96 主机资源诊断（diagnostic_common_diagnostics："
                        "CPU/RAM/磁盘；受 use_diagnostics 总门控）",
        ),
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
        DeclareLaunchArgument(
            "gravity_scales_file",
            default_value=default_scales,
            description="F89 重力比例 CM ParameterFile（zero_torque_controller.tau_scale）；"
                        "留空且 ~/.a3/gravity_scales.yaml 存在时自动采用",
        ),
        DeclareLaunchArgument(
            "fsm_backend",
            default_value="fjt_action",
            choices=["fjt_action", "topic"],
            description="编排层轨迹后端：fjt_action=JTC 标准 action（产品栈默认）；"
                        "topic=旧 motor_protocol 话题（legacy）",
        ),
        DeclareLaunchArgument(
            "use_rosbag",
            default_value="true",
            description="F90 故障黑匣子（rosbag2 snapshot-mode；FAULT 边沿自动落盘 mcap）",
        ),
        DeclareLaunchArgument(
            "bag_dir",
            default_value=os.path.expanduser(
                "~/.a3/blackbox/blackbox_" + datetime.now().strftime("%Y%m%d_%H%M%S")
            ),
            description="F90 黑匣子输出目录（默认每次启动生成带时间戳的新目录）",
        ),
        DeclareLaunchArgument(
            "use_servo_anchor",
            default_value="false",
            description="F118: 在 servo 下游叠 a3_servo_anchor 累积器（绝对目标/外力回弹）；"
                        "true 时 servo command_out 改道 /a3/servo/joint_trajectory/cmd 经锚定转发回 JTC",
        ),
        rsp,
        sim_power_sequence,
        can_power_sequence,
        diagnostic_aggregator,
        cpu_monitor,
        ram_monitor,
        hd_monitor,
        blackbox_recorder,
        controller_manager,
        move_group,
        delay_jtc,
        stage_jtc,
        stage_free_drive,
        stage_jsb,
        rviz,
    ])
