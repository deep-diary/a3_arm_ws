#!/usr/bin/env python3
"""Unified A3 arm bringup: 全栈单入口，各组件用参数开关控制。

执行底座（常开）:
  robot_state_publisher + can_bridge（can_transport + motor_protocol + power_sequence）
  + trajectory_bridge（reBot 话题桥接）

可选（默认开）:
  a3_arm_controller（编排层 F21 + F50 看门狗） use_arm_controller:=true
  a3_mqtt_bridge（web 遥测/指令 F18/F23）      use_mqtt:=true
  MoveIt move_group + FJT action（规划 + Execute）use_moveit:=true
  a3_gripper_controller（夹爪力控 F24-F28）    use_gripper:=true
  PS4 遥操作                                   use_teleop:=true

可选（默认关）:
  gravity_torque_node（重力 MIT 前馈）         use_gravity_compensation:=false
  MoveIt Servo 笛卡尔 jog（与 move_group Execute 互斥）use_servo:=false
  RViz                                         use_rviz:=false

注意：MoveIt 不能 include a3_moveit_config/demo.launch.py（自带 RSP + ros2_control +
spawner，会与 can_bridge 形成双 /joint_states / 双 RSP 冲突）；这里只起 move_group，
执行路径由 follow_joint_trajectory_action 的 /arm_controller/follow_joint_trajectory
action 落到 /joint_group_effort_controller/joint_trajectory → motor_protocol_node。
真机 /joint_states 为 BEST_EFFORT（SensorDataQoS），move_group 默认 RELIABLE 订阅
可能收不到（LL-030 同款坑），真机联调时需验证。
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, IfElseSubstitution, LaunchConfiguration
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
    desc_share = get_package_share_directory("a3_description")
    bridge_share = get_package_share_directory("a3_can_bridge")
    moveit_share = get_package_share_directory("a3_moveit_config")
    arm_ctrl_share = get_package_share_directory("a3_arm_controller")
    mqtt_share = get_package_share_directory("a3_mqtt_bridge")
    gripper_share = get_package_share_directory("a3_gripper_controller")

    use_rviz = LaunchConfiguration("use_rviz")
    use_sw_render = LaunchConfiguration("use_sw_render")
    use_target_ghost = LaunchConfiguration("use_target_ghost")
    use_teleop = LaunchConfiguration("use_teleop")
    teleop_mapping = LaunchConfiguration("teleop_mapping")
    use_power_sequence = LaunchConfiguration("use_power_sequence")
    use_gravity_compensation = LaunchConfiguration("use_gravity_compensation")
    use_arm_controller = LaunchConfiguration("use_arm_controller")
    arm_controller_config = LaunchConfiguration("arm_controller_config")
    use_mqtt = LaunchConfiguration("use_mqtt")
    use_moveit = LaunchConfiguration("use_moveit")
    use_gripper = LaunchConfiguration("use_gripper")
    use_servo = LaunchConfiguration("use_servo")
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

    # 目标 ghost 模型：读成品 el_a3.urdf 并换色（橙→青蓝 / 深棕→深蓝），
    # 与 urdf_dir_check.launch.py 的颜色替换保持一致
    with open(os.path.join(desc_share, "urdf", "el_a3.urdf"), "r", encoding="utf-8") as f:
        base_urdf = f.read()
    target_description = (
        base_urdf.replace(
            'rgba="0.972549 0.529412 0.00392157 1"',  # orange → 青蓝
            'rgba="0.0 0.65 0.85 1"'
        ).replace(
            'rgba="0.301961 0.290196 0.262745 1"',  # dark_brown → 深蓝
            'rgba="0.05 0.12 0.35 1"'
        )
    )

    # ---- 执行底座 ----
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    # 目标 ghost：独立 rsp（frame_prefix 必须带尾斜杠，LL-028）走独立话题，
    # 由 /a3/display_target_joint_states 注入驱动（默认 home 注入见 target_pub，
    # 手动 topic pub 注入自动让位）——臂断电后仍可离线调试目标
    rsp_target = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="target_robot_state_publisher",
        output="screen",
        remappings=[
            ("joint_states", "/a3/display_target_joint_states"),
            # rsp 会把 robot_description 当 transient_local 话题发布；两个 rsp
            # 同名话题互踩会互相覆盖，目标模型必须走独立话题（换色 URDF）
            ("robot_description", "/target_robot_description"),
        ],
        parameters=[{"robot_description": target_description, "frame_prefix": "target/"}],
        condition=IfCondition(use_target_ghost),
    )
    # rsp 不发布根帧 target/base_link，用恒等静态变换把目标树接到 base_link 上
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="target_base_link_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "target/base_link"],
        condition=IfCondition(use_target_ghost),
    )
    # ghost 默认注入（home 静止）：publish_traj:=false 只发 /a3/display_target_
    # joint_states、不向执行层下发轨迹（区别于 urdf_dir_check.launch.py 的
    # 方向校验用法）；无注入源时 target rsp 整树不发 TF，ghost 全白（LL-055/
    # LL-056）
    target_pub = Node(
        package="a3_bringup",
        executable="urdf_dir_check_pub",
        name="urdf_dir_check_pub",
        output="screen",
        parameters=[{"publish_traj": False, "backoff_on_foreign_msgs": True}],
        condition=IfCondition(use_target_ghost),
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

    # ---- 编排层（F21 对外门面 + F50 看门狗）----
    arm_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_ctrl_share, "launch", "arm_controller.launch.py")
        ),
        launch_arguments={
            "config_file": arm_controller_config,
        }.items(),
        condition=IfCondition(use_arm_controller),
    )

    # ---- MQTT 桥（web 遥测上行 F18 / 指令下行 F23）----
    mqtt_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mqtt_share, "launch", "bridge.launch.py")
        ),
        condition=IfCondition(use_mqtt),
    )

    # ---- MoveIt：move_group + FJT action（默认关 RViz，见 use_rviz）----
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

    # ---- 夹爪力控（5J 档缺 L7 时置 use_gripper:=false）----
    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripper_share, "launch", "gripper_controller.launch.py")
        ),
        condition=IfCondition(use_gripper),
    )

    # ---- MoveIt Servo 笛卡尔 jog（默认关；只起 servo 链，复用上面 RSP/描述，避免
    #      include servo.launch.py 带来的双 RSP + sim_executor 冲突）----
    servo_yaml = load_yaml("a3_moveit_config", "config/servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_params.update(servo_yaml)

    servo_mode_bridge = Node(
        package="a3_bringup",
        executable="servo_mode_bridge",
        name="a3_servo_mode_bridge",
        parameters=[
            {"twist_topic": "/servo_node/delta_twist_cmds"},
            {"timeout_s": 0.5},
        ],
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
            ("~/command_out", "/joint_group_effort_controller/joint_trajectory"),
        ],
        output="screen",
        condition=IfCondition(use_servo),
    )

    # ---- PS4 遥操作 ----
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("a3_teleop_ps4"),
                "launch",
                "ps4_teleop.launch.py",
            )
        ),
        launch_arguments={
            "auto_start_servo": "false",
            "mapping": teleop_mapping,
        }.items(),
        condition=IfCondition(use_teleop),
    )

    # ---- RViz ----
    rviz_cfg = os.path.join(desc_share, "config", "el_a3_dual_view.rviz")
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
        DeclareLaunchArgument(
            "teleop_mapping",
            default_value="default",
            description="手柄映射：default（F60 默认，L1 摇杆死人开关）或 simple（legacy）",
        ),
        DeclareLaunchArgument("use_power_sequence", default_value="true"),
        DeclareLaunchArgument(
            "use_gravity_compensation",
            default_value="false",
            description="启动 gravity_torque_node 并打开 motor_protocol 重力 MIT 前馈",
        ),
        DeclareLaunchArgument(
            "use_arm_controller",
            default_value="true",
            description="编排层 a3_arm_controller + F50 看门狗（a3_arm_monitor）",
        ),
        DeclareLaunchArgument(
            "arm_controller_config",
            default_value=os.path.join(arm_ctrl_share, "config", "arm_controller.yaml"),
            description="编排层参数文件（5J 档换 arm_controller_5j.yaml）",
        ),
        DeclareLaunchArgument(
            "use_mqtt",
            default_value="true",
            description="MQTT 桥 a3_mqtt_bridge（web 遥测/指令，F18/F23）",
        ),
        DeclareLaunchArgument(
            "use_moveit",
            default_value="true",
            description="MoveIt move_group + FJT action（规划 + Execute 到执行层）",
        ),
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            description="夹爪力控 a3_gripper_controller（5J 档缺 L7 时置 false）",
        ),
        DeclareLaunchArgument(
            "use_servo",
            default_value="false",
            description="MoveIt Servo 笛卡尔 jog（与 move_group Execute 互斥，需时显式开）",
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
        DeclareLaunchArgument(
            "use_target_ghost",
            default_value="true",
            description="目标 ghost 三件套（target rsp + 恒等静态 TF + home 注入）；"
            "关掉需配套不用双模型 RViz 配置",
        ),
        rsp,
        rsp_target,
        static_tf,
        target_pub,
        can_bridge,
        traj_bridge,
        gravity,
        arm_controller,
        mqtt_bridge,
        move_group,
        fjt_action,
        gripper,
        servo_mode_bridge,
        servo_node,
        teleop,
        rviz,
    ])
