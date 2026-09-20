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
    use_target_ghost = LaunchConfiguration("use_target_ghost")
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
        teleop,
        rviz,
    ])
