#!/usr/bin/env python3
"""全仿真 PS4 遥操作闭环（F62）：edge_web_sim + 内联 Servo + teleop + 双模型 RViz。

用于无手柄合成 /joy 全功能验证（scripts/a3_test/ps4_sim_test.py）与用户看 RViz
旁听：
  * 底座复用 edge_web_sim（sim_motor + sim_power + gravity + arm_controller +
    monitor + gripper + ghost 三件套），关掉它自带的单模型 RViz；
  * Servo 内联块照抄 a3_bringup.launch.py（不可 include servo.launch.py / demo：
    双 RSP / 双 /joint_states 冲突）；
  * ps4_teleop use_joy_node:=false——/joy 由测试脚本（或 ds4_tcp_joy_node）注入；
  * 本入口自有 el_a3_dual_view.rviz（软件渲染，LL-027）。

真机勿用；CAN/电机关联见 a3_bringup.launch.py。
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute = os.path.join(package_path, file_path)
    try:
        with open(absolute, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except (OSError, IOError):
        return None


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_sw_render = LaunchConfiguration("use_sw_render")
    use_gripper = LaunchConfiguration("use_gripper")
    mapping = LaunchConfiguration("mapping")
    use_joy_node = LaunchConfiguration("use_joy_node")

    desc_share = get_package_share_directory("a3_description")
    moveit_share = get_package_share_directory("a3_moveit_config")

    with open(os.path.join(desc_share, "urdf", "el_a3.urdf"), "r", encoding="utf-8") as f:
        robot_description = f.read()
    with open(os.path.join(moveit_share, "config", "el_a3.srdf"), "r", encoding="utf-8") as f:
        robot_description_semantic = f.read()
    kinematics_yaml = load_yaml("a3_moveit_config", "config/kinematics.yaml")

    # 注意：IncludeLaunchDescription 不隔离 LaunchConfiguration——子 launch 的
    # use_rviz:=false 会泄漏到本层把外层 RViz 条件顶成 false，必须用 scoped
    # GroupAction 包起来（LL-062）。
    web_sim = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("a3_bringup"),
                    "launch",
                    "edge_web_sim.launch.py",
                )
            ),
            launch_arguments={
                "use_rviz": "false",
                "use_target_ghost": "true",
                "use_gripper": use_gripper,
            }.items(),
        )
    ])

    # ---- Servo 内联块（与 a3_bringup.launch.py 保持一致）----
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
    )

    teleop = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("a3_teleop_ps4"),
                    "launch",
                    "ps4_teleop.launch.py",
                )
            ),
            launch_arguments={
                "use_joy_node": use_joy_node,
                "mapping": mapping,
                "auto_start_servo": "true",
                "enable_feedback": "true",
            }.items(),
        )
    ])

    rviz_cfg = os.path.join(desc_share, "config", "el_a3_dual_view.rviz")
    # 隐藏 RANDR：本机 X RANDR 报告尺寸但零刷新率 → Ogre 视频模式表为空 →
    # possibleValues[0] 空指针解引用，rviz2 启动即崩（LL-065）。
    hide_randr_so = os.path.expanduser("~/.a3/hide_randr/libhide_randr.so")
    preload = f"LD_PRELOAD={hide_randr_so} " if os.path.exists(hide_randr_so) else ""
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        prefix=IfElseSubstitution(
            use_sw_render,
            f"env {preload}LIBGL_ALWAYS_SOFTWARE=1",
            f"env {preload}".rstrip(),
        ),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "use_sw_render",
            default_value="true",
            description="RViz 软件渲染 LIBGL_ALWAYS_SOFTWARE=1（RK3588 默认开，LL-027）",
        ),
        DeclareLaunchArgument("use_gripper", default_value="true"),
        DeclareLaunchArgument(
            "use_joy_node",
            default_value="false",
            description="true=起 joy_node 接真手柄；false（合成测试默认）=/joy 由脚本/TCP 注入",
        ),
        DeclareLaunchArgument(
            "mapping",
            default_value="default",
            description="PS4 映射（config/mappings/<name>.yaml）",
        ),
        web_sim,
        servo_mode_bridge,
        servo_node,
        teleop,
        rviz,
    ])
