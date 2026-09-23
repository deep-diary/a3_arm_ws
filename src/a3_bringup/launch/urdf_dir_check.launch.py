"""
URDF 方向校验（真机，臂不失能）：RViz 双模型 + 目标摆动发布器.

前提：can_bridge（motor_protocol_node）已运行（refresh 播种保持反馈流），
臂处于失能状态（电机 mode=0，轨迹帧只收不执行）。

- ArmActual_实际反馈：robot_state_publisher 由 /joint_states（电机反馈）驱动
- ArmTarget_目标：target_robot_state_publisher（frame_prefix=target + 恒等静态变换
  base_link→target/base_link 接树）由 /a3/display_target_joint_states 驱动
- RViz 窗口由 wmctrl 最大化（无 wmctrl 静默跳过），需 DISPLAY（HDMI 用 :0）
- use_sw_render 默认开：LIBGL_ALWAYS_SOFTWARE=1 走 llvmpipe 软件渲染。
  RK3588 上 RViz 用 panfrost GL 曾整机硬卡死（LL-027），软件渲染规避
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_traj = LaunchConfiguration("use_traj")
    use_sw_render = LaunchConfiguration("use_sw_render")

    desc_share = get_package_share_directory("a3_description")
    bringup_share = get_package_share_directory("a3_bringup")

    with open(os.path.join(desc_share, "urdf", "el_a3.urdf"), "r", encoding="utf-8") as f:
        robot_description = f.read()

    # 目标模型换色系（青蓝），与实际模型（橙/深棕）高对比区分。
    # el_a3.urdf 只有两个命名材质，按定义处 rgba 原值整串替换即可。
    target_description = (
        robot_description.replace(
            'rgba="0.972549 0.529412 0.00392157 1"',  # orange → 青蓝
            'rgba="0.0 0.65 0.85 1"'
        ).replace(
            'rgba="0.301961 0.290196 0.262745 1"',  # dark_brown → 深蓝
            'rgba="0.05 0.12 0.35 1"'
        )
    )

    rsp_actual = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    # rsp 的 prefix 参数名是 frame_prefix（tf_prefix 会被静默忽略 → 两个 rsp 同名帧
    # 互踩，实际模型在两套变换间跳动，目标模型 No transform；LL-028）
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
    )
    # rsp 不发布根帧 target/base_link，用恒等静态变换把目标树接到 base_link 上，
    # RViz（Fixed Frame base_link + TF Prefix target）才能算到整条目标链
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="target_base_link_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "target/base_link"],
    )
    pub = Node(
        package="a3_bringup",
        executable="urdf_dir_check_pub",
        name="urdf_dir_check_pub",
        output="screen",
        condition=IfCondition(use_traj),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(bringup_share, "config", "urdf_dir_check.rviz")],
        output="screen",
        # 软件渲染规避 RK3588 panfrost GPU hang（LL-027）
        prefix=IfElseSubstitution(use_sw_render, "env LIBGL_ALWAYS_SOFTWARE=1", ""),
        condition=IfCondition(use_rviz),
    )

    # .rviz Window Geometry 只能写死宽高；真正最大化靠 wmctrl（无则静默跳过）
    maximize_rviz = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    (
                        "command -v wmctrl >/dev/null 2>&1 || exit 0; "
                        "for n in rviz2 RViz rviz; do "
                        'wmctrl -r "$n" -b add,maximized_vert,maximized_horz && exit 0; '
                        "done; "
                        "wid=$(wmctrl -l | awk 'tolower($0) ~ /rviz/ {print $1; exit}'); "
                        '[ -n "$wid" ] && wmctrl -i -r "$wid" '
                        "-b add,maximized_vert,maximized_horz || true"
                    ),
                ],
                output="log",
            )
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否打开 RViz 双模型（默认开，需 DISPLAY，HDMI 用 :0）",
            ),
            DeclareLaunchArgument(
                "use_traj",
                default_value="true",
                description="是否启动目标摆动发布器（默认开）",
            ),
            DeclareLaunchArgument(
                "use_sw_render",
                default_value="true",
                description="RViz 软件渲染 LIBGL_ALWAYS_SOFTWARE=1（默认开，LL-027）",
            ),
            rsp_actual,
            rsp_target,
            static_tf,
            pub,
            rviz,
            maximize_rviz,
        ]
    )
