#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _launch_nodes(context, *args, **kwargs):
    share = get_package_share_directory("a3_teleop_ps4")
    mapping = LaunchConfiguration("mapping").perform(context)
    mapping_file = os.path.join(share, "config", "mappings", f"{mapping}.yaml")
    layout = LaunchConfiguration("layout").perform(context)
    layout_file = os.path.join(share, "config", f"{layout}.yaml")
    registry_file = os.path.join(share, "config", "action_registry.yaml")

    auto_start = LaunchConfiguration("auto_start_servo").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )

    return [
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            condition=IfCondition(LaunchConfiguration("use_joy_node")),
            parameters=[{
                "device_name": LaunchConfiguration("device_name").perform(context),
                "deadzone": float(LaunchConfiguration("deadzone").perform(context)),
                "autorepeat_rate": 30.0,
            }],
            output="screen",
        ),
        Node(
            package="a3_teleop_ps4",
            executable="ps4_mapper",
            name="ps4_mapper",
            parameters=[{
                "layout_file": layout_file,
                "registry_file": registry_file,
                "mapping_file": mapping_file,
                "auto_start_servo": auto_start,
            }],
            output="screen",
        ),
        Node(
            package="a3_teleop_ps4",
            executable="joy_dump",
            name="joy_dump",
            condition=IfCondition(LaunchConfiguration("dump")),
            output="screen",
        ),
        Node(
            package="a3_teleop_ps4",
            executable="ds4_hid_node",
            name="ds4_hid_node",
            condition=IfCondition(LaunchConfiguration("enable_hid")),
            output="screen",
        ),
        Node(
            package="a3_teleop_ps4",
            executable="ds4_feedback_node",
            name="ds4_feedback_node",
            condition=IfCondition(LaunchConfiguration("enable_feedback")),
            output="screen",
        ),
        Node(
            package="a3_teleop_ps4",
            executable="ps4_arm_teleop",
            name="ps4_arm_teleop",
            condition=IfCondition(LaunchConfiguration("use_legacy_jog")),
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_joy_node",
            default_value="true",
            description="起内核 joy_node（合成 /joy 测试传 false）",
        ),
        DeclareLaunchArgument("device_name", default_value=""),
        DeclareLaunchArgument("deadzone", default_value="0.12"),
        DeclareLaunchArgument("dump", default_value="false"),
        DeclareLaunchArgument("enable_hid", default_value="false"),
        DeclareLaunchArgument(
            "enable_feedback",
            default_value="true",
            description="DS4 灯带/震动反馈节点（F61；无手柄时降级为 /a3/ds4/feedback）",
        ),
        DeclareLaunchArgument("use_legacy_jog", default_value="false"),
        DeclareLaunchArgument("auto_start_servo", default_value="false"),
        DeclareLaunchArgument(
            "mapping",
            default_value="default",
            description="config/mappings/<name>.yaml；default=F60 L1 摇杆死人开关，simple=legacy",
        ),
        DeclareLaunchArgument(
            "layout",
            default_value="ds4_linux",
            description="config/<name>.yaml；ds4_linux=蓝牙 hid-sony，ds4_linux_usb=USB 有线（ds4_generic 兜底布局）",
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
