r"""
F91 motor maintenance: standalone SetZero/SaveParam tool.

Run only while the product stack is STOPPED (the node owns an exclusive CAN
socket and refuses service while arm/gripper/zero-torque controllers are
active).

    ros2 launch a3_bringup motor_maintenance.launch.py can_interface:=can1

Then call one of:
    ros2 service call /a3/maintenance/set_zero \\
        a3_msgs/srv/MotorIdCommand '{motor_id: 255}'
    ros2 service call /a3/maintenance/save_parameters \\
        a3_msgs/srv/MotorIdCommand '{motor_id: 255}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    motor_ids = LaunchConfiguration("motor_ids")
    inter_command_delay_ms = LaunchConfiguration("inter_command_delay_ms")
    enforce_controller_interlock = LaunchConfiguration(
        "enforce_controller_interlock")

    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can1"),
        DeclareLaunchArgument("motor_ids", default_value="[1, 2, 3, 4, 5, 6, 7]"),
        DeclareLaunchArgument("inter_command_delay_ms", default_value="50"),
        DeclareLaunchArgument(
            "enforce_controller_interlock", default_value="true"),
        Node(
            package="a3_hardware_interface",
            executable="motor_maintenance",
            name="motor_maintenance",
            output="screen",
            parameters=[{
                "can_interface": can_interface,
                "motor_ids": motor_ids,
                "inter_command_delay_ms": inter_command_delay_ms,
                "enforce_controller_interlock": enforce_controller_interlock,
            }],
        ),
    ])
