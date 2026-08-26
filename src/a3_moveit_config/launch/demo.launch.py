"""
EL-A3 Robot Arm MoveIt Demo Launch File

Launches MoveIt motion planning interface (simulation mode)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 with MoveIt plugin",
        )
    )

    use_rviz = LaunchConfiguration("use_rviz")

    # Get URDF via xacro (mock hardware for demo)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("a3_description"), "urdf", "el_a3.urdf.xacro"]
            ),
            " use_mock_hardware:=true",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # SRDF
    robot_description_semantic_content = Command(
        [
            "cat ",
            PathJoinSubstitution(
                [FindPackageShare("a3_moveit_config"), "config", "el_a3.srdf"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # Kinematics
    kinematics_yaml = load_yaml("a3_moveit_config", "config/kinematics.yaml")

    # Joint limits
    joint_limits_yaml = load_yaml("a3_moveit_config", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    # Planning
    ompl_planning_yaml = load_yaml("a3_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {"move_group": ompl_planning_yaml}

    # Controllers
    moveit_controllers_yaml = load_yaml("a3_moveit_config", "config/moveit_controllers.yaml")

    # Trajectory execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_planning_scene_hz": 4.0,
    }

    # ros2_control controllers config
    ros2_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("a3_description"), "config", "el_a3_controllers.yaml"]
    )

    # RViz config
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("a3_moveit_config"), "config", "moveit.rviz"]
    )

    # Nodes
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="log",
    )

    # ros2_control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_yaml],
        output="both",
    )

    # Spawner nodes (with timeout to wait for controller_manager)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager",
                    "--controller-manager-timeout", "30"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager",
                    "--controller-manager-timeout", "30"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager",
                    "--controller-manager-timeout", "30"],
    )

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            robot_description_kinematics,
        ],
        condition=IfCondition(use_rviz),
    )

    delay_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ],
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

    nodes = [
        static_tf_node,
        robot_state_publisher_node,
        ros2_control_node,
        delay_spawners,
        move_group_node,
        rviz_node,
        maximize_rviz,
    ]

    return LaunchDescription(declared_arguments + nodes)

