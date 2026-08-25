#!/usr/bin/env python3
"""move_group only (no ros2_control execute). Used by CloudEdge link test."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def moveit_overlay_env():
    """Prepend vendored MoveIt debs if apt packages are not installed."""
    extra = {}
    roots = []
    for p in os.environ.get("COLCON_PREFIX_PATH", "").split(os.pathsep):
        if p:
            roots.append(os.path.join(os.path.dirname(p), "src", "third_party", "moveit_debs"))
    roots.append(os.path.expanduser("~/a3_arm_ws/src/third_party/moveit_debs"))
    for root in roots:
        humble = os.path.join(root, "opt", "ros", "humble")
        if not os.path.isdir(os.path.join(humble, "share", "moveit_ros_move_group")):
            continue
        usr_lib = os.path.join(root, "usr", "lib", "aarch64-linux-gnu")
        lib_paths = [
            os.path.join(humble, "lib"),
            os.path.join(humble, "lib", "aarch64-linux-gnu"),
            usr_lib,
            os.environ.get("LD_LIBRARY_PATH", ""),
        ]
        extra["AMENT_PREFIX_PATH"] = humble + os.pathsep + os.environ.get(
            "AMENT_PREFIX_PATH", ""
        )
        extra["LD_LIBRARY_PATH"] = os.pathsep.join([p for p in lib_paths if p])
        py = os.path.join(humble, "local", "lib", "python3.10", "dist-packages")
        if os.path.isdir(py):
            extra["PYTHONPATH"] = py + os.pathsep + os.environ.get("PYTHONPATH", "")
        return extra
    return extra


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    declare_rviz = DeclareLaunchArgument("use_rviz", default_value="false")

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
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command(
        [
            "cat ",
            PathJoinSubstitution(
                [FindPackageShare("a3_moveit_config"), "config", "el_a3.srdf"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    kinematics_yaml = load_yaml("a3_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("a3_moveit_config", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}
    ompl_planning_yaml = load_yaml("a3_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {"move_group": ompl_planning_yaml}
    moveit_controllers_yaml = load_yaml("a3_moveit_config", "config/moveit_controllers.yaml")

    trajectory_execution = {
        "moveit_manage_controllers": False,
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

    planning_pipelines = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": ompl_planning_yaml,
    }

    overlay_env = moveit_overlay_env()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        additional_env=overlay_env,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            planning_pipelines,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="log",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("a3_moveit_config"), "config", "moveit.rviz"]
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

    return LaunchDescription(
        [
            declare_rviz,
            static_tf_node,
            robot_state_publisher_node,
            move_group_node,
            rviz_node,
        ]
    )
