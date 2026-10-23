"""F183 人脸跟踪启动：检测节点（mediapipe venv）+ 跟踪控制器（系统 python）。

为什么检测节点要用 ExecuteProcess 而不是 Node：
  mediapipe 装在 ~/.a3/venv/face_track，且必须让 venv 的 site-packages 排在
  PYTHONPATH **最前**（系统 /usr/lib/python3/dist-packages 里的 pyparsing 2.4.7
  会让 mediapipe 导入失败）。ament 安装的 console_script 用系统 python，跑不起来。

用法：
  ros2 launch a3_perception face_track.launch.py                 # 只出目标，不动臂
  ros2 launch a3_perception face_track.launch.py debug_image:=true
  # 确认方向无误后再打开使能（必须显式，默认关闭）：
  ros2 service call /a3/face_track/enable std_srvs/srv/SetBool "{data: true}"

前置：真机栈已起（cameras → /camera/image_raw，a3_servo_target → twist，/a3/ee_pose 可用）。
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("a3_perception")
    params_file = os.path.join(pkg, "config", "face_track.yaml")
    user_calib = os.path.expanduser("~/.a3/camera/face_calib.yaml")

    py = f"python{sys.version_info.major}.{sys.version_info.minor}"
    default_venv_py = os.path.expanduser("~/.a3/venv/face_track/bin/python3")
    default_venv_site = os.path.expanduser(f"~/.a3/venv/face_track/lib/{py}/site-packages")

    pf = [params_file, user_calib] if os.path.isfile(user_calib) else [params_file]
    detector_cmd = [LaunchConfiguration("venv_python"), "-m",
                    "a3_perception.face_detect_node", "--ros-args"]
    for f in pf:
        detector_cmd += ["--params-file", f]
    detector_cmd += ["-p", ["debug_image:=", LaunchConfiguration("debug_image")]]

    declared = [
        DeclareLaunchArgument("use_detector", default_value="true"),
        DeclareLaunchArgument("use_controller", default_value="true"),
        DeclareLaunchArgument("debug_image", default_value="false",
                             description="发布 /a3/face_track/debug_image 标注图"),
        DeclareLaunchArgument("venv_python", default_value=default_venv_py,
                             description="装了 mediapipe 的解释器"),
        DeclareLaunchArgument("venv_site", default_value=default_venv_site,
                             description="该 venv 的 site-packages（会前置到 PYTHONPATH）"),
    ]

    detector = ExecuteProcess(
        cmd=detector_cmd,
        additional_env={
            # venv 在前：保证 venv 里的 pyparsing/numpy 压过系统版本，同时保留 ROS 路径
            "PYTHONPATH": [LaunchConfiguration("venv_site"), ":",
                           EnvironmentVariable("PYTHONPATH", default_value="")],
        },
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_detector")),
    )

    controller = Node(
        package="a3_perception",
        executable="face_track_controller",
        name="face_track_controller",
        parameters=pf,
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_controller")),
    )

    return LaunchDescription(declared + [
        LogInfo(msg=f"[F183] params={pf}  venv={default_venv_py}"),
        detector,
        controller,
    ])
