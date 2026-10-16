#!/usr/bin/env python3
"""USB 摄像头（UVC / v4l2）bringup：采集节点 + 可选 RViz 图像显示。

实现说明：走本仓 `a3_bringup.usb_camera_node`（cv2 采集），因为本机 RK3588 上
apt 的 ros-humble-usb-cam / v4l2-camera 依赖 libopencv-dev → libav*-dev 被
rockchip-multimedia PPA 的 pin 挡住，装不上。详见节点文件头注释。

默认设备为本机 USB 摄像头（Nozzle Alignment Camera，走 /dev/v4l/by-id 稳定路径）。
发布话题：/<camera_name>/image_raw、/<camera_name>/camera_info

用法：
  ros2 launch a3_bringup usb_cam.launch.py
  ros2 launch a3_bringup usb_cam.launch.py use_rviz:=true
  ros2 launch a3_bringup usb_cam.launch.py video_device:=/dev/video0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 本机摄像头默认设备（by-id 路径，插拔后序号变化也稳定）
DEFAULT_VIDEO_DEVICE = (
    "/dev/v4l/by-id/usb-DHZJ-250731-A_Nozzle_Alignment_Camera-video-index0"
)


def generate_launch_description():
    rviz_cfg = os.path.join(
        get_package_share_directory("a3_bringup"), "config", "usb_cam.rviz"
    )

    camera_name = LaunchConfiguration("camera_name")
    frame_id = LaunchConfiguration("frame_id")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_device",
                default_value=DEFAULT_VIDEO_DEVICE,
                description="v4l2 设备节点；推荐用 /dev/v4l/by-id/... 稳定路径",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="camera",
                description="节点命名空间，决定话题前缀 /<camera_name>/image_raw",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="camera_optical_frame",
                description="图像/camera_info 的 TF 坐标系",
            ),
            DeclareLaunchArgument(
                "image_width", default_value="640",
                description="要 30fps 需设 1280（720p 是设备唯一 30fps 模式，消息 2.7MB）",
            ),
            DeclareLaunchArgument("image_height", default_value="480"),
            DeclareLaunchArgument(
                "framerate",
                default_value="25.0",
                description="与分辨率绑定：640x480 上限 25fps，1280x720 上限 30fps",
            ),
            DeclareLaunchArgument(
                "overlay_fps",
                default_value="true",
                description="在画面左上角烧实时帧率；做视觉处理时可设 false",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="同时启动 RViz 显示图像（usb_cam.rviz）",
            ),
            Node(
                package="a3_bringup",
                executable="usb_camera_node",
                name="usb_cam",
                namespace=camera_name,
                parameters=[
                    {
                        "video_device": LaunchConfiguration("video_device"),
                        "frame_id": frame_id,
                        "image_width": LaunchConfiguration("image_width"),
                        "image_height": LaunchConfiguration("image_height"),
                        "framerate": LaunchConfiguration("framerate"),
                        "overlay_fps": LaunchConfiguration("overlay_fps"),
                    }
                ],
                output="screen",
            ),
            # RViz 的 Fixed Frame 需要存在，这里补一条静态 TF 避免报“Fixed Frame 不存在”
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_fixed_frame_tf",
                arguments=[
                    "--frame-id", "map",
                    "--child-frame-id", frame_id,
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_cfg],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
