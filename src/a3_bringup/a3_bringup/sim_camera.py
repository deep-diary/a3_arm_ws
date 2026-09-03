#!/usr/bin/env python3
"""No-hardware RGB-D camera simulator for LeRobot data-pipeline testing.

Publishes a synthetic ``sensor_msgs/Image`` on ``/camera/color/image_raw``
(``rgb8``), ``/camera/depth/image_raw`` (``16UC1``, millimetres) and a matching
``sensor_msgs/CameraInfo``. The color frame is a numpy-generated gradient with
seven coloured blocks whose horizontal positions track the latest
``/joint_states`` joint angles, so the recorded video visibly changes as the
sim arm moves (exercising the camera observation path without a real device).

Pure numpy: no OpenCV / cv_bridge dependency. Mirrors the topic contract in
``docs/shared/TOPIC_CONTRACT.md``; the real Orbbec Gemini 2 driver publishes
the same topics.
"""

from __future__ import annotations

import math
import threading
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState


class SimCamera(Node):
    def __init__(self) -> None:
        super().__init__("a3_sim_camera")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("publish_depth", True)
        self.declare_parameter(
            "joint_names",
            ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint", "L7_joint"],
        )

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_depth = bool(self.get_parameter("publish_depth").value)
        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )

        self._lock = threading.Lock()
        self._positions = [0.0] * len(self._joint_names)
        self._frame_idx = 0

        self._color_pub = self.create_publisher(
            Image, str(self.get_parameter("color_topic").value), 10
        )
        self._depth_pub = self.create_publisher(
            Image, str(self.get_parameter("depth_topic").value), 10
        )
        self._info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), 10
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_states,
            10,
        )

        period = 1.0 / max(fps, 1.0)
        self.create_timer(period, self._on_timer)

        # Precompute a base vertical/horizontal gradient (HWC, uint8 RGB).
        ys = np.linspace(0, 255, self.height, dtype=np.float32)[:, None]
        xs = np.linspace(0, 255, self.width, dtype=np.float32)[None, :]
        self._base = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self._base[..., 0] = (xs * 0.4 + 40).astype(np.uint8)        # R: horizontal
        self._base[..., 1] = (ys * 0.5 + 30).astype(np.uint8)        # G: vertical
        self._base[..., 2] = ((ys + xs) * 0.3 + 60).astype(np.uint8) # B: diagonal

        self.get_logger().info(
            f"a3_sim_camera ready: {self.width}x{self.height}@{fps:.0f}Hz "
            f"color={self.get_parameter('color_topic').value} "
            f"depth={'on' if self.publish_depth else 'off'}"
        )

    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._joint_names:
                    self._positions[self._joint_names.index(name)] = float(pos)

    def _render_color(self, positions: List[float]) -> np.ndarray:
        img = self._base.copy()
        # Slowly vary background brightness with the frame counter.
        shift = int(30 * math.sin(self._frame_idx * 0.05))
        img = np.clip(img.astype(np.int16) + shift, 0, 255).astype(np.uint8)

        n = len(positions)
        block_h = max(12, self.height // (n * 3))
        block_w = max(16, self.width // 24)
        palette = [
            (255, 80, 80), (80, 255, 80), (80, 120, 255),
            (255, 200, 60), (220, 80, 255), (80, 220, 220), (255, 160, 200),
        ]
        for i, pos in enumerate(positions):
            # Map the joint angle (roughly -pi..pi) to an x offset within the frame.
            norm = 0.5 + 0.5 * math.sin(float(pos))  # 0..1 smooth
            cx = int(norm * (self.width - block_w - 20)) + 10
            cy = int((i + 0.5) * self.height / n)
            y0 = max(0, cy - block_h // 2)
            y1 = min(self.height, cy + block_h // 2)
            x0 = max(0, cx)
            x1 = min(self.width, cx + block_w)
            img[y0:y1, x0:x1] = palette[i % len(palette)]
        return np.ascontiguousarray(img, dtype=np.uint8)

    def _render_depth(self, positions: List[float]) -> np.ndarray:
        # Synthetic depth in millimetres: 0.6m baseline + joint-modulated gradient.
        depth = np.full((self.height, self.width), 700, dtype=np.uint16)
        ramp = np.linspace(0, 400, self.width, dtype=np.float32)
        depth = depth + ramp[None, :].astype(np.uint16)
        # Blocks slightly closer where the colored markers are.
        for i, pos in enumerate(positions):
            norm = 0.5 + 0.5 * math.sin(float(pos))
            cx = int(norm * (self.width - 40)) + 20
            cy = int((i + 0.5) * self.height / len(positions))
            depth[max(0, cy - 12):cy + 12, max(0, cx - 12):cx + 12] = 450
        return np.ascontiguousarray(depth, dtype=np.uint16)

    def _on_timer(self) -> None:
        with self._lock:
            positions = list(self._positions)
        self._frame_idx += 1
        stamp = self.get_clock().now().to_msg()

        color = self._render_color(positions)
        color_msg = Image()
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = self.frame_id
        color_msg.height = self.height
        color_msg.width = self.width
        color_msg.encoding = "rgb8"
        color_msg.is_bigendian = 0
        color_msg.step = self.width * 3
        color_msg.data = color.tobytes()
        self._color_pub.publish(color_msg)
        self._info_pub.publish(self._make_camera_info(stamp))

        if self.publish_depth:
            depth = self._render_depth(positions)
            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.frame_id
            depth_msg.height = self.height
            depth_msg.width = self.width
            depth_msg.encoding = "16UC1"
            depth_msg.is_bigendian = 0
            depth_msg.step = self.width * 2
            depth_msg.data = depth.tobytes()
            self._depth_pub.publish(depth_msg)

    def _make_camera_info(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.height = self.height
        info.width = self.width
        fx = fy = float(self.width)
        cx = self.width / 2.0
        cy = self.height / 2.0
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return info


def main() -> None:
    rclpy.init()
    node = SimCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
