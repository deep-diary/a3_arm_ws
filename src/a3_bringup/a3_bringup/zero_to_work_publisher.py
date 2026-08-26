#!/usr/bin/env python3
"""Publish a multi-point JointTrajectory from named pose A to B (default zero→work)."""

from __future__ import annotations

import math
import os
from typing import Dict, List

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _load_poses() -> Dict:
    share = get_package_share_directory("a3_description")
    path = os.path.join(share, "config", "named_poses.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _duration(sec: float) -> Duration:
    d = Duration()
    d.sec = int(math.floor(sec))
    d.nanosec = int(round((sec - d.sec) * 1e9))
    return d


class ZeroToWorkPublisher(Node):
    def __init__(self) -> None:
        super().__init__("zero_to_work_publisher")
        self.declare_parameter(
            "trajectory_topic", "/joint_group_effort_controller/joint_trajectory"
        )
        self.declare_parameter("start_pose", "zero")
        self.declare_parameter("goal_pose", "work")
        self.declare_parameter("duration_s", 3.0)
        self.declare_parameter("num_waypoints", 11)
        self.declare_parameter("publish_once", True)
        self.declare_parameter("delay_s", 1.0)

        data = _load_poses()
        self._joint_names: List[str] = list(data["joint_names"])
        self._poses = data["poses"]

        topic = self.get_parameter("trajectory_topic").value
        self._pub = self.create_publisher(JointTrajectory, topic, 10)
        delay = float(self.get_parameter("delay_s").value)
        self._timer = self.create_timer(delay, self._publish)
        self._done = False
        self.get_logger().info(f"Will publish {self.get_parameter('start_pose').value}"
                               f"→{self.get_parameter('goal_pose').value} on {topic}")

    def _publish(self) -> None:
        if self._done and self.get_parameter("publish_once").value:
            return
        start_name = self.get_parameter("start_pose").value
        goal_name = self.get_parameter("goal_pose").value
        duration = float(self.get_parameter("duration_s").value)
        n = max(2, int(self.get_parameter("num_waypoints").value))

        q0 = list(self._poses[start_name]["positions"])
        q1 = list(self._poses[goal_name]["positions"])

        msg = JointTrajectory()
        msg.joint_names = list(self._joint_names)
        for i in range(n):
            alpha = i / (n - 1)
            t = duration * alpha
            pt = JointTrajectoryPoint()
            pt.positions = [a + alpha * (b - a) for a, b in zip(q0, q1)]
            pt.time_from_start = _duration(t)
            msg.points.append(pt)

        self._pub.publish(msg)
        self._done = True
        self.get_logger().info(
            f"Published {start_name}→{goal_name}: {len(msg.points)} pts, {duration:.1f}s"
        )
        if self.get_parameter("publish_once").value:
            self._timer.cancel()


def main() -> None:
    rclpy.init()
    node = ZeroToWorkPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
