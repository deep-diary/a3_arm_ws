#!/usr/bin/env python3
"""Publish a short test JointTrajectory per TOPIC_CONTRACT."""

import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINTS = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
    "L7_joint",
]


class TrajectoryTestPublisher(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_test_publisher")
        self.declare_parameter("trajectory_topic", "/joint_group_effort_controller/joint_trajectory")
        self.declare_parameter("namespace_prefix", "")
        self.declare_parameter("delay_sec", 3.0)
        self.declare_parameter("duration_sec", 2.0)
        self.declare_parameter("publish_once", True)

        topic = self.get_parameter("trajectory_topic").value
        prefix = self.get_parameter("namespace_prefix").value.strip("/")
        if prefix:
            topic = f"/{prefix}{topic}" if topic.startswith("/") else f"/{prefix}/{topic}"

        self._pub = self.create_publisher(JointTrajectory, topic, 10)
        delay = self.get_parameter("delay_sec").value
        self._timer = self.create_timer(delay, self._on_timer)
        self._published = False

        self.get_logger().info(f"Will publish test trajectory to {topic} after {delay:.1f}s")

    def _on_timer(self) -> None:
        if self.get_parameter("publish_once").value and self._published:
            return

        duration = self.get_parameter("duration_sec").value
        msg = JointTrajectory()
        msg.joint_names = list(DEFAULT_JOINTS)

        # Keep XRCE-friendly size: few points, positions only (no vel/effort arrays)
        n_points = 10
        for i in range(n_points + 1):
            t = duration * i / n_points
            pt = JointTrajectoryPoint()
            pt.positions = [0.2 * math.sin(0.5 * t + j * 0.3) for j in range(7)]
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            msg.points.append(pt)

        self._pub.publish(msg)
        self._published = True
        self.get_logger().info(f"Published test trajectory ({len(msg.points)} points, {duration:.1f}s)")


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = TrajectoryTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
