#!/usr/bin/env python3
"""Minimal draw-rectangle demo via joint waypoints (sim-friendly)."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DrawRectangleDemo(Node):
    def __init__(self) -> None:
        super().__init__("a3_draw_rectangle_demo")
        self.declare_parameter(
            "trajectory_topic", "/joint_group_effort_controller/joint_trajectory"
        )
        self.declare_parameter("segment_duration_s", 1.5)
        self.declare_parameter("amplitude_rad", 0.25)
        topic = self.get_parameter("trajectory_topic").value
        self._pub = self.create_publisher(JointTrajectory, topic, 10)
        self._timer = self.create_timer(1.0, self._once)
        self._sent = False

    def _once(self) -> None:
        if self._sent:
            return
        self._sent = True
        amp = float(self.get_parameter("amplitude_rad").value)
        seg = float(self.get_parameter("segment_duration_s").value)
        # Rectangle in L2/L3 joint space around a lifted posture
        corners = [
            [0.0, 0.6, -0.6, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.6 + amp, -0.6, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.6 + amp, -0.6 - amp, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.6, -0.6 - amp, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.6, -0.6, 0.0, 0.0, 0.0, 0.0],
        ]
        traj = JointTrajectory()
        traj.joint_names = [
            "L1_joint",
            "L2_joint",
            "L3_joint",
            "L4_joint",
            "L5_joint",
            "L6_joint",
            "L7_joint",
        ]
        for i, pos in enumerate(corners):
            pt = JointTrajectoryPoint()
            pt.positions = pos
            # Non-zero mid velocities encourage cubic when method=auto
            if 0 < i < len(corners) - 1:
                pt.velocities = [0.0] * 7
            t = seg * i
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1.0) * 1e9)
            traj.points.append(pt)
        time.sleep(0.5)
        self._pub.publish(traj)
        self.get_logger().info(
            f"Published rectangle trajectory: {len(traj.points)} corners, seg={seg}s"
        )


def main() -> None:
    rclpy.init()
    node = DrawRectangleDemo()
    try:
        # Publish once via timer then exit cleanly
        import time

        t0 = time.time()
        while time.time() - t0 < 3.0 and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._sent:
                break
        time.sleep(0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
