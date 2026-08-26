#!/usr/bin/env python3
"""No-CAN trajectory executor with time interpolation (CloudEdge-mock semantics)."""

from __future__ import annotations

import threading
from typing import List, Optional

import rclpy
from a3_bringup.trajectory_spline import sample_joint_trajectory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class SimExecutor(Node):
    def __init__(self) -> None:
        super().__init__("a3_sim_executor")
        self.declare_parameter(
            "trajectory_topic", "/joint_group_effort_controller/joint_trajectory"
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("trajectory_interpolation_method", "auto")
        self.declare_parameter(
            "joint_names",
            [
                "L1_joint",
                "L2_joint",
                "L3_joint",
                "L4_joint",
                "L5_joint",
                "L6_joint",
                "L7_joint",
            ],
        )

        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self._interp_method = str(
            self.get_parameter("trajectory_interpolation_method").value
        )
        self._positions = [0.0] * len(self._joint_names)
        self._velocities = [0.0] * len(self._joint_names)
        self._effort = [0.0] * len(self._joint_names)
        self._lock = threading.Lock()
        self._traj: Optional[JointTrajectory] = None
        self._traj_start = None

        traj_topic = self.get_parameter("trajectory_topic").value
        js_topic = self.get_parameter("joint_states_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)

        self._sub = self.create_subscription(
            JointTrajectory, traj_topic, self._on_traj, 10
        )
        self._pub = self.create_publisher(JointState, js_topic, 10)
        period = 1.0 / max(rate_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"a3_sim_executor ready: sub={traj_topic} pub={js_topic} @ {rate_hz} Hz "
            f"interp={self._interp_method}"
        )

    def _on_traj(self, msg: JointTrajectory) -> None:
        if not msg.points:
            self.get_logger().warn("Empty trajectory ignored")
            return
        with self._lock:
            self._traj = msg
            self._traj_start = self.get_clock().now()
        self.get_logger().info(
            f"Trajectory received: {len(msg.points)} points, {len(msg.joint_names)} joints"
        )

    def _map_to_fixed(self, names: List[str], values: List[float]) -> List[float]:
        out = list(self._positions)
        if not names:
            for i, v in enumerate(values):
                if i < len(out):
                    out[i] = v
            return out
        for i, jn in enumerate(self._joint_names):
            if jn in names:
                src = names.index(jn)
                if src < len(values):
                    out[i] = values[src]
        return out

    def _on_timer(self) -> None:
        with self._lock:
            if self._traj is not None and self._traj_start is not None:
                elapsed = (self.get_clock().now() - self._traj_start).nanoseconds * 1e-9
                pos, vel, eff, finished = sample_joint_trajectory(
                    self._traj, elapsed, self._interp_method
                )
                names = list(self._traj.joint_names)
                self._positions = self._map_to_fixed(names, pos)
                if vel:
                    self._velocities = self._map_to_fixed(names, vel)
                if eff:
                    self._effort = self._map_to_fixed(names, eff)
                if finished:
                    self._traj = None
                    self._traj_start = None

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(self._joint_names)
            js.position = list(self._positions)
            js.velocity = list(self._velocities)
            js.effort = list(self._effort)
        self._pub.publish(js)


def main() -> None:
    rclpy.init()
    node = SimExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
