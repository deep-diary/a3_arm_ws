#!/usr/bin/env python3
"""FollowJointTrajectory Action Server → trajectory topic (MoveIt Execute path)."""

from __future__ import annotations

import time
from typing import List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory


class FollowJointTrajectoryActionNode(Node):
    def __init__(self) -> None:
        super().__init__("a3_follow_joint_trajectory_action")
        self.declare_parameter("action_name", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter(
            "trajectory_topic", "/joint_group_effort_controller/joint_trajectory"
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("gate_topic", "/power_sequence/gate_open")
        self.declare_parameter("control_mode_topic", "/a3/control_mode")
        self.declare_parameter("require_gate", False)
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("success_hold_s", 0.2)

        self._cb = ReentrantCallbackGroup()
        self._gate_open = True
        self._mode = "IDLE"
        self._joint_pos: dict[str, float] = {}
        self._busy = False

        traj_topic = self.get_parameter("trajectory_topic").value
        self._pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        self._mode_pub = self.create_publisher(
            String, self.get_parameter("control_mode_topic").value, 10
        )
        self.create_subscription(
            Bool,
            self.get_parameter("gate_topic").value,
            lambda m: setattr(self, "_gate_open", bool(m.data)),
            10,
            callback_group=self._cb,
        )
        self.create_subscription(
            String,
            self.get_parameter("control_mode_topic").value,
            self._on_mode,
            10,
            callback_group=self._cb,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self._on_js,
            10,
            callback_group=self._cb,
        )

        action_name = self.get_parameter("action_name").value
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb,
        )
        self.get_logger().info(f"FJT Action Server on {action_name} → {traj_topic}")

    def _on_mode(self, msg: String) -> None:
        if msg.data:
            self._mode = msg.data

    def _on_js(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_pos[name] = float(msg.position[i])

    def _blocked_mode(self) -> bool:
        return self._mode in ("ZERO_TORQUE", "SERVO", "GRAVITY_COMP")

    def _goal_cb(self, goal_request) -> GoalResponse:
        if self.get_parameter("require_gate").value and not self._gate_open:
            self.get_logger().warn("Reject FJT: gate closed")
            return GoalResponse.REJECT
        if self._blocked_mode():
            self.get_logger().warn(f"Reject FJT: mode={self._mode}")
            return GoalResponse.REJECT
        if self._busy:
            self.get_logger().warn("Reject FJT: busy")
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_mode(self, mode: str) -> None:
        self._mode = mode
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    def _execute(self, goal_handle):
        self._busy = True
        result = FollowJointTrajectory.Result()
        traj: JointTrajectory = goal_handle.request.trajectory
        self._publish_mode("TRAJ_RUNNING")
        self._pub.publish(traj)

        last = traj.points[-1]
        t_end = float(last.time_from_start.sec) + float(last.time_from_start.nanosec) * 1e-9
        tol = float(self.get_parameter("goal_tolerance").value)
        hold = float(self.get_parameter("success_hold_s").value)
        names: List[str] = list(traj.joint_names)
        target = list(last.positions)
        deadline = time.monotonic() + t_end + 2.0
        ok_since: Optional[float] = None

        feedback = FollowJointTrajectory.Feedback()
        try:
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = "canceled"
                    self._publish_mode("IDLE")
                    self._busy = False
                    return result

                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = names
                feedback.desired.positions = target
                feedback.actual.positions = [
                    self._joint_pos.get(n, 0.0) for n in names
                ]
                goal_handle.publish_feedback(feedback)

                errs = [
                    abs(self._joint_pos.get(names[i], 0.0) - target[i])
                    for i in range(min(len(names), len(target)))
                ]
                if errs and max(errs) <= tol:
                    if ok_since is None:
                        ok_since = time.monotonic()
                    elif time.monotonic() - ok_since >= hold:
                        goal_handle.succeed()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "ok"
                        self._publish_mode("IDLE")
                        self._busy = False
                        return result
                else:
                    ok_since = None
                time.sleep(0.05)

            # Time elapsed: still succeed if close enough (sim-friendly)
            errs = [
                abs(self._joint_pos.get(names[i], 0.0) - target[i])
                for i in range(min(len(names), len(target)))
            ]
            if errs and max(errs) <= tol * 3.0:
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "ok_timeout_close"
            else:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = "timeout"
            self._publish_mode("IDLE")
            self._busy = False
            return result
        except Exception as exc:  # noqa: BLE001
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            self._publish_mode("IDLE")
            self._busy = False
            return result


def main() -> None:
    rclpy.init()
    node = FollowJointTrajectoryActionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
