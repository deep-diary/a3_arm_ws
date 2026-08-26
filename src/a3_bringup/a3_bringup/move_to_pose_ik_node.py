#!/usr/bin/env python3
"""Pinocchio MoveToPose IK service + optional MoveToPose action."""

from __future__ import annotations

import math
import os
from typing import List, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from a3_msgs.action import MoveToPose
from a3_msgs.srv import MoveToPoseIK

DEFAULT_JOINTS = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
]


class MoveToPoseIkNode(Node):
    def __init__(self) -> None:
        super().__init__("a3_move_to_pose_ik")
        self.declare_parameter("ee_frame", "end_effector")
        self.declare_parameter("joint_names", DEFAULT_JOINTS)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter(
            "trajectory_topic", "/joint_group_effort_controller/joint_trajectory"
        )
        self.declare_parameter("ik_eps", 1e-4)
        self.declare_parameter("ik_max_iter", 200)

        self._cb = ReentrantCallbackGroup()
        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self._q = np.zeros(len(self._joint_names), dtype=np.float64)
        self._pin = None
        self._model = None
        self._data = None
        self._ee_id = None
        self._iq_map: List[int] = []
        self._backend = self._init_pinocchio()

        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self._on_js,
            10,
            callback_group=self._cb,
        )
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            self.get_parameter("trajectory_topic").value,
            10,
        )
        self.create_service(MoveToPoseIK, "/a3/move_to_pose_ik", self._on_ik)
        self._action = ActionServer(
            self,
            MoveToPose,
            "/a3/move_to_pose",
            execute_callback=self._execute_move,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )
        self.get_logger().info(f"move_to_pose_ik backend={self._backend}")

    def _init_pinocchio(self) -> str:
        try:
            import pinocchio as pin

            urdf = os.path.join(
                get_package_share_directory("a3_description"), "urdf", "el_a3.urdf"
            )
            model = pin.buildModelFromUrdf(urdf)
            self._pin = pin
            self._model = model
            self._data = model.createData()
            ee = self.get_parameter("ee_frame").value
            self._ee_id = model.getFrameId(ee)
            if self._ee_id >= model.nframes:
                # try common alternates
                for cand in ("end_effector", "L6_link", "tool0", "link6"):
                    fid = model.getFrameId(cand)
                    if fid < model.nframes:
                        self._ee_id = fid
                        ee = cand
                        break
            self._iq_map = []
            for name in self._joint_names:
                jid = model.getJointId(name)
                if jid == 0 or model.nqs[jid] != 1:
                    self._iq_map.append(-1)
                else:
                    self._iq_map.append(int(model.idx_qs[jid]))
            self.get_logger().info(f"IK ee_frame={ee} id={self._ee_id}")
            return "pinocchio"
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Pinocchio IK unavailable: {exc}")
            return "none"

    def _on_js(self, msg: JointState) -> None:
        for i, name in enumerate(self._joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self._q[i] = float(msg.position[idx])

    def _solve_ik(
        self, pose: PoseStamped, seed: Optional[List[float]]
    ) -> tuple[bool, str, List[float]]:
        if self._backend != "pinocchio" or self._model is None:
            return False, "pinocchio unavailable", []
        pin = self._pin
        q = pin.neutral(self._model)
        seed_vals = seed if seed and len(seed) >= len(self._joint_names) else self._q
        for i, iq in enumerate(self._iq_map):
            if 0 <= iq < self._model.nq:
                q[iq] = float(seed_vals[i])

        oMdes = pin.SE3(
            pin.Quaternion(
                pose.pose.orientation.w,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
            ).normalized().matrix(),
            np.array(
                [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ]
            ),
        )
        eps = float(self.get_parameter("ik_eps").value)
        max_iter = int(self.get_parameter("ik_max_iter").value)
        for _ in range(max_iter):
            pin.forwardKinematics(self._model, self._data, q)
            pin.updateFramePlacements(self._model, self._data)
            iMd = self._data.oMf[self._ee_id].actInv(oMdes)
            err = pin.log(iMd).vector
            if np.linalg.norm(err) < eps:
                out = []
                for iq in self._iq_map:
                    out.append(float(q[iq]) if 0 <= iq < len(q) else 0.0)
                return True, "ok", out
            J = pin.computeFrameJacobian(
                self._model, self._data, q, self._ee_id, pin.ReferenceFrame.LOCAL
            )
            v = np.linalg.lstsq(J, err, rcond=None)[0]
            q = pin.integrate(self._model, q, v * 0.5)
        return False, "ik did not converge", []

    def _on_ik(self, req, resp):
        ok, msg, q = self._solve_ik(req.pose, list(req.seed_positions) if req.seed_positions else None)
        resp.success = ok
        resp.message = msg
        resp.joint_names = list(self._joint_names)
        resp.joint_positions = q
        return resp

    def _execute_move(self, goal_handle):
        result = MoveToPose.Result()
        target = goal_handle.request.target
        duration = max(float(goal_handle.request.duration_s), 0.5)
        seed = list(goal_handle.request.seed_positions)
        ok, msg, q = self._solve_ik(target, seed if seed else None)
        fb = MoveToPose.Feedback()
        fb.status = msg
        fb.joint_positions = q
        goal_handle.publish_feedback(fb)
        if not ok:
            goal_handle.abort()
            result.success = False
            result.message = msg
            return result

        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names) + ["L7_joint"]
        start = list(self._q) + [0.0]
        goal_q = list(q) + [0.0]
        p0 = JointTrajectoryPoint()
        p0.positions = start
        p0.time_from_start.sec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = goal_q
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        traj.points = [p0, p1]
        self._traj_pub.publish(traj)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "canceled"
            return result

        goal_handle.succeed()
        result.success = True
        result.message = "trajectory published"
        return result


def main() -> None:
    rclpy.init()
    node = MoveToPoseIkNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
