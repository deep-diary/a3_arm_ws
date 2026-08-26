#!/usr/bin/env python3
"""Gravity torque publisher (reBot-aligned C3 / F8).

Uses Pinocchio ``computeGeneralizedGravity`` on EL-A3 URDF for all 7 joints.
Falls back to planar approx only if Pinocchio/URDF cannot load.

Services (reBot-compatible naming under /a3):
  /a3/gravity_compensation/start  std_srvs/Trigger
  /a3/gravity_compensation/stop   std_srvs/Trigger

Mode interlocking: while a JointTrajectory is active (TRAJ_RUNNING), start is
rejected. Trajectory subscription sets the mode; finished trajectories clear it.
"""

from __future__ import annotations

import math
import os
from typing import List, Optional

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory


G = 9.80665

DEFAULT_JOINTS = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
    "L7_joint",
]


class GravityTorqueNode(Node):
    def __init__(self) -> None:
        super().__init__("a3_gravity_torque")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("gravity_torque_topic", "/a3/gravity_torque")
        self.declare_parameter("control_mode_topic", "/a3/control_mode")
        self.declare_parameter(
            "trajectory_topic", "/joint_group_effort_controller/joint_trajectory"
        )
        self.declare_parameter("enabled", True)
        self.declare_parameter("apply_calibrated_inertia", True)
        self.declare_parameter("joint_names", DEFAULT_JOINTS)
        # joint_states from sim/MoveIt are already in URDF frame → default 1.0.
        # Set to motor signs only when feeding motor-frame positions (reBot-style).
        self.declare_parameter(
            "joint_direction",
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        )
        self.declare_parameter(
            "tau_scale",
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        )

        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self._direction = np.array(
            self.get_parameter("joint_direction").get_parameter_value().double_array_value
            or [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            dtype=np.float64,
        )
        self._tau_scale = np.array(
            self.get_parameter("tau_scale").get_parameter_value().double_array_value
            or [1.0] * 7,
            dtype=np.float64,
        )
        n = len(self._joint_names)
        if self._direction.size != n:
            self._direction = np.ones(n)
        if self._tau_scale.size != n:
            self._tau_scale = np.ones(n)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._q = np.zeros(n, dtype=np.float64)
        self._mode = "IDLE"  # IDLE | TRAJ_RUNNING | GRAVITY_COMP
        self._traj_end_time: Optional[rclpy.time.Time] = None

        self._inertia = self._load_inertia()
        self._pin_model = None
        self._pin_data = None
        self._pin = None
        self._iq_map: List[int] = []
        self._backend = self._try_init_pinocchio()

        js_topic = self.get_parameter("joint_states_topic").value
        gt_topic = self.get_parameter("gravity_torque_topic").value
        mode_topic = self.get_parameter("control_mode_topic").value
        traj_topic = self.get_parameter("trajectory_topic").value

        self._sub = self.create_subscription(JointState, js_topic, self._on_js, 10)
        self._traj_sub = self.create_subscription(
            JointTrajectory, traj_topic, self._on_traj, 10
        )
        self._pub = self.create_publisher(JointState, gt_topic, 10)
        self._mode_pub = self.create_publisher(String, mode_topic, 10)
        self._srv_start = self.create_service(
            Trigger, "/a3/gravity_compensation/start", self._on_start
        )
        self._srv_stop = self.create_service(
            Trigger, "/a3/gravity_compensation/stop", self._on_stop
        )
        self._timer = self.create_timer(0.05, self._on_timer)

        self.get_logger().info(
            f"gravity_torque backend={self._backend} pub={gt_topic} "
            f"mode={mode_topic} enabled={self._enabled}"
        )

    def _load_inertia(self) -> dict:
        share = get_package_share_directory("a3_description")
        path = os.path.join(share, "config", "inertia_params.yaml")
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def _try_init_pinocchio(self) -> str:
        try:
            import pinocchio as pin  # type: ignore

            # Prefer source tree URDF (meshes path not required for dynamics)
            candidates = [
                os.path.join(
                    get_package_share_directory("a3_description"), "urdf", "el_a3.urdf"
                ),
                os.path.join(
                    os.path.dirname(__file__),
                    "..",
                    "..",
                    "a3_description",
                    "urdf",
                    "el_a3.urdf",
                ),
            ]
            urdf_path = next((p for p in candidates if os.path.isfile(p)), None)
            if urdf_path is None:
                self.get_logger().error("el_a3.urdf not found")
                return "approx_inertia"

            model = pin.buildModelFromUrdf(urdf_path)
            if bool(self.get_parameter("apply_calibrated_inertia").value):
                self._apply_calibrated_inertia(model)

            self._pin = pin
            self._pin_model = model
            self._pin_data = model.createData()
            self._iq_map = []
            for name in self._joint_names:
                jid = model.getJointId(name)
                if jid == 0 or model.nqs[jid] != 1:
                    self.get_logger().warn(f"Joint {name} missing/unsupported in URDF")
                    self._iq_map.append(-1)
                else:
                    self._iq_map.append(int(model.idx_qs[jid]))

            self.get_logger().info(
                f"Pinocchio model nq={model.nq} nv={model.nv} urdf={urdf_path}"
            )
            return "pinocchio"
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"Pinocchio unavailable ({exc}); using approx_inertia"
            )
            return "approx_inertia"

    def _apply_calibrated_inertia(self, model) -> None:
        """Best-effort override of link mass/COM from inertia_params.yaml."""
        joint_to_link = {
            "L2": "l1_link_urdf_asm",
            "L3": "l2_l3_urdf_asm",
            "L4": "l3_lnik_urdf_asm",
            "L5": "l4_l5_urdf_asm",
            "L6": "part_9",
        }
        params = self._inertia.get("inertia_params", {})
        if not params or not self._inertia.get("use_calibrated_params", False):
            return
        pin = self._pin
        applied = 0
        for key, link_name in joint_to_link.items():
            if key not in params:
                continue
            parent = None
            for fid, frame in enumerate(model.frames):
                if frame.name == link_name:
                    parent = frame.parentJoint
                    break
            if parent is None or parent <= 0 or parent >= len(model.inertias):
                continue
            p = params[key]
            mass = float(p.get("mass", model.inertias[parent].mass))
            com = np.array(p.get("com", [0.0, 0.0, 0.0]), dtype=np.float64)
            try:
                Y = model.inertias[parent]
                model.inertias[parent] = pin.Inertia(mass, com, Y.inertia)
                applied += 1
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Skip calibrated inertia {key}: {exc}")
        self.get_logger().info(f"Applied calibrated inertia to {applied} links")

    def _set_mode(self, mode: str) -> None:
        if mode == self._mode:
            return
        self._mode = mode
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)
        self.get_logger().info(f"control_mode → {mode}")

    def _on_start(self, _req, resp):
        if self._mode == "TRAJ_RUNNING":
            resp.success = False
            resp.message = "rejected: TRAJ_RUNNING (wait for trajectory to finish)"
            return resp
        self._enabled = True
        self._set_mode("GRAVITY_COMP")
        resp.success = True
        resp.message = f"gravity compensation enabled (backend={self._backend})"
        return resp

    def _on_stop(self, _req, resp):
        self._enabled = False
        if self._mode == "GRAVITY_COMP":
            self._set_mode("IDLE")
        resp.success = True
        resp.message = "gravity compensation disabled"
        return resp

    def _on_traj(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        if self._mode == "GRAVITY_COMP":
            self.get_logger().warn(
                "Trajectory received while GRAVITY_COMP — stopping gravity mode"
            )
            self._enabled = False
        last = msg.points[-1].time_from_start
        duration = float(last.sec) + float(last.nanosec) * 1e-9
        self._traj_end_time = self.get_clock().now() + rclpy.duration.Duration(
            seconds=max(duration, 0.1) + 0.2
        )
        self._set_mode("TRAJ_RUNNING")

    def _on_js(self, msg: JointState) -> None:
        for i, name in enumerate(self._joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self._q[i] = float(msg.position[idx])

    def _compute_tau(self) -> List[float]:
        if self._backend == "pinocchio" and self._pin_model is not None:
            pin = self._pin
            q = pin.neutral(self._pin_model)
            # Model space: apply joint_direction like reBot (q_model = q * direction)
            q_model_vals = self._q * self._direction
            for i, iq in enumerate(self._iq_map):
                if 0 <= iq < self._pin_model.nq:
                    q[iq] = float(q_model_vals[i])
            pin.computeGeneralizedGravity(self._pin_model, self._pin_data, q)
            tau_model = np.array(self._pin_data.g[: self._pin_model.nv], dtype=np.float64)
            # Map back to joint order and motor frame
            tau = np.zeros(len(self._joint_names), dtype=np.float64)
            for i, iq in enumerate(self._iq_map):
                if 0 <= iq < tau_model.size:
                    tau[i] = tau_model[iq]
            tau = tau * self._direction * self._tau_scale
            return [float(x) for x in tau]

        return self._approx_tau()

    def _approx_tau(self) -> List[float]:
        params = self._inertia.get("inertia_params", {})
        q2 = float(self._q[1]) if len(self._q) > 1 else 0.0
        q3 = float(self._q[2]) if len(self._q) > 2 else 0.0
        m2 = float(params.get("L2", {}).get("mass", 0.7))
        m3 = float(params.get("L3", {}).get("mass", 0.27))
        m4 = float(params.get("L4", {}).get("mass", 0.57))
        c2 = float(params.get("L2", {}).get("com", [0.1, 0, 0])[0])
        c3 = float(params.get("L3", {}).get("com", [-0.08, 0, 0])[0])
        c4 = float(params.get("L4", {}).get("com", [-0.02, 0, 0])[0])
        L2_len = abs(c2) + 0.15
        tau = [0.0] * len(self._joint_names)
        tau[1] = (m2 * c2 + (m3 + m4) * L2_len) * G * math.cos(q2)
        tau[2] = m3 * abs(c3) * G * math.cos(q2 + q3)
        # Rough L4 pitch contribution (not a substitute for Pinocchio)
        tau[3] = m4 * abs(c4) * G * math.sin(q2 + q3)
        return tau

    def _on_timer(self) -> None:
        # Clear TRAJ_RUNNING after trajectory end
        if self._mode == "TRAJ_RUNNING" and self._traj_end_time is not None:
            if self.get_clock().now() >= self._traj_end_time:
                self._traj_end_time = None
                self._set_mode("IDLE")

        mode_msg = String()
        mode_msg.data = self._mode
        self._mode_pub.publish(mode_msg)

        if not self._enabled:
            return
        tau = self._compute_tau()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = [float(x) for x in self._q]
        msg.effort = tau
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GravityTorqueNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
