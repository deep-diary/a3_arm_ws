#!/usr/bin/env python3
"""Publish a short test JointTrajectory per TOPIC_CONTRACT.

Modes:
  sine        — small sinusoidal joint motion (XRCE-friendly)
  forward_dx  — IK so the TCP moves +X in base_link (default 0.10 m)
"""

import math
import sys

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    import PyKDL
except ImportError:  # pragma: no cover
    PyKDL = None


DEFAULT_JOINTS = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
    "L7_joint",
]

# Revolute limits from el_a3.urdf (L1..L6). L7 is gripper, held at 0.
_JOINT_LIMITS = np.array(
    [
        [-2.79253, 2.79253],
        [0.0, 3.66519],
        [-4.01426, 0.0],
        [-1.0472, 1.5708],
        [-1.5708, 1.5708],
        [-1.5708, 1.5708],
    ]
)


def _kdl_frame(xyz, rpy):
    return PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))


def _revolute(name, xyz, rpy, axis):
    joint = PyKDL.Joint(name, PyKDL.Vector(0, 0, 0), PyKDL.Vector(*axis), PyKDL.Joint.RotAxis)
    return PyKDL.Segment(joint, _kdl_frame(xyz, rpy))


def _fixed(name, xyz, rpy):
    return PyKDL.Segment(PyKDL.Joint(name, PyKDL.Joint.Fixed), _kdl_frame(xyz, rpy))


def el_a3_arm_chain():
    """KDL chain base_link → end_effector matching el_a3.urdf (L1–L6 + fixed TCP)."""
    chain = PyKDL.Chain()
    chain.addSegment(_fixed("base_to_l1", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    chain.addSegment(_revolute("L1_joint", (0.0, 0.0, 0.06), (3.14159, 0.0, 0.0), (0.0, 0.0, -1.0)))
    chain.addSegment(
        _revolute("L2_joint", (-0.00964286, 0.0, -0.069), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0))
    )
    chain.addSegment(_revolute("L3_joint", (0.19, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)))
    chain.addSegment(
        _revolute("L4_joint", (-0.185, 0.064, 0.0), (-3.14159, 0.0, 0.0), (0.0, 0.0, -1.0))
    )
    chain.addSegment(
        _revolute("L5_joint", (-0.0492, -0.038, 0.0), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0))
    )
    chain.addSegment(
        _revolute("L6_joint", (0.00685, 0.0, 0.039), (-1.5708, 0.0, -1.5708), (0.0, 0.0, 1.0))
    )
    chain.addSegment(_fixed("end_effector_joint", (0.0, 0.0, -0.074), (0.0, 0.0, 0.0)))
    return chain


def _jnt(values):
    q = PyKDL.JntArray(len(values))
    for i, v in enumerate(values):
        q[i] = float(v)
    return q


def _fk_pos(fk, q):
    frame = PyKDL.Frame()
    fk.JntToCart(q, frame)
    return np.array([frame.p.x(), frame.p.y(), frame.p.z()])


def ik_delta_x(delta_x_m, q_start=None, max_iter=80):
    """Damped least-squares IK for TCP +X in base_link. Returns (q6, p_start, p_goal)."""
    chain = el_a3_arm_chain()
    n = chain.getNrOfJoints()
    fk = PyKDL.ChainFkSolverPos_recursive(chain)
    jac_solver = PyKDL.ChainJntToJacSolver(chain)
    if q_start is None:
        q_start = [0.0] * n
    q = _jnt(q_start)
    p0 = _fk_pos(fk, q)
    target = p0 + np.array([delta_x_m, 0.0, 0.0])
    for _ in range(max_iter):
        p = _fk_pos(fk, q)
        err = target - p
        if np.linalg.norm(err) < 1e-4:
            q_out = np.array([q[i] for i in range(n)])
            return q_out, p0, p
        jac = PyKDL.Jacobian(n)
        jac_solver.JntToJac(q, jac)
        j_pos = np.zeros((3, n))
        for i in range(n):
            col = jac.getColumn(i)
            j_pos[0, i] = col.vel.x()
            j_pos[1, i] = col.vel.y()
            j_pos[2, i] = col.vel.z()
        lam = 1e-3
        dq = j_pos.T @ np.linalg.solve(j_pos @ j_pos.T + lam * np.eye(3), err)
        dq = np.clip(dq, -0.2, 0.2)
        for i in range(n):
            q[i] = float(np.clip(q[i] + dq[i], _JOINT_LIMITS[i, 0], _JOINT_LIMITS[i, 1]))
    p = _fk_pos(fk, q)
    return np.array([q[i] for i in range(n)]), p0, p


def _duration(t_sec):
    return Duration(sec=int(t_sec), nanosec=int((t_sec % 1.0) * 1e9))


class TrajectoryTestPublisher(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_test_publisher")
        self.declare_parameter("trajectory_topic", "/a3/planned_joint_trajectory")
        self.declare_parameter("namespace_prefix", "")
        self.declare_parameter("delay_sec", 3.0)
        self.declare_parameter("duration_sec", 2.0)
        self.declare_parameter("publish_once", True)
        self.declare_parameter("mode", "forward_dx")
        self.declare_parameter("delta_x_m", 0.10)

        topic = self.get_parameter("trajectory_topic").value
        prefix = self.get_parameter("namespace_prefix").value.strip("/")
        if prefix:
            topic = f"/{prefix}{topic}" if topic.startswith("/") else f"/{prefix}/{topic}"

        self._pub = self.create_publisher(JointTrajectory, topic, 10)
        delay = self.get_parameter("delay_sec").value
        self._timer = self.create_timer(delay, self._on_timer)
        self._published = False

        mode = self.get_parameter("mode").value
        dx = self.get_parameter("delta_x_m").value
        self.get_logger().info(
            f"Will publish {mode} trajectory to {topic} after {delay:.1f}s (delta_x={dx:.3f} m)"
        )

    def _on_timer(self) -> None:
        if self.get_parameter("publish_once").value and self._published:
            return

        duration = float(self.get_parameter("duration_sec").value)
        mode = str(self.get_parameter("mode").value)
        if mode == "forward_dx":
            msg = self._forward_dx_msg(duration)
        else:
            msg = self._sine_msg(duration)

        self._pub.publish(msg)
        self._published = True
        self.get_logger().info(
            f"Published {mode} trajectory ({len(msg.points)} points, {duration:.1f}s)"
        )

    def _sine_msg(self, duration: float) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = list(DEFAULT_JOINTS)
        n_points = 10
        for i in range(n_points + 1):
            t = duration * i / n_points
            pt = JointTrajectoryPoint()
            pt.positions = [0.2 * math.sin(0.5 * t + j * 0.3) for j in range(7)]
            pt.time_from_start = _duration(t)
            msg.points.append(pt)
        return msg

    def _forward_dx_msg(self, duration: float) -> JointTrajectory:
        dx = float(self.get_parameter("delta_x_m").value)
        q_start = np.zeros(6)
        if PyKDL is None:
            self.get_logger().error("PyKDL missing; falling back to sine trajectory")
            return self._sine_msg(duration)
        q_goal, p0, p1 = ik_delta_x(dx, q_start)
        actual_dx = float(p1[0] - p0[0])
        self.get_logger().info(
            f"IK TCP {p0} -> {p1} (Δx={actual_dx:.4f} m, requested {dx:.4f} m); "
            f"q_goal={np.round(q_goal, 4).tolist()}"
        )
        if abs(actual_dx - dx) > 0.02:
            self.get_logger().warn(f"IK Δx error {abs(actual_dx - dx):.4f} m (limits/singularity)")

        msg = JointTrajectory()
        msg.joint_names = list(DEFAULT_JOINTS)
        n_points = 8
        for i in range(n_points + 1):
            alpha = i / n_points
            t = duration * alpha
            q6 = (1.0 - alpha) * q_start + alpha * q_goal
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q6] + [0.0]
            pt.time_from_start = _duration(t)
            msg.points.append(pt)
        return msg


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
