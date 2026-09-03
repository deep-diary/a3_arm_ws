"""ROS 2 backend for the A3 LeRobot plugin.

This module is deliberately free of any ``lerobot`` import so it can be unit
tested against a plain ROS 2 Humble stack (e.g. the ``sim_executor`` sim arm).

Contract (mirrors ``src/a3_lerobot_config/config/a3_robot.yaml`` and
``docs/shared/TOPIC_CONTRACT.md``):
  * joint state is read from ``/joint_states`` (``sensor_msgs/JointState``),
    positions are radians in the URDF joint frame (NOT multiplied by
    ``joint_signs``);
  * actions are sent as a single-point ``trajectory_msgs/JointTrajectory`` on
    the executor main topic ``/joint_group_effort_controller/joint_trajectory``
    (``/a3/joint_trajectory`` is only a bridge alias);
  * everything is clipped to the per-joint soft limits; L7 (gripper) valid
    range is 0 (closed) ~ 1.5708 (open);
  * AI mode is entered/exited via the ``a3_arm_controller`` facade services
    ``/a3/arm/enter_ai`` and ``/a3/arm/exit_ai`` on a best-effort basis.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np

A3_JOINT_NAMES = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
    "L7_joint",
]

# rad, mirrors a3_robot.yaml joint_limits_rad
A3_JOINT_LIMITS_RAD = {
    "L1_joint": (-2.79253, 2.79253),
    "L2_joint": (0.0, 3.66519),
    "L3_joint": (-4.01426, 0.0),
    "L4_joint": (-1.5708, 1.5708),
    "L5_joint": (-1.5708, 1.5708),
    "L6_joint": (-1.5708, 1.5708),
    "L7_joint": (-1.5708, 1.5708),
}

DEFAULT_JOINT_STATES_TOPIC = "/joint_states"
DEFAULT_TRAJECTORY_TOPIC = "/joint_group_effort_controller/joint_trajectory"
DEFAULT_ENTER_AI_SERVICE = "/a3/arm/enter_ai"
DEFAULT_EXIT_AI_SERVICE = "/a3/arm/exit_ai"


class A3RosBackend:
    """Owns one rclpy node on a background spin thread."""

    def __init__(
        self,
        joint_states_topic: str = DEFAULT_JOINT_STATES_TOPIC,
        trajectory_topic: str = DEFAULT_TRAJECTORY_TOPIC,
        enter_ai_service: str = DEFAULT_ENTER_AI_SERVICE,
        exit_ai_service: str = DEFAULT_EXIT_AI_SERVICE,
        use_ai_mode: bool = True,
        service_timeout: float = 2.0,
        node_name: str = "lerobot_robot_a3",
    ):
        self.joint_states_topic = joint_states_topic
        self.trajectory_topic = trajectory_topic
        self.enter_ai_service = enter_ai_service
        self.exit_ai_service = exit_ai_service
        self.use_ai_mode = use_ai_mode
        self.service_timeout = service_timeout

        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}
        self._velocities: dict[str, float] = {}
        self._joint_state_seen = threading.Event()
        self._node_name = node_name

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def connect(self) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory
        from std_srvs.srv import Trigger

        if not rclpy.ok():
            rclpy.init()

        self._node = Node(self._node_name)
        # Dedicated executor for this node: the default global executor is shared
        # across spin_once() callers, which raises "generator already executing"
        # when several background threads (backend + cameras) spin concurrently.
        from rclpy.executors import SingleThreadedExecutor

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._node.create_subscription(
            JointState, self.joint_states_topic, self._on_joint_states, 10
        )
        self._traj_pub = self._node.create_publisher(
            JointTrajectory, self.trajectory_topic, 10
        )
        self._enter_ai_client = self._node.create_client(
            Trigger, self.enter_ai_service
        )
        self._exit_ai_client = self._node.create_client(
            Trigger, self.exit_ai_service
        )

        self._spin_thread = threading.Thread(
            target=self._spin_loop, name="lerobot_a3_spin", daemon=True
        )
        self._spin_thread.start()

        # Wait briefly for the executor's trajectory subscription to be matched
        # (DDS discovery), so the very first command from a fresh process is not
        # published before the subscriber exists and silently dropped.
        deadline = time.time() + self.service_timeout
        while time.time() < deadline:
            if self._traj_pub.get_subscription_count() > 0:
                break
            time.sleep(0.05)

        if self.use_ai_mode:
            self._call_trigger_best_effort(
                self._enter_ai_client, self.enter_ai_service
            )

    def disconnect(self) -> None:
        import rclpy

        try:
            if self.use_ai_mode and self._node is not None:
                self._call_trigger_best_effort(
                    self._exit_ai_client, self.exit_ai_service
                )
        finally:
            if self._executor is not None:
                try:
                    self._executor.shutdown()
                except Exception:
                    pass
            if self._node is not None:
                self._node.destroy_node()
                self._node = None
            if self._spin_thread is not None:
                self._spin_thread.join(timeout=2.0)
                self._spin_thread = None
            self._executor = None
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass

    # ------------------------------------------------------------------
    # observation
    # ------------------------------------------------------------------
    def wait_for_joint_states(self, timeout: float = 10.0) -> bool:
        return self._joint_state_seen.wait(timeout)

    def get_joint_positions(self) -> np.ndarray:
        """Return 7 joint positions (rad, URDF frame) in L1..L7 order."""
        with self._lock:
            if not self._positions:
                return np.zeros(len(A3_JOINT_NAMES), dtype=np.float64)
            return np.asarray(
                [self._positions.get(n, 0.0) for n in A3_JOINT_NAMES],
                dtype=np.float64,
            )

    def get_joint_velocities(self) -> np.ndarray:
        with self._lock:
            return np.asarray(
                [self._velocities.get(n, 0.0) for n in A3_JOINT_NAMES],
                dtype=np.float64,
            )

    # ------------------------------------------------------------------
    # action
    # ------------------------------------------------------------------
    def send_joint_target(self, positions_rad, dt: float = 0.1) -> None:
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        if self._node is None:
            raise RuntimeError("A3RosBackend not connected")

        positions = np.asarray(positions_rad, dtype=np.float64).reshape(-1)
        if positions.shape[0] != len(A3_JOINT_NAMES):
            raise ValueError(
                f"expected {len(A3_JOINT_NAMES)} joint targets, got {positions.shape[0]}"
            )

        clipped = self._clip_to_limits(positions)

        msg = JointTrajectory()
        msg.joint_names = list(A3_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in clipped]
        point.time_from_start.sec = int(dt)
        point.time_from_start.nanosec = int((dt - int(dt)) * 1e9)
        msg.points = [point]
        self._traj_pub.publish(msg)

    @staticmethod
    def _clip_to_limits(positions: np.ndarray) -> np.ndarray:
        out = positions.copy()
        for i, name in enumerate(A3_JOINT_NAMES):
            lo, hi = A3_JOINT_LIMITS_RAD[name]
            out[i] = float(np.clip(out[i], lo, hi))
        return out

    # ------------------------------------------------------------------
    # internals
    # ------------------------------------------------------------------
    def _spin_loop(self) -> None:
        import rclpy

        while self._node is not None and self._executor is not None and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                break

    def _on_joint_states(self, msg) -> None:
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._positions[name] = float(pos)
            for name, vel in zip(msg.name, msg.velocity or []):
                self._velocities[name] = float(vel)
        self._joint_state_seen.set()

    def _call_trigger_best_effort(self, client, service_name: str) -> None:
        if self._node is None:
            return
        if not client.wait_for_service(timeout_sec=self.service_timeout):
            self._node.get_logger().warn(
                f"service {service_name} not available; skipping (best-effort)"
            )
            return
        req = client.srv_type.Request()
        future = client.call_async(req)
        # The background SingleThreadedExecutor services the response; wait for it.
        deadline = time.time() + self.service_timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if future.done():
            try:
                res = future.result()
                self._node.get_logger().info(
                    f"{service_name} -> success={getattr(res, 'success', '?')} "
                    f"msg={getattr(res, 'message', '')!r}"
                )
            except Exception as exc:  # pragma: no cover
                self._node.get_logger().warn(f"{service_name} call failed: {exc}")
        else:
            self._node.get_logger().warn(
                f"{service_name} timed out; continuing (best-effort)"
            )
