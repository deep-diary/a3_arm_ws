#!/usr/bin/env python3
"""No-CAN motor protocol simulator (feedback = command + first-order plant).

Replaces the real C++ `motor_protocol_node` in `edge_web_sim.launch.py`, so the
whole stack (arm controller + gripper force control + gravity + power sequence)
can run closed-loop without motors / CAN / battery.

Behaviours:
  * Subscribes `/joint_group_effort_controller/joint_trajectory` (from both
    `a3_arm_controller` multi-point trajectories and `gripper_controller`'s
    50 Hz single-point L7 stream) and time-interpolates it with
    `trajectory_spline.sample_joint_trajectory`.
  * Publishes `/joint_states` (7 joints, position + velocity + effort) @ 50 Hz.
    Each joint is a first-order follower toward the commanded target (small lag
    makes the gripper force loop observable).
  * L7 effort uses a "contact spring" model: `tau = contact_k * max(0, contact_q - q)`
    so the gripper force controller can genuinely converge to GRASPED.
  * Provides the `/a3/motor/*` services (`set_zero` zeroes feedback positions),
    `/a3/motor/set_param` (gripper torque-limit write) and `/a3/zero_torque/{start,stop}`
    (teach mode). `set_zero` resets all joints to 0 and clears any active trajectory.
  * Publishes `/a3/control_mode` on ZERO_TORQUE transitions and `/motor_feedback`
    summary, and honours `/power_sequence/gate_open`.
"""

from __future__ import annotations

import threading
from typing import List, Optional

import rclpy
from a3_bringup.trajectory_spline import sample_joint_trajectory
from a3_can_bridge.srv import MotorCommand, SetMotorParam
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

JOINTS = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
    "L7_joint",
]
L7_IDX = 6


class SimMotorNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_protocol_node")

        self.declare_parameter(
            "trajectory_topic", "/joint_group_effort_controller/joint_trajectory"
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("trajectory_interpolation_method", "auto")
        self.declare_parameter("joint_names", JOINTS)
        self.declare_parameter("follow_alpha", 0.35)
        self.declare_parameter("contact_q", 0.6)
        self.declare_parameter("contact_k", 20.0)
        self.declare_parameter("enable_contact", True)
        self.declare_parameter("control_mode_topic", "/a3/control_mode")
        self.declare_parameter("motor_feedback_topic", "/motor_feedback")
        self.declare_parameter("gate_topic", "/power_sequence/gate_open")
        self.declare_parameter("zero_torque_kp", 0.0)
        self.declare_parameter("zero_torque_kd", 1.0)

        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self._n = len(self._joint_names)
        self._interp_method = str(
            self.get_parameter("trajectory_interpolation_method").value
        )
        self._follow_alpha = float(self.get_parameter("follow_alpha").value)
        self._contact_q = float(self.get_parameter("contact_q").value)
        self._contact_k = float(self.get_parameter("contact_k").value)
        self._enable_contact = bool(self.get_parameter("enable_contact").value)

        self._positions = [0.0] * self._n
        self._velocities = [0.0] * self._n
        self._effort = [0.0] * self._n
        self._target = [0.0] * self._n
        self._enabled = False
        self._zero_torque = False
        self._gate_open = False
        self._lock = threading.Lock()
        self._traj: Optional[JointTrajectory] = None
        self._traj_start = None
        self._last_tick = None
        self._fb_seq = 0

        traj_topic = self.get_parameter("trajectory_topic").value
        js_topic = self.get_parameter("joint_states_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        gate_topic = self.get_parameter("gate_topic").value

        self._sub = self.create_subscription(
            JointTrajectory, traj_topic, self._on_traj, 10
        )
        self._gate_sub = self.create_subscription(
            Bool, gate_topic, self._on_gate, 10
        )
        self._js_pub = self.create_publisher(JointState, js_topic, 10)
        self._mode_pub = self.create_publisher(
            String, self.get_parameter("control_mode_topic").value, 10
        )
        self._feedback_pub = self.create_publisher(
            String, self.get_parameter("motor_feedback_topic").value, 50
        )

        # ---- 电机协议服务（F17 命名，广播/单电机均返回 ok）----
        def _motor_cb(command: int):
            def cb(req: MotorCommand.Request, resp: MotorCommand.Response):
                with self._lock:
                    if command == 3:  # set_zero
                        self._positions = [0.0] * self._n
                        self._velocities = [0.0] * self._n
                        self._target = [0.0] * self._n
                        self._traj = None
                        self._traj_start = None
                        self._effort = [0.0] * self._n
                    elif command == 1:  # enable
                        self._enabled = True
                    elif command == 2:  # reset / disable
                        self._enabled = False
                        self._zero_torque = False
                        self._publish_mode("IDLE")
                resp.success = True
                resp.message = f"sim ok ({command})"
                return resp

            return cb

        self.create_service(MotorCommand, "/a3/motor/set_zero", _motor_cb(3))
        self.create_service(MotorCommand, "/a3/motor/enable", _motor_cb(1))
        self.create_service(MotorCommand, "/a3/motor/reset", _motor_cb(2))
        self.create_service(MotorCommand, "/a3/motor/get_device_id", _motor_cb(0))
        self.create_service(MotorCommand, "/a3/motor/request_version", _motor_cb(4))

        self.create_service(
            SetMotorParam,
            "/a3/motor/set_param",
            lambda req, resp: self._set_param_cb(req, resp),
        )

        self.create_service(
            Trigger, "/a3/zero_torque/start", lambda req, resp: self._zt_start(req, resp)
        )
        self.create_service(
            Trigger, "/a3/zero_torque/stop", lambda req, resp: self._zt_stop(req, resp)
        )

        period = 1.0 / max(rate_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)
        self._fb_timer = self.create_timer(1.0, self._on_feedback_timer)

        self.get_logger().info(
            f"sim_motor_node (motor_protocol_node) ready: sub={traj_topic} "
            f"pub={js_topic} @ {rate_hz} Hz interp={self._interp_method} "
            f"contact_q={self._contact_q} contact_k={self._contact_k}"
        )

    # ---------------------------------------------------------------- services

    def _set_param_cb(self, req, resp):
        # 夹爪写入电机力矩硬限（0x700B）等参数；模拟直接确认成功
        resp.success = True
        resp.message = (
            f"sim ok motor={int(req.motor_id)} param=0x{int(req.param_id):04X} "
            f"value={float(req.value):.4f}"
        )
        return resp

    def _zt_start(self, req, resp):
        with self._lock:
            self._zero_torque = True
        self._publish_mode("ZERO_TORQUE")
        resp.success = True
        resp.message = "ZERO_TORQUE on (sim)"
        return resp

    def _zt_stop(self, req, resp):
        with self._lock:
            self._zero_torque = False
        self._publish_mode("IDLE")
        resp.success = True
        resp.message = "ZERO_TORQUE off (sim)"
        return resp

    def _publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    # ------------------------------------------------------------ callbacks

    def _on_gate(self, msg: Bool) -> None:
        with self._lock:
            self._gate_open = bool(msg.data)

    def _on_traj(self, msg: JointTrajectory) -> None:
        if not msg.points:
            self.get_logger().warn("Empty trajectory ignored")
            return

        # 单点轨迹（夹爪 L7 50 Hz 流式直驱）→ 直接设目标
        streaming = len(msg.points) == 1 and len(msg.joint_names) <= 6
        with self._lock:
            if streaming:
                names = list(msg.joint_names)
                pos = list(msg.points[0].positions)
                self._apply_target(names, pos)
                self._traj = None
                self._traj_start = None
                return

            self._traj = msg
            self._traj_start = self.get_clock().now()
            pos, _vel, _eff, _fin = sample_joint_trajectory(
                msg, 0.0, self._interp_method
            )
            self._apply_target(list(msg.joint_names), pos)

    def _apply_target(self, names: List[str], values: List[float]) -> None:
        for i, jn in enumerate(self._joint_names):
            if jn in names:
                src = names.index(jn)
                if src < len(values):
                    self._target[i] = float(values[src])

    def _on_timer(self) -> None:
        with self._lock:
            now = self.get_clock().now()
            dt = 1.0 / max(float(self.get_parameter("rate_hz").value), 1.0)
            if self._last_tick is not None:
                dt = max(1e-3, (now - self._last_tick).nanoseconds * 1e-9)
            self._last_tick = now

            if self._traj is not None and self._traj_start is not None:
                elapsed = (now - self._traj_start).nanoseconds * 1e-9
                pos, _vel, _eff, finished = sample_joint_trajectory(
                    self._traj, elapsed, self._interp_method
                )
                self._apply_target(list(self._traj.joint_names), pos)
                if finished:
                    self._traj = None
                    self._traj_start = None

            # 一阶跟随 + 速度估计 + L7 接触弹簧力矩
            for i in range(self._n):
                q_old = self._positions[i]
                q_new = q_old + (self._target[i] - q_old) * self._follow_alpha
                self._positions[i] = q_new
                self._velocities[i] = (q_new - q_old) / dt if dt > 1e-6 else 0.0
            self._effort = [0.0] * self._n
            if self._enable_contact and self._n > L7_IDX:
                penetration = max(0.0, self._contact_q - self._positions[L7_IDX])
                self._effort[L7_IDX] = self._contact_k * penetration

            js = JointState()
            js.header.stamp = now.to_msg()
            js.name = list(self._joint_names)
            js.position = list(self._positions)
            js.velocity = list(self._velocities)
            js.effort = list(self._effort)
        self._js_pub.publish(js)

    def _on_feedback_timer(self) -> None:
        with self._lock:
            self._fb_seq += 1
            line = (
                f"sim motor={int(self._fb_seq)} enabled={1 if self._enabled else 0} "
                f"zt={1 if self._zero_torque else 0} gate={1 if self._gate_open else 0} "
                f"tau_L7={self._effort[L7_IDX]:.3f}"
            )
        msg = String()
        msg.data = line
        self._feedback_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SimMotorNode()
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
