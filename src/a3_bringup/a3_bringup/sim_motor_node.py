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
  * F32：发布 `/a3/motor/states`（7 条逐电机状态）并提供 `/a3/motor/mit_command|stop|
    set_mode|scan_and_collect`（MIT 保持超时自动取消；扫描返回 1..7 + 假 UID）。
    仿真不实现 gate 互锁（与真机有意分歧，见 docs/shared/TOPIC_CONTRACT.md）。
"""

from __future__ import annotations

import math
import threading
from typing import List, Optional

import rclpy
from a3_bringup.trajectory_spline import sample_joint_trajectory
from a3_can_bridge.msg import MotorState, MotorStates
from a3_can_bridge.srv import (
    MotorCommand,
    MotorMitCommand,
    MotorScanCollect,
    MotorSetMode,
    MotorStop,
    SetMotorParam,
)
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
        self.declare_parameter("motor_states_topic", "/a3/motor/states")
        self.declare_parameter("max_hold_duration_s", 30.0)

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
        # F32 MIT 保持（仿真）：hold 期间按 hz 重复施加目标，超时自动取消
        self._mit_hold = {
            "active": False,
            "motor": 0,
            "p": 0.0,
            "hz": 50.0,
            "end_time": None,
        }
        self._mit_hold_timer = None
        self._modes = [0] * self._n  # 0=mit 1=position 2=speed（装饰性）

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
        self._motor_states_pub = self.create_publisher(
            MotorStates, self.get_parameter("motor_states_topic").value, 50
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

        # ---- F32 电机调试服务（仿真对齐：零 CAN 可跑通 web 全链路）----
        self.create_service(
            MotorMitCommand, "/a3/motor/mit_command", self._mit_cb
        )
        self.create_service(
            MotorStop, "/a3/motor/stop", self._motor_stop_cb
        )
        self.create_service(
            MotorSetMode, "/a3/motor/set_mode", self._set_mode_cb
        )
        self.create_service(
            MotorScanCollect, "/a3/motor/scan_and_collect", self._scan_cb
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

    # ------------------------------------------------- F32 电机调试服务

    def _cancel_mit_hold(self) -> None:
        if self._mit_hold_timer is not None:
            self._mit_hold_timer.cancel()
            self.destroy_timer(self._mit_hold_timer)
            self._mit_hold_timer = None
        self._mit_hold["active"] = False
        self._mit_hold["end_time"] = None

    def _mit_cb(self, req, resp):
        motor = int(req.motor_id)
        if not 1 <= motor <= 127:
            resp.success = False
            resp.message = f"motor_id {motor} out of [1, 127]"
            return resp
        duration = float(req.hold_duration_s)
        p = float(req.position_rad)
        hz = float(req.hold_hz) if float(req.hold_hz) > 0 else 50.0
        hz = min(max(hz, 1.0), 200.0)
        with self._lock:
            if motor - 1 < self._n:
                self._target[motor - 1] = p
        if duration <= 0:
            with self._lock:
                if self._mit_hold["active"] and self._mit_hold["motor"] == motor:
                    self._cancel_mit_hold()
            resp.success = True
            resp.message = f"one-shot sent motor={motor} (sim)"
            return resp
        duration = min(duration, float(self.get_parameter("max_hold_duration_s").value))
        with self._lock:
            self._cancel_mit_hold()
            self._mit_hold["active"] = True
            self._mit_hold["motor"] = motor
            self._mit_hold["p"] = p
            self._mit_hold["hz"] = hz
            self._mit_hold["end_time"] = self.get_clock().now() + rclpy.duration.Duration(
                seconds=duration
            )

            def tick():
                with self._lock:
                    hold = self._mit_hold
                    if not hold["active"]:
                        return
                    if self.get_clock().now() >= hold["end_time"]:
                        self._cancel_mit_hold()
                        self.get_logger().info(
                            f"sim mit hold ended motor={hold['motor']}"
                        )
                        return
                    if hold["motor"] - 1 < self._n:
                        self._target[hold["motor"] - 1] = hold["p"]

            self._mit_hold_timer = self.create_timer(1.0 / hz, tick)
        resp.success = True
        resp.message = f"hold started motor={motor} ({duration:.1f} s @ {hz:.1f} Hz) (sim)"
        return resp

    def _motor_stop_cb(self, req, resp):
        motor = int(req.motor_id)
        if not 0 <= motor <= 127:
            resp.success = False
            resp.message = f"motor_id {motor} out of [0, 127]"
            return resp
        with self._lock:
            if motor == 0:
                self._cancel_mit_hold()
            elif self._mit_hold["active"] and self._mit_hold["motor"] == motor:
                self._cancel_mit_hold()
        resp.success = True
        resp.message = f"stopped motor={'all' if motor == 0 else motor} (sim)"
        return resp

    def _set_mode_cb(self, req, resp):
        motor = int(req.motor_id)
        mode = str(req.mode).strip().lower()
        if not 1 <= motor <= 127:
            resp.success = False
            resp.message = f"motor_id {motor} out of [1, 127]"
            return resp
        mode_val = {"mit": 0, "position": 1, "speed": 2}.get(mode)
        if mode_val is None:
            resp.success = False
            resp.message = f"mode must be mit|position|speed (got {mode!r})"
            return resp
        with self._lock:
            if motor - 1 < self._n:
                self._modes[motor - 1] = mode_val
            if self._mit_hold["active"] and self._mit_hold["motor"] == motor:
                self._cancel_mit_hold()
        resp.success = True
        resp.message = f"mode={mode} motor={motor} (sim)"
        return resp

    def _scan_cb(self, req, resp):
        ids = list(range(1, self._n + 1))
        resp.success = True
        resp.ids = [int(i) for i in ids]
        # 假 UID：0x0102030405060708 + id（仿真便于前端展示）
        resp.uids = [0x0102030405060708 + int(i) for i in ids]
        resp.message = "found {}: {}".format(
            len(ids),
            " ".join(
                f"id={i} uid={0x0102030405060708 + i:016X}" for i in ids
            ),
        )
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

            # F32：/a3/motor/states（7 条逐电机状态，web 电机调试页数据源）
            ms = MotorStates()
            ms.header.stamp = now.to_msg()
            t_now = now.nanoseconds * 1e-9
            for i in range(self._n):
                st = MotorState()
                st.motor_id = i + 1
                st.master_id = 0xFD
                st.position_rad = float(self._positions[i])
                st.speed_rad_s = float(self._velocities[i])
                st.torque_nm = float(self._effort[i])
                st.temperature_c = 28.0 + 2.0 * math.sin(t_now * 0.5 + i)
                st.mode_status = 2 if self._enabled else 0
                st.error_status = 0
                st.fault_mask = 0
                st.has_feedback = True
                st.fresh = True
                st.enabled = self._enabled
                ms.states.append(st)
        self._js_pub.publish(js)
        self._motor_states_pub.publish(ms)

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
