#!/usr/bin/env python3
"""A3 arm orchestration layer (facade).

统一对外交互门面：状态机、电机初始化闭环、使能/失能、预设点、状态聚合、
示教录制/回放/保存、AI 模式、模式仲裁。底层能力全部通过调用现有服务/话题复用，
不重写 CAN 编解码/插值/规划（需求 F21）。
"""

from __future__ import annotations

import math
import os
import re
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from a3_can_bridge.srv import MotorCommand
from a3_msgs.msg import ArmStatus
from a3_msgs.srv import (
    GotoNamedPose,
    PlaybackTrajectory,
    SaveTrajectory,
    SetJointPositions,
)

JOINTS = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
    "L7_joint",
]

# 编排状态机
STATE_IDLE = "IDLE"
STATE_INIT = "INIT"
STATE_READY = "READY"
STATE_TRAJ = "TRAJ"
STATE_SERVO = "SERVO"
STATE_TEACH = "TEACH"
STATE_AI = "AI"
STATE_FAULT = "FAULT"

# 禁止运动类命令的底层控制模式
BLOCKED_MODES = {"ZERO_TORQUE", "SERVO", "GRAVITY_COMP"}


def _duration(sec: float) -> Duration:
    d = Duration()
    d.sec = int(math.floor(sec))
    d.nanosec = int(round((sec - d.sec) * 1e9))
    return d


def _sanitize_name(name: str) -> str:
    s = re.sub(r"[^A-Za-z0-9_-]", "_", (name or "").strip())
    return s or "trajectory"


class ArmController(Node):
    def __init__(self) -> None:
        super().__init__("a3_arm_controller")

        # 可重入回调组：服务回调内会同步调用电机服务（_wait_future 轮询等待），
        # 若用默认 MutuallyExclusiveCallbackGroup，回调阻塞期间 client 响应回调无法
        # 并发执行，导致 init/enable/disable/teach 全部死锁超时。
        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter("joint_names", JOINTS)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("control_mode_topic", "/a3/control_mode")
        self.declare_parameter("arm_status_topic", "/a3/arm_status")
        self.declare_parameter("gate_topic", "/power_sequence/gate_open")
        self.declare_parameter("power_state_topic", "/power_sequence/state")
        self.declare_parameter("traj_topic", "/joint_group_effort_controller/joint_trajectory")
        self.declare_parameter("status_hz", 10.0)
        self.declare_parameter("init_zero_tol_rad", 0.05)
        self.declare_parameter("init_timeout_s", 5.0)
        self.declare_parameter("require_gate", False)
        self.declare_parameter("goto_duration_s", 3.0)
        self.declare_parameter("goto_waypoints", 21)
        self.declare_parameter("trajectories_dir", "~/.a3/trajectories")
        self.declare_parameter("named_poses_pkg", "a3_description")

        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self._n_joints = len(self._joint_names)
        self._traj_topic = str(self.get_parameter("traj_topic").value)
        self._arm_status_topic = str(self.get_parameter("arm_status_topic").value)

        # 状态与镜像
        self._state = STATE_IDLE
        self._mode = "IDLE"
        self._gate_open = False
        self._power_state = ""
        self._message = ""
        self._traj_done_at = 0.0
        self._lock = threading.Lock()

        # 关节状态缓存
        self._positions: List[float] = [0.0] * self._n_joints
        self._velocities: List[float] = [0.0] * self._n_joints
        self._efforts: List[float] = [0.0] * self._n_joints
        self._have_js = False
        # jog（滑动条直驱）进行中标志：区分 TRAJ 是 jog 还是 goto/playback
        self._jogging = False

        # 示教录制
        self._recording = False
        self._record_start = 0.0
        self._record: List[Tuple[float, List[float]]] = []

        # 轨迹持久化目录
        self._traj_dir = os.path.expanduser(str(self.get_parameter("trajectories_dir").value))
        os.makedirs(self._traj_dir, exist_ok=True)

        # 命名预设点
        self._poses = self._load_poses()
        # 关节限位（来自 URDF，与前端滑动条上下限同源）
        self._joint_limits = self._load_joint_limits()

        # 订阅（/joint_states 为 best-effort；gate/state 为 transient_local 锁存）
        js_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            JointState, str(self.get_parameter("joint_states_topic").value), self._on_js, js_qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            String, str(self.get_parameter("control_mode_topic").value), self._on_mode, 10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Bool, str(self.get_parameter("gate_topic").value), self._on_gate, latched_qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            String, str(self.get_parameter("power_state_topic").value), self._on_power_state, latched_qos,
            callback_group=self._cb_group,
        )

        # 发布
        self._arm_status_pub = self.create_publisher(ArmStatus, self._arm_status_topic, 10)
        self._traj_pub = self.create_publisher(JointTrajectory, self._traj_topic, 10)
        self._mode_pub = self.create_publisher(
            String, str(self.get_parameter("control_mode_topic").value), 10
        )

        # 服务（对外门面）
        self.create_service(Trigger, "/a3/arm/init", self._init_cb, callback_group=self._cb_group)
        self.create_service(Trigger, "/a3/arm/enable", self._enable_cb, callback_group=self._cb_group)
        self.create_service(Trigger, "/a3/arm/disable", self._disable_cb, callback_group=self._cb_group)
        self.create_service(GotoNamedPose, "/a3/arm/goto_named_pose", self._goto_cb, callback_group=self._cb_group)
        self.create_service(
            SetJointPositions, "/a3/arm/set_joint_positions", self._set_joint_positions_cb,
            callback_group=self._cb_group,
        )
        self.create_service(Trigger, "/a3/arm/start_teach", self._start_teach_cb, callback_group=self._cb_group)
        self.create_service(Trigger, "/a3/arm/stop_teach", self._stop_teach_cb, callback_group=self._cb_group)
        self.create_service(SaveTrajectory, "/a3/arm/save_trajectory", self._save_cb, callback_group=self._cb_group)
        self.create_service(PlaybackTrajectory, "/a3/arm/playback", self._playback_cb, callback_group=self._cb_group)
        self.create_service(Trigger, "/a3/arm/enter_ai", self._enter_ai_cb, callback_group=self._cb_group)
        self.create_service(Trigger, "/a3/arm/exit_ai", self._exit_ai_cb, callback_group=self._cb_group)

        # 底层服务客户端
        self._motor_cli = self.create_client(MotorCommand, "/a3/motor/set_zero", callback_group=self._cb_group)
        self._enable_cli = self.create_client(MotorCommand, "/a3/motor/enable", callback_group=self._cb_group)
        self._reset_cli = self.create_client(MotorCommand, "/a3/motor/reset", callback_group=self._cb_group)
        self._zt_start_cli = self.create_client(Trigger, "/a3/zero_torque/start", callback_group=self._cb_group)
        self._zt_stop_cli = self.create_client(Trigger, "/a3/zero_torque/stop", callback_group=self._cb_group)

        # 状态发布定时器
        rate = max(1.0, float(self.get_parameter("status_hz").value))
        self.create_timer(1.0 / rate, self._publish_status, callback_group=self._cb_group)

        self.get_logger().info(
            f"a3_arm_controller ready: joints={self._n_joints} state={self._state} "
            f"status_topic={self._arm_status_topic}"
        )

    # ------------------------------------------------------------------ utils

    def _load_poses(self) -> Dict[str, List[float]]:
        try:
            pkg = str(self.get_parameter("named_poses_pkg").value)
            share = get_package_share_directory(pkg)
            path = os.path.join(share, "config", "named_poses.yaml")
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            return {
                name: list(spec["positions"])
                for name, spec in (data.get("poses") or {}).items()
            }
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cannot load named poses: {exc}")
            return {}

    def _load_joint_limits(self) -> Dict[str, Tuple[float, float]]:
        """从 a3_description/urdf/el_a3.urdf 读取各关节 limit lower/upper（与前端滑动条同源）。"""
        import xml.etree.ElementTree as ET

        limits: Dict[str, Tuple[float, float]] = {}
        try:
            share = get_package_share_directory("a3_description")
            path = os.path.join(share, "urdf", "el_a3.urdf")
            root = ET.parse(path).getroot()
            for joint in root.iter("joint"):
                name = joint.get("name", "")
                if name not in self._joint_names:
                    continue
                limit = joint.find("limit")
                if limit is None:
                    continue
                try:
                    lower = float(limit.get("lower"))
                    upper = float(limit.get("upper"))
                except (TypeError, ValueError):
                    continue
                limits[name] = (lower, upper)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cannot load joint limits from URDF: {exc}")
        return limits

    def _set_state(self, state: str, message: str = "") -> None:
        with self._lock:
            self._state = state
            if message:
                self._message = message
        self.get_logger().info(f"state -> {state}" + (f" ({message})" if message else ""))

    def _publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    def _wait_service(self, client, timeout_s: float = 2.0) -> bool:
        if client.service_is_ready():
            return True
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            if client.service_is_ready():
                return True
            time.sleep(0.05)
        return client.service_is_ready()

    def _wait_future(self, future, timeout_s: float = 2.0) -> bool:
        """轮询等待 future 完成。

        本节点跑在 MultiThreadedExecutor（默认 cpu_count 个线程），服务回调占住
        一个线程；若在回调里用 rclpy.spin_until_future_complete() 会把节点临时
        「转挂」到全局 SingleThreadedExecutor 并导致后续回调停摆（init/enable/
        disable/teach 全部超时）。改为纯轮询，靠 executor 其余线程处理 client
        响应回调来完成 future。
        """
        deadline = time.monotonic() + max(timeout_s, 0.0)
        while time.monotonic() < deadline and not future.done() and rclpy.ok():
            time.sleep(0.01)
        return bool(future.done() and future.result() is not None)

    def _motor_command(self, client, command: int) -> Tuple[bool, str]:
        if not self._wait_service(client):
            return False, "motor service unavailable"
        req = MotorCommand.Request()
        req.motor_id = 0  # 广播到映射内全部电机
        req.command = command
        future = client.call_async(req)
        if future is None:
            return False, "service call rejected"
        if self._wait_future(future, 2.0):
            resp = future.result()
            return bool(resp.success), resp.message
        return False, "service call timeout"

    def _call_trigger(self, client, label: str) -> Tuple[bool, str]:
        if not self._wait_service(client):
            return False, f"{label} unavailable"
        future = client.call_async(Trigger.Request())
        if future is None:
            return False, f"{label} rejected"
        if self._wait_future(future, 2.0):
            resp = future.result()
            return bool(resp.success), resp.message
        return False, f"{label} timeout"

    def _count_at_zero(self, tol: float) -> int:
        if not self._have_js:
            return 0
        count = 0
        for p in self._positions:
            if math.isfinite(p) and abs(p) <= tol:
                count += 1
        return count

    def _can_move(self) -> Tuple[bool, str]:
        if self.get_parameter("require_gate").value and not self._gate_open:
            return False, "gate closed"
        if self._mode in BLOCKED_MODES:
            return False, f"mode={self._mode}"
        if self._state in (STATE_INIT, STATE_TEACH, STATE_AI, STATE_TRAJ, STATE_SERVO):
            return False, f"state={self._state}"
        return True, ""

    def _schedule_back_to_ready(self, delay_s: float) -> None:
        self._traj_done_at = time.monotonic() + max(delay_s, 0.1)

    def _back_to_ready(self) -> None:
        self._jogging = False
        if self._state == STATE_TRAJ:
            self._set_state(STATE_READY, "trajectory finished")

    # ------------------------------------------------------------ subscriptions

    def _on_js(self, msg: JointState) -> None:
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for i, jn in enumerate(self._joint_names):
            if jn in name_to_idx and name_to_idx[jn] < len(msg.position):
                self._positions[i] = float(msg.position[name_to_idx[jn]])
            if jn in name_to_idx and name_to_idx[jn] < len(msg.velocity):
                self._velocities[i] = float(msg.velocity[name_to_idx[jn]])
            if jn in name_to_idx and name_to_idx[jn] < len(msg.effort):
                self._efforts[i] = float(msg.effort[name_to_idx[jn]])
        self._have_js = True

        if self._recording:
            now = time.monotonic()
            self._record.append((now - self._record_start, list(self._positions)))

    def _on_mode(self, msg: String) -> None:
        if msg.data:
            self._mode = msg.data

    def _on_gate(self, msg: Bool) -> None:
        self._gate_open = bool(msg.data)

    def _on_power_state(self, msg: String) -> None:
        self._power_state = msg.data or ""

    # ------------------------------------------------------------------ status

    def _publish_status(self) -> None:
        # TRAJ 结束后自动回到 READY（Humble 无 oneshot 定时器，用时间戳判断）
        if (
            self._state == STATE_TRAJ
            and self._traj_done_at > 0.0
            and time.monotonic() >= self._traj_done_at
        ):
            self._traj_done_at = 0.0
            self._back_to_ready()

        st = ArmStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            st.state = self._state
            st.message = self._message
        st.mode = self._mode
        st.joint_names = list(self._joint_names)
        st.positions = [float(p) for p in self._positions]
        st.velocities = [float(v) for v in self._velocities]
        st.efforts = [float(e) for e in self._efforts]
        self._arm_status_pub.publish(st)

    # ------------------------------------------------------------------ services

    def _init_cb(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._state in (STATE_TRAJ, STATE_SERVO, STATE_TEACH, STATE_AI):
            resp.success = False
            resp.message = f"busy in state={self._state}"
            return resp

        self._set_state(STATE_INIT, "init: set_zero")
        ok, msg = self._motor_command(self._motor_cli, 3)  # set_zero
        if not ok:
            self._set_state(STATE_FAULT, f"set_zero failed: {msg}")
            resp.success = False
            resp.message = msg
            return resp

        # 异步确认 7 电机到位
        tol = float(self.get_parameter("init_zero_tol_rad").value)
        timeout = float(self.get_parameter("init_timeout_s").value)
        deadline = time.monotonic() + timeout
        confirmed = 0
        while time.monotonic() < deadline and rclpy.ok():
            confirmed = self._count_at_zero(tol)
            if confirmed >= self._n_joints:
                break
            time.sleep(0.05)
        confirmed = self._count_at_zero(tol)

        if confirmed < self._n_joints:
            self._set_state(STATE_FAULT, f"zero confirmed {confirmed}/{self._n_joints}")
            resp.success = False
            resp.message = f"zero confirmed {confirmed}/{self._n_joints}"
            return resp

        ok, msg = self._motor_command(self._enable_cli, 1)  # enable
        if not ok:
            self._set_state(STATE_FAULT, f"enable failed: {msg}")
            resp.success = False
            resp.message = msg
            return resp

        self._set_state(STATE_READY, f"init ok {confirmed}/{self._n_joints} enabled")
        resp.success = True
        resp.message = f"init ok: zero confirmed {confirmed}/{self._n_joints}, enabled"
        return resp

    def _enable_cb(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._state in (STATE_TRAJ, STATE_SERVO, STATE_TEACH, STATE_AI):
            resp.success = False
            resp.message = f"busy in state={self._state}"
            return resp
        ok, msg = self._motor_command(self._enable_cli, 1)
        if ok:
            self._set_state(STATE_READY, "enabled")
        resp.success = ok
        resp.message = msg
        return resp

    def _disable_cb(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._state in (STATE_TRAJ, STATE_SERVO, STATE_TEACH, STATE_AI):
            resp.success = False
            resp.message = f"busy in state={self._state}"
            return resp
        ok, msg = self._motor_command(self._reset_cli, 2)  # reset = 失能
        if ok:
            self._set_state(STATE_IDLE, "disabled")
        resp.success = ok
        resp.message = msg
        return resp

    def _goto_cb(
        self, req: GotoNamedPose.Request, resp: GotoNamedPose.Response
    ) -> GotoNamedPose.Response:
        name = (req.pose_name or "").strip()
        if name not in self._poses:
            resp.success = False
            resp.message = f"unknown pose '{name}' (have: {sorted(self._poses)})"
            return resp
        can, why = self._can_move()
        if not can:
            resp.success = False
            resp.message = why
            return resp
        if not self._have_js:
            resp.success = False
            resp.message = "no /joint_states yet"
            return resp

        q1 = self._poses[name]
        if len(q1) < self._n_joints:
            q1 = q1 + [0.0] * (self._n_joints - len(q1))
        q0 = list(self._positions)
        duration = float(self.get_parameter("goto_duration_s").value)
        n = max(2, int(self.get_parameter("goto_waypoints").value))

        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)
        for i in range(n):
            alpha = i / (n - 1)
            pt = JointTrajectoryPoint()
            pt.positions = [a + alpha * (b - a) for a, b in zip(q0, q1)]
            pt.time_from_start = _duration(duration * alpha)
            traj.points.append(pt)

        self._publish_mode("TRAJ_RUNNING")
        self._traj_pub.publish(traj)
        self._set_state(STATE_TRAJ, f"goto {name}")
        self._schedule_back_to_ready(duration + 0.3)

        resp.success = True
        resp.message = f"goto {name} ({duration:.1f}s, {n} pts)"
        return resp

    def _set_joint_positions_cb(
        self, req: SetJointPositions.Request, resp: SetJointPositions.Response
    ) -> SetJointPositions.Response:
        """Web 滑动条 jog 直驱：设 7 关节目标位置，短插值下发执行层（需求 F23 扩展）。"""
        if not self._have_js:
            resp.success = False
            resp.message = "no /joint_states yet"
            return resp
        if self._state in (STATE_INIT, STATE_TEACH, STATE_AI, STATE_FAULT, STATE_SERVO):
            resp.success = False
            resp.message = f"busy in state={self._state}"
            return resp
        if self._mode in BLOCKED_MODES:
            resp.success = False
            resp.message = f"mode={self._mode}"
            return resp
        if self._state == STATE_TRAJ and not self._jogging:
            resp.success = False
            resp.message = "busy in goto/playback"
            return resp

        if len(req.positions) != self._n_joints:
            resp.success = False
            resp.message = f"need {self._n_joints} positions, got {len(req.positions)}"
            return resp

        target: List[float] = []
        clamped: List[str] = []
        for i, jn in enumerate(self._joint_names):
            v = float(req.positions[i])
            lo_hi = self._joint_limits.get(jn)
            if lo_hi:
                lo, hi = lo_hi
                if v < lo:
                    v = lo
                    clamped.append(jn)
                elif v > hi:
                    v = hi
                    clamped.append(jn)
            target.append(v)

        duration = float(req.duration) if req.duration and req.duration > 0 else 0.3
        duration = max(0.05, min(duration, 5.0))

        q0 = list(self._positions)
        n = 11
        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)
        for i in range(n):
            alpha = i / (n - 1)
            pt = JointTrajectoryPoint()
            pt.positions = [a + alpha * (b - a) for a, b in zip(q0, target)]
            pt.time_from_start = _duration(duration * alpha)
            traj.points.append(pt)

        self._publish_mode("TRAJ_RUNNING")
        self._traj_pub.publish(traj)
        self._jogging = True
        self._set_state(STATE_TRAJ, "jog")
        self._schedule_back_to_ready(duration + 0.5)

        resp.success = True
        msg = f"jog {self._n_joints} joints ({duration:.2f}s)"
        if clamped:
            msg += f"; clamped {','.join(sorted(set(clamped)))}"
        resp.message = msg
        return resp

    def _start_teach_cb(
        self, req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        if self._state not in (STATE_READY,):
            resp.success = False
            resp.message = f"require READY (now {self._state})"
            return resp
        if self._mode in BLOCKED_MODES:
            resp.success = False
            resp.message = f"mode={self._mode}"
            return resp

        ok, msg = self._call_trigger(self._zt_start_cli, "zero_torque/start")
        if not ok:
            resp.success = False
            resp.message = msg
            return resp

        self._record = []
        self._record_start = time.monotonic()
        self._recording = True
        self._set_state(STATE_TEACH, "teaching (drag)")
        resp.success = True
        resp.message = "teach started"
        return resp

    def _stop_teach_cb(
        self, req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        if self._state != STATE_TEACH:
            resp.success = False
            resp.message = f"not teaching (state={self._state})"
            return resp
        self._recording = False
        ok, msg = self._call_trigger(self._zt_stop_cli, "zero_torque/stop")
        self._set_state(STATE_READY, f"teach stopped ({len(self._record)} samples)")
        resp.success = True
        resp.message = f"recorded {len(self._record)} samples" + (
            f" (zero_torque/stop: {msg})" if not ok else ""
        )
        return resp

    def _save_cb(
        self, req: SaveTrajectory.Request, resp: SaveTrajectory.Response
    ) -> SaveTrajectory.Response:
        name = _sanitize_name(req.name)
        if not self._record:
            resp.success = False
            resp.message = "no recording to save"
            return resp

        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)
        for t, pos in self._record:
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in pos]
            pt.time_from_start = _duration(t)
            traj.points.append(pt)

        path = os.path.join(self._traj_dir, f"{name}.yaml")
        data = {
            "joint_names": list(traj.joint_names),
            "points": [
                {
                    "positions": list(p.positions),
                    "time_from_start_sec": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                }
                for p in traj.points
            ],
        }
        try:
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f)
        except Exception as exc:  # noqa: BLE001
            resp.success = False
            resp.message = f"save failed: {exc}"
            return resp

        resp.success = True
        resp.message = f"saved {len(traj.points)} pts"
        resp.path = path
        self.get_logger().info(f"trajectory saved to {path}")
        return resp

    def _playback_cb(
        self, req: PlaybackTrajectory.Request, resp: PlaybackTrajectory.Response
    ) -> PlaybackTrajectory.Response:
        name = _sanitize_name(req.name)
        path = os.path.join(self._traj_dir, f"{name}.yaml")
        if not os.path.exists(path):
            resp.success = False
            resp.message = f"trajectory not found: {path}"
            return resp
        can, why = self._can_move()
        if not can:
            resp.success = False
            resp.message = why
            return resp

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except Exception as exc:  # noqa: BLE001
            resp.success = False
            resp.message = f"load failed: {exc}"
            return resp

        traj = JointTrajectory()
        traj.joint_names = list(data.get("joint_names") or self._joint_names)
        for p in data.get("points") or []:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in p["positions"]]
            pt.time_from_start = _duration(float(p["time_from_start_sec"]))
            traj.points.append(pt)

        if not traj.points:
            resp.success = False
            resp.message = "empty trajectory"
            return resp

        duration = (
            traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9
        )
        self._publish_mode("TRAJ_RUNNING")
        self._traj_pub.publish(traj)
        self._set_state(STATE_TRAJ, f"playback {name}")
        self._schedule_back_to_ready(duration + 0.3)

        resp.success = True
        resp.message = f"playback {name} ({len(traj.points)} pts, {duration:.1f}s)"
        return resp

    def _enter_ai_cb(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._state in (STATE_TRAJ, STATE_SERVO, STATE_TEACH, STATE_AI):
            resp.success = False
            resp.message = f"busy in state={self._state}"
            return resp
        self._set_state(STATE_AI, "AI control")
        resp.success = True
        resp.message = "AI mode entered"
        return resp

    def _exit_ai_cb(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._state != STATE_AI:
            resp.success = False
            resp.message = f"not in AI (state={self._state})"
            return resp
        self._set_state(STATE_READY, "AI exited")
        resp.success = True
        resp.message = "AI mode exited"
        return resp


def main() -> None:
    rclpy.init()
    node = ArmController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
