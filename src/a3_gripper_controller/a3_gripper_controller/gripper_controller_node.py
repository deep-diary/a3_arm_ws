"""A3 夹爪（L7）自适应力控节点（需求 F24-F26）。

力外环（50 Hz PI，读 eff_L7）+ 电机位置内环：
  - FORCE 模式：e = tau_target - tau_meas；力不足 e>0 → 向闭合方向累加位置目标，
    力超了 e<0 → 回退；接触后电机位置环顶住物体，把接触力维持在设定值。
  - POSITION 模式：归一化 0..1 → L7 角度（沿用 PS4 开合语义，补上执行订阅）。
实时力环只在 Edge 本地运行；安全条款见 docs/shared/SAFETY.md「夹爪力控安全」。
"""

from __future__ import annotations

import math
import os
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from a3_msgs.msg import GripperStatus
from a3_msgs.srv import GripperCommand, GripperSetConfig
from a3_can_bridge.srv import SetMotorParam

# 状态机
ST_IDLE = "IDLE"
ST_POSITION = "POSITION"
ST_FORCE_CLOSING = "FORCE_CLOSING"
ST_GRASPED = "GRASPED"
ST_RELEASING = "RELEASING"
ST_FAULT = "FAULT"

# 错误码
ERR_NONE = 0
ERR_INTERLOCK = 1
ERR_GRASP_TIMEOUT = 2
ERR_WATCHDOG = 3
ERR_OVERTORQUE = 4
ERR_CONFIG = 5
ERR_FIRMWARE = 6

# MIT 力矩帧量程（protocol_codec.hpp kTMin/kTMax）
MIT_TORQUE_LIMIT_NM = 6.0
# 固件力矩限制参数索引（0x700B）
PARAM_TORQUE_LIMIT = 0x700B

GRIPPER_JOINT = "L7_joint"


def _duration(sec: float) -> Duration:
    d = Duration()
    d.sec = int(math.floor(sec))
    d.nanosec = int(round((sec - d.sec) * 1e9))
    return d


# 缺 key 时的内置安全默认值（与 gripper_config.yaml 对齐）
_DEFAULTS = {
    "gripper_motor_id": 7,
    "gripper_joint_index": 6,
    "torque_limit_param_id": float(PARAM_TORQUE_LIMIT),
    "max_grasp_torque_nm": 2.0,
    "default_target_torque_nm": 0.6,
    "torque_preset_weak_nm": 0.3,
    "torque_preset_medium_nm": 0.6,
    "torque_preset_strong_nm": 1.0,
    "gripper_torque_sign": 1.0,
    "force_kp": 0.5,
    "force_ki": 0.6,
    "integral_limit_nm": 0.5,
    "max_close_speed_rad_s": 0.3,
    "max_open_speed_rad_s": 0.6,
    # 接触检测：实测力矩 >= max(contact_detect_min_nm, contact_detect_ratio*target) 认为接触，
    # 接触前恒速软闭合不积分，接触后才切入 PI（防接触前积分饱和猛夹）
    "contact_detect_ratio": 0.35,
    "contact_detect_min_nm": 0.1,
    "force_mode_kp": 20.0,
    "force_mode_kd": 1.0,
    "position_mode_kp": 80.0,
    "position_mode_kd": 2.0,
    "gripper_open_rad": 1.5708,
    "gripper_close_rad": 0.0,
    "release_position": 1.0,
    "force_band_ratio": 0.10,
    "settle_s": 0.3,
    "grasp_timeout_s": 5.0,
    "feedback_fresh_timeout_s": 0.30,
    "overtorque_ratio": 1.0,
    "status_hz": 10.0,
    "force_status_hz": 50.0,
    "require_gate": False,
    "apply_firmware_torque_limit": True,
}


class GripperControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("gripper_controller_node")

        # ---- 声明参数（缺 key 用内置安全默认）----
        for key, val in _DEFAULTS.items():
            self.declare_parameter(key, val)
        for key, topic in {
            "joint_states_topic": "/joint_states",
            "control_mode_topic": "/a3/control_mode",
            "gate_topic": "/power_sequence/gate_open",
            "gripper_cmd_topic": "/a3/gripper_cmd",
            "gripper_status_topic": "/a3/gripper_status",
            "traj_topic": "/joint_group_effort_controller/joint_trajectory",
            "gains_cmd_topic": "/mit_gains_cmd",
            "set_param_service": "/a3/motor/set_param",
            "overrides_dir": "~/.a3/gripper",
        }.items():
            self.declare_parameter(key, topic)

        self._lock = threading.Lock()

        # 关节/电机
        self._joint_name = GRIPPER_JOINT
        self._joint_idx = int(self._p("gripper_joint_index"))
        self._motor_id = int(self._p("gripper_motor_id"))
        self._param_torque_limit = int(self._p("torque_limit_param_id"))

        # 行程
        self._q_open = float(self._p("gripper_open_rad"))
        self._q_close = float(self._p("gripper_close_rad"))
        self._release_pos = float(self._clamp01(self._p("release_position")))

        # 力控参数
        # 出厂硬上限（YAML，运行时不可越界）；_max_torque 为可调上限（可经服务下调并落盘）
        self._hard_max_torque = float(self._p("max_grasp_torque_nm"))
        self._max_torque = self._hard_max_torque
        self._default_target = float(self._p("default_target_torque_nm"))
        self._presets = {
            "weak": float(self._p("torque_preset_weak_nm")),
            "medium": float(self._p("torque_preset_medium_nm")),
            "strong": float(self._p("torque_preset_strong_nm")),
        }
        self._torque_sign = 1.0 if float(self._p("gripper_torque_sign")) >= 0 else -1.0
        self._kp = float(self._p("force_kp"))
        self._ki = float(self._p("force_ki"))
        self._int_limit = float(self._p("integral_limit_nm"))
        self._v_close = float(self._p("max_close_speed_rad_s"))
        self._v_open = float(self._p("max_open_speed_rad_s"))
        self._contact_detect_ratio = float(self._p("contact_detect_ratio"))
        self._contact_detect_min = float(self._p("contact_detect_min_nm"))
        self._force_kp_gain = float(self._p("force_mode_kp"))
        self._force_kd_gain = float(self._p("force_mode_kd"))
        self._pos_kp_gain = float(self._p("position_mode_kp"))
        self._pos_kd_gain = float(self._p("position_mode_kd"))
        self._band_ratio = float(self._p("force_band_ratio"))
        self._settle_s = float(self._p("settle_s"))
        self._grasp_timeout_s = float(self._p("grasp_timeout_s"))
        self._fb_timeout_s = float(self._p("feedback_fresh_timeout_s"))
        self._overtorque_ratio = float(self._p("overtorque_ratio"))
        self._require_gate = bool(self._p("require_gate"))
        self._apply_fw_limit = bool(self._p("apply_firmware_torque_limit"))

        # 运行时状态
        self._state = ST_IDLE
        self._mode = "stop"
        self._target_torque = 0.0
        self._meas_torque = 0.0
        self._q_cmd = self._q_open          # 力环输出的 L7 位置目标（rad）
        self._q_meas = self._q_open
        self._norm_pos = 1.0
        self._contact = False
        self._error_code = ERR_NONE
        self._message = "init"
        self._have_js = False
        self._gate_open = not self._require_gate
        self._arm_mode = "IDLE"

        # 力环积分/计时
        self._integral = 0.0
        self._contact_detected = False
        self._force_active = False
        self._force_start_time = 0.0
        self._in_band_since = 0.0
        self._last_js_time = 0.0
        self._last_tick_time = 0.0

        # 落盘
        self._overrides_dir = os.path.expanduser(str(self._p("overrides_dir")))
        os.makedirs(self._overrides_dir, exist_ok=True)
        self._load_overrides()

        # ---- ROS 接口 ----
        js_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            JointState, str(self._p("joint_states_topic")), self._on_js, js_qos
        )
        self.create_subscription(
            String, str(self._p("control_mode_topic")), self._on_arm_mode, 10
        )
        self.create_subscription(
            Bool, str(self._p("gate_topic")), self._on_gate, latched
        )
        self.create_subscription(
            Float32, str(self._p("gripper_cmd_topic")), self._on_gripper_cmd, 10
        )

        self._traj_pub = self.create_publisher(
            JointTrajectory, str(self._p("traj_topic")), 10
        )
        self._gains_pub = self.create_publisher(
            String, str(self._p("gains_cmd_topic")), 10
        )
        self._status_pub = self.create_publisher(
            GripperStatus, str(self._p("gripper_status_topic")), 10
        )

        self.create_service(GripperCommand, "/a3/gripper/command", self._cmd_cb)
        self.create_service(GripperSetConfig, "/a3/gripper/set_config", self._set_config_cb)
        self._set_param_cli = self.create_client(
            SetMotorParam, str(self._p("set_param_service"))
        )

        # 定时器：力环 50 Hz；状态发布按当前状态在力环内决定频率
        self._loop_hz = 50.0
        self._fw_limit_written = not self._apply_fw_limit
        self.create_timer(1.0 / self._loop_hz, self._tick)
        # 固件硬限写入重试（电机/服务可能在本节点之后才就绪）
        self.create_timer(3.0, self._fw_retry)

        self.get_logger().info(
            f"gripper_controller ready: max_torque={self._max_torque:.2f}Nm "
            f"presets={self._presets} sign={self._torque_sign:+.0f}"
        )

    # ---------- 工具 ----------
    def _p(self, key: str):
        return self.get_parameter(key).value

    @staticmethod
    def _clamp01(v: float) -> float:
        return max(0.0, min(1.0, float(v)))

    def _norm_from_rad(self, q: float) -> float:
        span = self._q_open - self._q_close
        if abs(span) < 1e-6:
            return 0.0
        return self._clamp01((q - self._q_close) / span)

    # ---------- 配置落盘（F24）----------
    @property
    def _overrides_path(self) -> str:
        return os.path.join(self._overrides_dir, "gripper_overrides.yaml")

    def _load_overrides(self) -> None:
        path = self._overrides_path
        if not os.path.exists(path):
            return
        try:
            import yaml
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            if "max_torque_nm" in data:
                v = float(data["max_torque_nm"])
                if 0 < v <= MIT_TORQUE_LIMIT_NM:
                    # 可调上限不得超过出厂硬上限
                    self._max_torque = min(v, self._hard_max_torque)
                    self.get_logger().info(f"loaded override max_torque={self._max_torque:.2f}Nm")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"load overrides failed: {exc}")

    def _save_overrides(self) -> None:
        try:
            import yaml
            with open(self._overrides_path, "w", encoding="utf-8") as f:
                yaml.safe_dump({"max_torque_nm": float(self._max_torque)}, f)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"save overrides failed: {exc}")

    def _validate_torque(self, tau: float) -> Optional[str]:
        """返回 None 表示合法，否则返回错误原因。"""
        if tau is None or not math.isfinite(tau):
            return "torque not finite"
        if tau <= 0:
            return "torque must be > 0"
        if tau > self._max_torque + 1e-6:
            return f"torque {tau:.2f} exceeds max {self._max_torque:.2f}Nm"
        if tau > MIT_TORQUE_LIMIT_NM:
            return f"torque {tau:.2f} exceeds MIT range {MIT_TORQUE_LIMIT_NM}Nm"
        return None

    # ---------- 固件硬限（F24 双保险）----------
    def _fw_retry(self) -> None:
        """服务未就绪时周期性重试写入固件力矩限制；就绪后只写一次。"""
        if self._fw_limit_written:
            return
        if self._set_param_cli.service_is_ready():
            self._apply_firmware_limit()

    def _apply_firmware_limit(self) -> bool:
        if not self._apply_fw_limit:
            self._fw_limit_written = True
            return True
        if not self._set_param_cli.service_is_ready():
            self.get_logger().warn("set_param service not ready; retry later", throttle_duration_sec=10)
            return False
        req = SetMotorParam.Request()
        req.motor_id = int(self._motor_id)
        req.param_id = int(self._param_torque_limit)
        req.value = float(self._max_torque)
        future = self._set_param_cli.call_async(req)
        future.add_done_callback(self._on_set_param_done)
        return True

    def _on_set_param_done(self, future) -> None:
        try:
            resp = future.result()
            if resp is None or not resp.success:
                self.get_logger().error(
                    f"firmware torque limit write failed: {getattr(resp, 'message', 'no resp')}"
                )
                with self._lock:
                    self._error_code = ERR_FIRMWARE
                    self._message = "firmware torque limit write failed"
            else:
                self._fw_limit_written = True
                self.get_logger().info(
                    f"firmware torque limit {self._max_torque:.2f}Nm -> motor {self._motor_id}"
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"set_param exception: {exc}")
            with self._lock:
                self._error_code = ERR_FIRMWARE
                self._message = f"set_param exception: {exc}"

    # ---------- 订阅回调 ----------
    def _on_js(self, msg: JointState) -> None:
        # 优先按关节名取 L7；回退到索引
        idx = -1
        for i, n in enumerate(msg.name):
            if n == self._joint_name:
                idx = i
                break
        if idx < 0:
            idx = self._joint_idx
        if idx >= len(msg.position):
            return
        with self._lock:
            self._q_meas = float(msg.position[idx])
            self._norm_pos = self._norm_from_rad(self._q_meas)
            if idx < len(msg.effort) and math.isfinite(msg.effort[idx]):
                # 握力反馈取力矩幅值（夹紧阻力）；用绝对值对反馈方向符号不敏感，
                # 避免真机力矩符号标定反了导致 PI 反向跑飞猛夹。
                self._meas_torque = abs(float(msg.effort[idx]))
            self._have_js = True
            self._last_js_time = self.get_clock().now().nanoseconds * 1e-9

    def _on_arm_mode(self, msg: String) -> None:
        with self._lock:
            self._arm_mode = str(msg.data or "IDLE")

    def _on_gate(self, msg: Bool) -> None:
        with self._lock:
            self._gate_open = bool(msg.data)

    def _on_gripper_cmd(self, msg: Float32) -> None:
        # POSITION 开合语义（0 闭…1 开）
        v = self._clamp01(float(msg.data))
        self._goto_position(v)

    # ---------- 互锁 ----------
    def _interlocked(self) -> Optional[str]:
        """返回 None 表示允许力控，否则返回拒绝原因。"""
        if self._require_gate and not self._gate_open:
            return "gate closed"
        blocked = {"TRAJ_RUNNING", "SERVO", "ZERO_TORQUE", "GRAVITY_COMP"}
        if self._arm_mode in blocked:
            return f"arm mode {self._arm_mode} active"
        return None

    # ---------- 服务回调 ----------
    def _cmd_cb(self, req, resp):
        mode = str(req.mode or "").strip().lower()
        with self._lock:
            if mode == "position":
                v = self._clamp01(float(req.position))
                self._goto_position(v)
                resp.success = True
                resp.message = f"position {v:.2f}"
            elif mode == "force":
                if req.torque_nm and req.torque_nm > 0:
                    tau = float(req.torque_nm)
                else:
                    preset = str(getattr(req, "preset", "") or "").strip().lower()
                    if preset:
                        if preset not in self._presets:
                            resp.success = False
                            resp.message = f"unknown preset '{preset}'"
                            self._error_code = ERR_CONFIG
                            return resp
                        tau = float(self._presets[preset])
                    else:
                        tau = self._default_target
                reason = self._validate_torque(tau)
                if reason:
                    resp.success = False
                    resp.message = reason
                    self._error_code = ERR_CONFIG
                    self._message = reason
                    return resp
                lock = self._interlocked()
                if lock:
                    resp.success = False
                    resp.message = f"interlock: {lock}"
                    self._error_code = ERR_INTERLOCK
                    self._message = resp.message
                    return resp
                timeout = float(req.timeout_s) if req.timeout_s and req.timeout_s > 0 else self._grasp_timeout_s
                self._start_force(tau, timeout)
                resp.success = True
                resp.message = f"force grasp {tau:.2f}Nm"
            elif mode == "release":
                self._release()
                resp.success = True
                resp.message = "release"
            elif mode == "stop":
                self._stop_force()
                resp.success = True
                resp.message = "stop"
            else:
                resp.success = False
                resp.message = f"unknown mode '{mode}'"
        return resp

    def _set_config_cb(self, req, resp):
        key = str(req.key or "").strip()
        value = float(req.value)
        with self._lock:
            if key != "max_torque_nm":
                resp.success = False
                resp.message = f"unknown config key '{key}'"
                resp.applied_value = self._max_torque
                return resp
            if not math.isfinite(value) or value <= 0:
                resp.success = False
                resp.message = "max_torque must be > 0"
                resp.applied_value = self._max_torque
                self._error_code = ERR_CONFIG
                return resp
            if value > self._hard_max_torque + 1e-6:
                resp.success = False
                resp.message = (
                    f"value {value:.2f} exceeds hard limit {self._hard_max_torque:.2f}Nm"
                )
                resp.applied_value = self._max_torque
                self._error_code = ERR_CONFIG
                self._message = resp.message
                return resp
            if value > MIT_TORQUE_LIMIT_NM:
                resp.success = False
                resp.message = f"value {value:.2f} exceeds MIT range"
                resp.applied_value = self._max_torque
                self._error_code = ERR_CONFIG
                return resp
            self._max_torque = value
            self._save_overrides()
            # 同步更新固件硬限
            self._apply_firmware_limit()
            resp.success = True
            resp.message = f"max_torque set to {value:.2f}Nm"
            resp.applied_value = value
            self._error_code = ERR_NONE
            self._message = resp.message
        return resp

    # ---------- 命令实现 ----------
    def _goto_position(self, norm_v: float) -> None:
        """POSITION 模式：归一化 0..1 → L7 角度，单次轨迹。"""
        self._force_active = False
        self._integral = 0.0
        self._state = ST_POSITION
        self._mode = "position"
        self._contact = False
        if self._error_code in (ERR_GRASP_TIMEOUT, ERR_WATCHDOG, ERR_OVERTORQUE):
            self._error_code = ERR_NONE
        q = self._q_close + norm_v * (self._q_open - self._q_close)
        self._q_cmd = q
        self._set_gains(self._pos_kp_gain, self._pos_kd_gain)
        self._publish_traj(q, duration=0.6)

    def _start_force(self, tau: float, timeout: float) -> None:
        """FORCE 模式：从当前位置开始，先恒速软闭合，接触后 PI 力环调节。"""
        self._mode = "force"
        self._target_torque = tau
        self._integral = 0.0
        self._contact = False
        self._contact_detected = False
        self._error_code = ERR_NONE
        self._force_active = True
        self._state = ST_FORCE_CLOSING
        now = self.get_clock().now().nanoseconds * 1e-9
        self._force_start_time = now
        self._in_band_since = 0.0
        # 力环从当前实测位置起步，避免阶跃
        self._q_cmd = self._q_meas if self._have_js else self._q_open
        self._last_tick_time = now
        # 力控前确保固件力矩硬限已写入（双保险；软件 clamp 始终生效）
        self._apply_firmware_limit()
        self._set_gains(self._force_kp_gain, self._force_kd_gain)
        self.get_logger().info(f"FORCE start target={tau:.2f}Nm from q={self._q_cmd:.3f}")

    def _release(self) -> None:
        self._force_active = False
        self._integral = 0.0
        self._contact = False
        self._state = ST_RELEASING
        self._mode = "release"
        if self._error_code in (ERR_GRASP_TIMEOUT, ERR_WATCHDOG, ERR_OVERTORQUE):
            self._error_code = ERR_NONE
        q = self._q_close + self._release_pos * (self._q_open - self._q_close)
        self._q_cmd = q
        self._set_gains(self._pos_kp_gain, self._pos_kd_gain)
        self._publish_traj(q, duration=0.8)
        self._state = ST_IDLE

    def _stop_force(self) -> None:
        self._force_active = False
        self._integral = 0.0
        self._contact = False
        if self._state not in (ST_FAULT,):
            self._state = ST_IDLE
        self._mode = "stop"
        self._set_gains(self._pos_kp_gain, self._pos_kd_gain)

    def _fault(self, code: int, message: str) -> None:
        self._force_active = False
        self._integral = 0.0
        self._state = ST_FAULT
        self._error_code = code
        self._message = message
        self._contact = False
        self.get_logger().error(f"GRIPPER FAULT [{code}]: {message}")

    # ---------- 下发 ----------
    def _set_gains(self, kp: float, kd: float) -> None:
        # per-motor 覆盖（motor_protocol_node 扩展：motor=<id> 作用于单电机）
        msg = String()
        msg.data = f"motor={self._motor_id} kp={kp:.3f} kd={kd:.3f}"
        self._gains_pub.publish(msg)

    def _publish_traj(self, q: float, duration: float = 0.05) -> None:
        q = max(self._q_close, min(self._q_open, q))
        traj = JointTrajectory()
        traj.joint_names = [self._joint_name]
        pt = JointTrajectoryPoint()
        pt.positions = [float(q)]
        pt.time_from_start = _duration(max(duration, 0.02))
        traj.points = [pt]
        self._traj_pub.publish(traj)

    # ---------- 力环主循环（50 Hz）----------
    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            if self._force_active:
                self._tick_force(now)
            self._publish_status_locked(now)

    def _tick_force(self, now: float) -> None:
        dt = now - self._last_tick_time if self._last_tick_time > 0 else 1.0 / self._loop_hz
        self._last_tick_time = now
        dt = min(max(dt, 1e-3), 0.2)

        # 看门狗：反馈超时
        if self._have_js and (now - self._last_js_time) > self._fb_timeout_s:
            self._fault(ERR_WATCHDOG, f"feedback stale > {self._fb_timeout_s:.2f}s")
            return

        # 超硬限：瞬时力矩越限
        hard = self._max_torque * self._overtorque_ratio
        if self._meas_torque > hard + 1e-6:
            self._fault(ERR_OVERTORQUE, f"torque {self._meas_torque:.2f} > hard {hard:.2f}Nm")
            return

        # 接触检测：力矩升到阈值前恒速软闭合；接触后才切入 PI
        contact_thresh = max(
            self._contact_detect_min, self._contact_detect_ratio * self._target_torque
        )
        if not self._contact_detected and self._meas_torque >= contact_thresh:
            self._contact_detected = True
            self._integral = 0.0

        e = self._target_torque - self._meas_torque  # >0 力不足
        if not self._contact_detected:
            # 接触前：恒速软闭合（不积分，避免接触瞬间积分饱和猛夹）
            dq = -self._v_close * dt
        else:
            # 接触后 PI，单向约束：力够了/超了绝不继续夹；超力按比例回退
            self._integral = max(
                -self._int_limit, min(self._int_limit, self._integral + e * dt)
            )
            if e > 0.0:
                v_close = self._kp * e + self._ki * max(self._integral, 0.0)
                dq = -min(max(v_close, 0.0), self._v_close) * dt
            else:
                back = self._kp * (-e)  # 超力 → 开口回退
                dq = min(back, self._v_open) * dt

        q_new = self._q_cmd + dq
        q_new = max(self._q_close, min(self._q_open, q_new))
        self._q_cmd = q_new
        self._publish_traj(q_new)

        # 接触 / 抓稳判定
        band = self._band_ratio * max(self._target_torque, 1e-3)
        if abs(e) <= band:
            if self._in_band_since <= 0:
                self._in_band_since = now
            elif now - self._in_band_since >= self._settle_s:
                if self._state != ST_GRASPED:
                    self.get_logger().info(
                        f"GRASPED target={self._target_torque:.2f} meas={self._meas_torque:.2f}"
                    )
                self._state = ST_GRASPED
                self._contact = True
        else:
            self._in_band_since = 0.0
            if self._state == ST_GRASPED:
                # 抓稳后滑脱：回到闭合中
                self._state = ST_FORCE_CLOSING
                self._contact = False

        # 抓取超时
        if self._state != ST_GRASPED and (now - self._force_start_time) > self._grasp_timeout_s:
            self._fault(
                ERR_GRASP_TIMEOUT,
                f"grasp timeout > {self._grasp_timeout_s:.1f}s (meas={self._meas_torque:.2f})",
            )

    # ---------- 状态发布 ----------
    def _publish_status_locked(self, now: float) -> None:
        # 力控期间 force_status_hz，否则 status_hz
        target_hz = float(self._p("force_status_hz")) if self._force_active else float(self._p("status_hz"))
        interval = 1.0 / target_hz
        if not hasattr(self, "_last_status_time"):
            self._last_status_time = 0.0
        if now - self._last_status_time < interval:
            return
        self._last_status_time = now

        msg = GripperStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self._state
        msg.mode = self._mode
        msg.target_torque_nm = float(self._target_torque)
        msg.actual_torque_nm = float(self._meas_torque)
        msg.position = float(self._norm_pos)
        msg.contact = bool(self._contact)
        msg.error_code = int(self._error_code)
        msg.message = str(self._message)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperControllerNode()
    # 固件力矩硬限由 3s 重试定时器与 force 启动时确保写入（服务可能晚于本节点就绪）
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()






