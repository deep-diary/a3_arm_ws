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
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

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

from a3_can_bridge.msg import MotorStates
from a3_can_bridge.srv import MotorCommand
from a3_msgs.msg import ArmStatus, MonitorStatus
from a3_msgs.srv import (
    GotoNamedPose,
    MoveToJointPositions,
    PlaybackTrajectory,
    SaveNamedPose,
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

# 编排状态机（F45：11 态）
STATE_IDLE = "IDLE"            # 上电未初始化
STATE_INIT = "INIT"
STATE_READY = "READY"
STATE_TRAJ = "TRAJ"
STATE_SERVO = "SERVO"
STATE_TEACH = "TEACH"
STATE_AI = "AI"
STATE_SAFE_PARK = "SAFE_PARK"  # F40: 回安全位中，拒绝新运动指令
STATE_DISABLED = "DISABLED"    # F45: 已失能，须显式 enable
STATE_COOLING = "COOLING"      # F44: 超温保护后降温中，enable 被拒直到降温
STATE_FAULT = "FAULT"

# 禁止运动类命令的底层控制模式
BLOCKED_MODES = {"ZERO_TORQUE", "SERVO", "GRAVITY_COMP"}


def _duration(sec: float) -> Duration:
    d = Duration()
    d.sec = int(math.floor(sec))
    d.nanosec = int(round((sec - d.sec) * 1e9))
    return d


def _smooth_points(
    points: List[JointTrajectoryPoint], window: int
) -> List[JointTrajectoryPoint]:
    """LL-047: 中心滑动平均低通（仅动 positions，保留 time_from_start）。

    手拖录制的 50Hz 轨迹天然带加速度尖峰（实测 L2 113→7 rad/s² @w=9），伺服忠实复现
    即"抖动"。对整条轨迹（ramp+录制）做中心滑动平均：凸组合不越出原始 min/max，
    不会把关节推出录制轨迹本身的包络；仅圆顺快速段尖角，停顿点几乎不动。
    """
    w = int(window)
    if w < 3:
        return points
    if w % 2 == 0:
        w += 1  # 取奇数保证窗口对称
    h = w // 2
    n = len(points)
    m = len(points[0].positions)
    # 逐关节独立平滑
    smoothed = []
    for k in range(m):
        seq = [p.positions[k] for p in points]
        out = [0.0] * n
        for i in range(n):
            lo = max(0, i - h)
            hi = min(n, i + h + 1)
            out[i] = sum(seq[lo:hi]) / (hi - lo)  # 边界自动缩窗，不拉飞端点
        smoothed.append(out)
    result: List[JointTrajectoryPoint] = []
    for i, pt in enumerate(points):
        q = JointTrajectoryPoint()
        q.positions = [smoothed[k][i] for k in range(m)]
        q.time_from_start = pt.time_from_start
        result.append(q)
    return result


def _sanitize_name(name: str) -> str:
    s = re.sub(r"[^A-Za-z0-9_-]", "_", (name or "").strip())
    return s or "trajectory"


def _traj_path_for(traj_dir: str, name: str) -> str:
    """F54: 空名 ≡ latest 槽位（save/playback 都读写 latest.yaml）；非空名 → {name}.yaml。"""
    if not (name or "").strip():
        return os.path.join(traj_dir, "latest.yaml")
    return os.path.join(traj_dir, f"{_sanitize_name(name)}.yaml")


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
        # F51/LL-039：消费看门狗确认故障（电机被带外失能 → 编排层不得停在 READY/TRAJ）
        self.declare_parameter("monitor_status_topic", "/a3/monitor/status")
        self.declare_parameter("unexpected_disable_guard", True)
        self.declare_parameter("unexpected_disable_sustain_s", 1.0)
        self.declare_parameter("gate_topic", "/power_sequence/gate_open")
        self.declare_parameter("power_state_topic", "/power_sequence/state")
        self.declare_parameter("traj_topic", "/joint_group_effort_controller/joint_trajectory")
        self.declare_parameter("status_hz", 10.0)
        self.declare_parameter("init_zero_tol_rad", 0.05)
        self.declare_parameter("init_timeout_s", 5.0)
        self.declare_parameter("require_gate", False)
        self.declare_parameter("goto_duration_s", 3.0)
        self.declare_parameter("goto_waypoints", 21)
        self.declare_parameter("playback_ramp_duration_s", 2.5)
        # LL-047: 回放低通平滑窗口（中心滑动平均，@50Hz 采样数）。7 点把录制 L2/L3 最大
        # 加速度 119/86 → ~8 rad/s²（-93%），几何扰动 ≤20 mrad；0/1 关闭。手拖录制天然带
        # 加速度尖峰，伺服忠实复现即"抖"——平滑压尖峰而非改路径。热设置回放前读取。
        self.declare_parameter("playback_smooth_samples", 7)
        # F54: 示教停止自动保存的最小样本数——start 后立刻 stop 的误触发（1~2 个样本）不许
        # 用退化单点文件覆盖上一条好的 latest。~0.2s 拖动即可超过（@50Hz 10 样本）。
        self.declare_parameter("teach_auto_save_min_samples", 10)
        self.declare_parameter("trajectories_dir", "~/.a3/trajectories")
        self.declare_parameter("named_poses_pkg", "a3_description")
        # F41: move_to/goto/ramp 兜底——最短时长 + ≥50Hz 插值点
        self.declare_parameter("move_to_min_duration_s", 3.0)
        self.declare_parameter("move_to_points_hz", 50.0)
        self.declare_parameter("move_to_max_points", 5000)
        # F40: 失能保护（不在 home 容差内先平滑回 home 再失能）
        self.declare_parameter("disable_home_pose_name", "home")
        self.declare_parameter("disable_home_tol_rad", 0.15)
        self.declare_parameter("disable_home_duration_s", 3.0)
        self.declare_parameter("disable_home_confirm_s", 0.5)
        self.declare_parameter("disable_park_timeout_s", 8.0)
        # F43: 最大力矩持久化
        self.declare_parameter("motor_states_topic", "/a3/motor/states")
        self.declare_parameter("torque_stats_file", "~/.a3/stats/torque_stats.yaml")
        self.declare_parameter("torque_stats_save_interval_s", 10.0)
        # F44: 温度管理（warn 仅告警；protect 自动回 home 失能降温；迟滞恢复）
        # 默认阈值 2026-09-13 调高：官方电机自带 130°C 保护兜底，初版 65°C 过低（LL-023）
        self.declare_parameter("temp_protect_enabled", True)
        self.declare_parameter("temp_warn_c", 90.0)
        self.declare_parameter("temp_protect_c", 95.0)
        self.declare_parameter("temp_hysteresis_c", 5.0)
        self.declare_parameter("fault_mask_reset_on_fault", True)
        # F48: 使能前读数限位门禁（环绕读数超限时拒绝使能，见 LL-019）
        self.declare_parameter("enable_position_check", True)
        # 限位比较裕量：set_zero 后编码器量化噪声 ±0.0002（L2/L3/L7 下界为 0），
        # 1e-6 不够；0.001 rad ≈ 0.057°，远小于环绕量 2π
        self.declare_parameter("position_check_margin_rad", 0.001)
        # /joint_states 最大陈旧时长：桥异常（refresh 停发）时 js 会冻结在旧值
        # （LL-020），旧值校验形同虚设——超过此时长视为不新鲜，拒绝使能
        self.declare_parameter("js_max_stale_s", 1.0)

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
        # F51/LL-039: 电机被带外失能（看门狗 stop/reset、外部直接调 /a3/motor/reset）
        self._pending_unexpected_disable = ""   # 待处置原因（空=无）
        self._all_disabled_since = 0.0          # 整臂失能起始（本地兜底持续窗）

        # 关节状态缓存
        self._positions: List[float] = [0.0] * self._n_joints
        self._velocities: List[float] = [0.0] * self._n_joints
        self._efforts: List[float] = [0.0] * self._n_joints
        self._have_js = False
        self._last_js_stamp = None  # F48: /joint_states 新鲜度检查（LL-020）
        # jog（滑动条直驱）进行中标志：区分 TRAJ 是 jog 还是 goto/playback
        self._jogging = False

        # F43: 每关节最大力矩统计（正负双向 + 绝对值，节流落盘，重启恢复）
        self._torque_stats: Dict[str, Dict[str, Any]] = self._load_torque_stats()
        self._torque_stats_dirty = False
        self._torque_stats_saved_at = 0.0
        # F44: 温度/故障监视（fresh 门控；无反馈温度=0.0，勿当 NaN 判读）
        self._temperatures: List[float] = [0.0] * self._n_joints
        self._temp_fresh: List[bool] = [False] * self._n_joints
        self._temp_warn = False
        self._temp_protect_pending = False
        self._fault_reset_pending = False
        self._pending_fault: Tuple[str, int] = ("", 0)

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
        # F43/F44: 电机原始状态（力矩统计 + 温度/故障监视）；发布端 SensorDataQoS，
        # best_effort 订阅兼容（js_qos 同）
        self.create_subscription(
            MotorStates, str(self.get_parameter("motor_states_topic").value), self._on_motor_states,
            js_qos, callback_group=self._cb_group,
        )
        # F51/LL-039: 看门狗状态（消费 UNEXPECTED_DISABLE——本节点在电机被带外失能后
        # 不得继续停在 READY/TRAJ 并保留陈旧保持目标）
        self.create_subscription(
            MonitorStatus, str(self.get_parameter("monitor_status_topic").value),
            self._on_monitor_status, 10, callback_group=self._cb_group,
        )

        # 发布
        self._arm_status_pub = self.create_publisher(ArmStatus, self._arm_status_topic, 10)
        self._traj_pub = self.create_publisher(JointTrajectory, self._traj_topic, 10)
        self._mode_pub = self.create_publisher(
            String, str(self.get_parameter("control_mode_topic").value), 10
        )
        # /a3/control_mode 是 VOLATILE 且仅状态变化时发布。本节点重启后须主动
        # 清掉其他节点（如夹爪力控互锁）锁存的旧 TRAJ_RUNNING：启动即发一次 +
        # 2 s 后重发一次（首次发布可能早于订阅发现、被静默丢弃）。
        self._publish_mode("IDLE")
        self._startup_mode_timer = self.create_timer(2.0, self._announce_startup_mode)

        # 服务（对外门面）
        self.create_service(Trigger, "/a3/arm/init", self._init_cb, callback_group=self._cb_group)
        self.create_service(Trigger, "/a3/arm/enable", self._enable_cb, callback_group=self._cb_group)
        self.create_service(Trigger, "/a3/arm/disable", self._disable_cb, callback_group=self._cb_group)
        self.create_service(GotoNamedPose, "/a3/arm/goto_named_pose", self._goto_cb, callback_group=self._cb_group)
        self.create_service(
            SetJointPositions, "/a3/arm/set_joint_positions", self._set_joint_positions_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            MoveToJointPositions, "/a3/arm/move_to", self._move_to_cb, callback_group=self._cb_group
        )
        self.create_service(
            SaveNamedPose, "/a3/arm/save_named_pose", self._save_named_pose_cb,
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
        poses: Dict[str, List[float]] = {}
        try:
            pkg = str(self.get_parameter("named_poses_pkg").value)
            share = get_package_share_directory(pkg)
            path = os.path.join(share, "config", "named_poses.yaml")
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            poses.update({
                name: list(spec["positions"])
                for name, spec in (data.get("poses") or {}).items()
            })
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cannot load named poses: {exc}")
        # F39: 用户层点位覆盖包内点位（save_named_pose 写入，同名覆盖）。
        # F39 落盘格式是 {poses: {name: [q...]}}（扁平列表），包级是 {name: {positions: [..]}}，
        # 两种都兼容，避免「list indices must be integers」误告警导致 home 点位回退全零。
        try:
            user_path = self._user_poses_path()
            if os.path.exists(user_path):
                with open(user_path, "r", encoding="utf-8") as f:
                    udata = yaml.safe_load(f) or {}
                for name, spec in (udata.get("poses") or {}).items():
                    poses[name] = (
                        list(spec["positions"]) if isinstance(spec, dict) else list(spec)
                    )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cannot load user named poses: {exc}")
        return poses

    def _user_poses_path(self) -> str:
        return os.path.expanduser("~/.a3/poses.yaml")

    # ------------------------------------------------------ F43 torque stats

    def _torque_stats_path(self) -> str:
        return os.path.expanduser(str(self.get_parameter("torque_stats_file").value))

    def _load_torque_stats(self) -> Dict[str, Dict[str, Any]]:
        """启动恢复 ~/.a3/stats/torque_stats.yaml（不存在则空）。"""
        path = self._torque_stats_path()
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                return {str(k): dict(v) for k, v in (data.get("joints") or {}).items()}
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"cannot load torque stats: {exc}")
        return {}

    def _save_torque_stats(self) -> None:
        path = self._torque_stats_path()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        try:
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump({"joints": self._torque_stats}, f)
            self._torque_stats_dirty = False
            self._torque_stats_saved_at = time.monotonic()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cannot save torque stats: {exc}")

    def _update_torque_stats(self, jn: str, tau: float) -> None:
        st = self._torque_stats.setdefault(
            jn, {"max_abs": 0.0, "max_pos": 0.0, "max_neg": 0.0, "ts": ""}
        )
        changed = False
        if tau > st["max_pos"]:
            st["max_pos"] = tau
            changed = True
        if tau < st["max_neg"]:
            st["max_neg"] = tau
            changed = True
        if abs(tau) > st["max_abs"]:
            st["max_abs"] = abs(tau)
            changed = True
        if changed:
            st["ts"] = datetime.now().astimezone().isoformat(timespec="seconds")
            self._torque_stats_dirty = True

    # ------------------------------------------------------------ F40 helpers

    def _home_pose(self) -> List[float]:
        """失能安全位：优先 poses.yaml 的 home（用户层覆盖包级），缺失回退全零。"""
        name = str(self.get_parameter("disable_home_pose_name").value)
        q = self._poses.get(name)
        if q is None:
            return [0.0] * self._n_joints
        q = list(q)
        if len(q) < self._n_joints:
            q += [0.0] * (self._n_joints - len(q))
        return [float(v) for v in q[: self._n_joints]]

    def _at_home(self, tol: float) -> Tuple[bool, float]:
        """全部关节 |q_i - home_i| ≤ tol 视为已在 home；返回 (是否, 最大偏差)。"""
        if not self._have_js:
            return False, float("inf")
        home = self._home_pose()
        worst = 0.0
        for p, h in zip(self._positions, home):
            worst = max(worst, abs(p - h))
        return worst <= tol, worst

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

    def _check_positions_in_limits(self) -> Tuple[bool, List[str]]:
        """F48: 使能前读数限位检查。

        返回 (全部在限位内, 违规描述列表)。未收到 /joint_states 或限位未加载
        也视为不通过——宁可拒绝使能也不盲使（LL-019 红线：断电多圈环绕读数
        超限时 kp×误差会瞬间猛拉）。
        """
        if not self._have_js:
            return False, ["no /joint_states received yet"]
        if not self._joint_limits:
            return False, ["joint limits not loaded from URDF"]
        # LL-020：桥异常时 js 冻结在旧值（stamp 陈旧），旧值校验形同虚设——
        # 必须检查消息新鲜度（桥正常时 50 Hz 发布，1 s 内必有新值）。
        if self._last_js_stamp is None:
            return False, ["no /joint_states received yet"]
        age = (self.get_clock().now() - self._last_js_stamp).nanoseconds * 1e-9
        if age > float(self.get_parameter("js_max_stale_s").value):
            return False, [f"stale /joint_states ({age:.1f}s old) — CAN feedback not flowing"]
        margin = float(self.get_parameter("position_check_margin_rad").value)
        viol: List[str] = []
        for jn, p in zip(self._joint_names, self._positions):
            lo_hi = self._joint_limits.get(jn)
            if lo_hi is None:
                viol.append(f"{jn} no limits in URDF")
                continue
            if p < lo_hi[0] - margin or p > lo_hi[1] + margin:
                viol.append(f"{jn}={p:+.4f} limit={lo_hi[0]:.4f}..{lo_hi[1]:.4f}")
        return not viol, viol

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

    def _announce_startup_mode(self) -> None:
        """启动 2 s 后重发当前状态模式，覆盖订阅发现窗口（一次性）。"""
        self._startup_mode_timer.cancel()
        self._publish_mode(self._state)

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
        # F45/F53: DISABLED/COOLING/FAULT 下运动命令被拒（原 IDLE 允许 move_to 语义混乱；
        # FAULT 电机已复位关断，发轨迹只被静默接受、臂不动）
        if self._state in (
            STATE_INIT, STATE_TEACH, STATE_AI, STATE_TRAJ, STATE_SERVO,
            STATE_SAFE_PARK, STATE_DISABLED, STATE_COOLING, STATE_FAULT,
        ):
            return False, f"state={self._state}"
        return True, ""

    def _traj_point_count(self, duration_s: float) -> int:
        """F41: 插值点数 = max(goto_waypoints, duration×move_to_points_hz)，上限防爆。"""
        n = max(
            int(self.get_parameter("goto_waypoints").value),
            min(
                int(math.ceil(duration_s * float(self.get_parameter("move_to_points_hz").value))),
                int(self.get_parameter("move_to_max_points").value),
            ),
        )
        return max(2, n)

    def _safe_park_then_disable(self) -> Tuple[bool, str]:
        """F40: 平滑回 home → 连续确认收敛 → 失能（同步阻塞，仿 _init_cb 轮询先例）。

        发布 home 轨迹抢占活跃轨迹（执行层 OnTrajectory 天然支持替换），
        SAFE_PARK 期间拒绝新运动指令；park 超时 → FAULT 且不 reset（保持使能，
        停在半途，需人工介入）。
        """
        home = self._home_pose()
        tol = float(self.get_parameter("disable_home_tol_rad").value)
        confirm_s = float(self.get_parameter("disable_home_confirm_s").value)
        timeout_s = float(self.get_parameter("disable_park_timeout_s").value)
        duration = max(
            float(self.get_parameter("disable_home_duration_s").value),
            float(self.get_parameter("move_to_min_duration_s").value),
        )
        n = self._traj_point_count(duration)

        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)
        q0 = list(self._positions)
        for i in range(n):
            alpha = i / (n - 1)
            pt = JointTrajectoryPoint()
            pt.positions = [a + alpha * (b - a) for a, b in zip(q0, home)]
            pt.time_from_start = _duration(duration * alpha)
            traj.points.append(pt)

        self._publish_mode("TRAJ_RUNNING")
        self._traj_pub.publish(traj)
        self._traj_done_at = 0.0  # 防旧 TRAJ 时间戳在 SAFE_PARK 中误触发回 READY
        self._set_state(STATE_SAFE_PARK, f"safe park -> home ({duration:.1f}s, {n} pts)")
        t0 = time.monotonic()

        converge_start = 0.0
        while time.monotonic() - t0 < duration + timeout_s and rclpy.ok():
            # F51/LL-039: 电机已带外失能 → park 不可能收敛，立即退出（保持“已失能”事实）
            if self._pending_unexpected_disable:
                why = self._pending_unexpected_disable
                self._pending_unexpected_disable = ""
                self._all_disabled_since = 0.0
                self._publish_mode("IDLE")
                self._set_state(STATE_DISABLED, f"safe park aborted：{why}")
                self.get_logger().error(f"[arm_controller] safe park aborted: {why}")
                return False, f"safe park aborted: {why}"
            at_home, _ = self._at_home(tol)
            if at_home:
                if converge_start == 0.0:
                    converge_start = time.monotonic()
                elif time.monotonic() - converge_start >= confirm_s:
                    break
            else:
                converge_start = 0.0
            time.sleep(0.05)

        at_home, _ = self._at_home(tol)
        if not at_home:
            self._set_state(STATE_FAULT, "safe park timeout: not at home, still enabled")
            return False, "safe park timeout: not at home, still enabled"
        ok, msg = self._motor_command(self._reset_cli, 2)
        if not ok:
            # reset 被拒（如 gate 互锁）：臂已在 home 位（安全），回 READY 待人工
            self._set_state(STATE_READY, f"parked at home but disable refused: {msg}")
            return False, f"{msg} (parked at home; stop power sequence first)"
        self._set_state(STATE_DISABLED, "safe park -> disabled")
        self._publish_mode("IDLE")
        return True, f"safe park -> disabled ({time.monotonic() - t0:.1f}s)"

    def _check_cooling(self) -> Tuple[bool, str]:
        """F44: COOLING 下重新使能门禁——全 fresh 关节 < protect−hysteresis 才放行。

        无 fresh 反馈的关节不阻碍（电机不在线则无从谈温度）。
        """
        limit = float(self.get_parameter("temp_protect_c").value) - float(
            self.get_parameter("temp_hysteresis_c").value
        )
        hot = [
            f"{jn}={t:.1f}" for jn, t, fr in zip(
                self._joint_names, self._temperatures, self._temp_fresh
            )
            if fr and math.isfinite(t) and t >= limit
        ]
        if hot:
            return False, f"cooling: {', '.join(hot)} >= {limit:.1f}C, wait"
        return True, ""

    def _trigger_temp_protect(self) -> None:
        """F44: 超温保护——平滑回 home → 失能 → COOLING（在状态发布节拍执行）。"""
        protect_c = float(self.get_parameter("temp_protect_c").value)
        hot = ", ".join(
            f"{jn}={t:.1f}" for jn, t, fr in zip(
                self._joint_names, self._temperatures, self._temp_fresh
            )
            if fr and math.isfinite(t) and t >= protect_c
        )
        if self._state in (STATE_IDLE, STATE_DISABLED, STATE_COOLING):
            self._set_state(STATE_COOLING, f"overtemp: {hot}, already disabled")
            return
        if self._state == STATE_SAFE_PARK:
            return  # F40 流程进行中，不打断（同目标）
        if not self._have_js:
            ok, msg = self._motor_command(self._reset_cli, 2)
            if ok:
                self._set_state(STATE_COOLING, f"overtemp: {hot}, disabled (no js)")
            else:
                self._set_state(STATE_FAULT, f"overtemp: {hot}, reset refused: {msg}")
            return
        ok, msg = self._safe_park_then_disable()
        if ok:
            self._set_state(STATE_COOLING, f"overtemp: {hot}, parked+disabled")
        else:
            # park 超时已置 FAULT；reset 被拒已回 READY——温度保护不可放弃 → FAULT
            self._set_state(STATE_FAULT, f"overtemp: {hot}, protect failed: {msg}")

    def _handle_motor_fault(self) -> None:
        """F44: 电机故障监视（fault_mask≠0，含固件过温锁存 bit3）→ 紧急失能 + FAULT。"""
        jn, mask = self._pending_fault
        self.get_logger().error(f"motor fault: joint={jn} mask=0x{mask:X} -> emergency reset")
        ok, msg = self._motor_command(self._reset_cli, 2)
        if ok:
            self._set_state(STATE_FAULT, f"motor fault: {jn} mask=0x{mask:X}, disabled")
        else:
            self._set_state(STATE_FAULT, f"motor fault: {jn} mask=0x{mask:X}, reset refused: {msg}")

    def _schedule_back_to_ready(self, delay_s: float) -> None:
        self._traj_done_at = time.monotonic() + max(delay_s, 0.1)

    def _back_to_ready(self) -> None:
        self._jogging = False
        if self._state == STATE_TRAJ:
            self._set_state(STATE_READY, "trajectory finished")
            # 轨迹结束必须发回非阻塞模式：此前只发 TRAJ_RUNNING（轨迹开始），
            # 真机上夹爪力控互锁会永久锁存 TRAJ_RUNNING，力控永远被拒。
            self._publish_mode("READY")

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
        # F48: 记录消息时间戳用于新鲜度检查（LL-020）
        self._last_js_stamp = self.get_clock().now()

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

    def _on_motor_states(self, msg: MotorStates) -> None:
        """F43/F44: 电机原始状态——最大力矩跟踪 + 温度/故障监视。

        无反馈关节温度在 C++ 侧被置 0.0（非 NaN），所有温度判断必须 fresh 门控，
        否则断连会被误判「已冷却」放行使能（LL-011 教训）。6J 臂 MotorStates 仍
        含 7 条（motor 7 fresh=false），按 motor_id-1 索引并越界保护。
        """
        n = self._n_joints
        temps = [0.0] * n
        fresh = [False] * n
        for st in msg.states:
            idx = int(st.motor_id) - 1
            if idx < 0 or idx >= n:
                continue
            jn = self._joint_names[idx]
            if st.fresh and math.isfinite(float(st.torque_nm)):
                self._update_torque_stats(jn, float(st.torque_nm))
            if st.fresh:
                temps[idx] = float(st.temperature_c)
                fresh[idx] = True
        self._temperatures = temps
        self._temp_fresh = fresh

        if not self.get_parameter("temp_protect_enabled").value:
            return

        # warn 级：仅告警 + 遥测，不动状态机
        warn_c = float(self.get_parameter("temp_warn_c").value)
        self._temp_warn = any(
            fr and math.isfinite(t) and t >= warn_c for t, fr in zip(temps, fresh)
        )

        # protect 级：pending 标记交给 _publish_status 节拍执行（避免订阅回调长阻塞）
        protect_c = float(self.get_parameter("temp_protect_c").value)
        if any(fr and math.isfinite(t) and t >= protect_c for t, fr in zip(temps, fresh)):
            if self._state in (STATE_READY, STATE_TRAJ, STATE_IDLE, STATE_DISABLED, STATE_COOLING):
                self._temp_protect_pending = True

        # 电机故障监视（fault_mask 非零，含固件过温锁存 bit3）
        if self.get_parameter("fault_mask_reset_on_fault").value and self._state in (
            STATE_READY, STATE_TRAJ, STATE_IDLE,
        ):
            for st in msg.states:
                if st.fresh and int(st.fault_mask) != 0:
                    idx = int(st.motor_id) - 1
                    jn = self._joint_names[idx] if 0 <= idx < n else f"motor{st.motor_id}"
                    self._pending_fault = (jn, int(st.fault_mask))
                    self._fault_reset_pending = True
                    break

        # F51/LL-039 本地兜底（不依赖看门狗在跑）：READY/TRAJ/SAFE_PARK 期间整臂报告
        # 「已失能」且持续 ≥ sustain → 电机被带外失能。持续窗用于抑制本节点自身
        # park→reset 的竞态（reset 后电机先报 mode 0、状态机随后才转 DISABLED）。
        if not self.get_parameter("unexpected_disable_guard").value:
            return
        if self._state not in (STATE_READY, STATE_TRAJ, STATE_SAFE_PARK):
            self._all_disabled_since = 0.0
            return
        fresh_motors = [st for st in msg.states if st.fresh]
        if fresh_motors and all(not st.enabled for st in fresh_motors):
            now = time.monotonic()
            if self._all_disabled_since == 0.0:
                self._all_disabled_since = now
            elif now - self._all_disabled_since >= float(
                    self.get_parameter("unexpected_disable_sustain_s").value):
                if not self._pending_unexpected_disable:
                    self._pending_unexpected_disable = (
                        f"电机带外失能（{len(fresh_motors)} 个 fresh 电机均报关闭，持续 "
                        f"{now - self._all_disabled_since:.1f}s）")
        else:
            self._all_disabled_since = 0.0

    def _on_monitor_status(self, msg: MonitorStatus) -> None:
        """F51/LL-039：消费看门狗「已确认」故障——只认 TRIGGERED（瞬时 PENDING 不动作）。"""
        if not self.get_parameter("unexpected_disable_guard").value:
            return
        if msg.status == "TRIGGERED" and msg.fault == "UNEXPECTED_DISABLE":
            if not self._pending_unexpected_disable:
                self._pending_unexpected_disable = "看门狗确认 UNEXPECTED_DISABLE"

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

        # F51/LL-039: 电机被带外失能 → 立刻离开 READY/TRAJ（不再保留保持目标与后续指令）
        if self._pending_unexpected_disable and self._state in (STATE_READY, STATE_TRAJ):
            why = self._pending_unexpected_disable
            self._pending_unexpected_disable = ""
            self._all_disabled_since = 0.0
            self._traj_done_at = 0.0
            self._jogging = False
            self._publish_mode("IDLE")
            self._set_state(
                STATE_DISABLED,
                f"{why} → DISABLED（恢复：/a3/arm/enable；执行层 F51 使能会重锚到当前位姿）")
            self.get_logger().error(f"[arm_controller] {why} → state=DISABLED")

        # F44: 超温/故障保护在状态节拍执行（订阅回调只置 pending 标记）
        if self._temp_protect_pending:
            self._temp_protect_pending = False
            self._trigger_temp_protect()
        if self._fault_reset_pending:
            self._fault_reset_pending = False
            self._handle_motor_fault()

        # F43: 力矩统计节流落盘
        if self._torque_stats_dirty and time.monotonic() - self._torque_stats_saved_at >= float(
            self.get_parameter("torque_stats_save_interval_s").value
        ):
            self._save_torque_stats()

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
        # F43/F44: 温度（无反馈 0.0）+ 历史最大力矩绝对值（无数据 0.0）
        st.temperatures = [float(t) for t in self._temperatures]
        st.max_torques = [
            float(self._torque_stats.get(jn, {}).get("max_abs", 0.0))
            for jn in self._joint_names
        ]
        st.temp_warn = bool(self._temp_warn)
        self._arm_status_pub.publish(st)

    # ------------------------------------------------------------------ services

    def _init_cb(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._state in (STATE_TRAJ, STATE_SERVO, STATE_TEACH, STATE_AI, STATE_SAFE_PARK):
            resp.success = False
            resp.message = f"busy in state={self._state}"
            return resp
        # F53/LL-045: 零力矩/重力补偿模式下 set_zero 会打碎零位帧，且执行层 enable 不
        # 退出 zero_torque（kp 仍 0）——init 变“伪成功”。拒绝并给还原路径。
        if self._mode in BLOCKED_MODES:
            resp.success = False
            resp.message = (
                f"mode={self._mode}: 先 /a3/zero_torque/stop 恢复闭环再 init"
            )
            return resp
        # F44: COOLING 下重新使能门禁（init 含 enable）
        if self._state == STATE_COOLING:
            can_cool, why = self._check_cooling()
            if not can_cool:
                resp.success = False
                resp.message = why
                return resp

        # F48: init 是恢复路径（set_zero 重建零位帧后再使能），限位检查只 WARN
        # 不阻断。环绕读数下 init 会把「当前位姿」定义为零位——仅当臂确实摆在
        # 已知位姿（如工装摆 URDF 零位）时才应这样恢复，否则位姿帧无意义。
        if self.get_parameter("enable_position_check").value:
            ok_lim, viol = self._check_positions_in_limits()
            if not ok_lim:
                self.get_logger().warn(
                    "init with readings out of URDF limits: " + "; ".join(viol)
                    + " — set_zero defines CURRENT pose as zero; only valid at a known pose"
                )

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
        if self._state in (STATE_TRAJ, STATE_SERVO, STATE_TEACH, STATE_AI, STATE_SAFE_PARK):
            resp.success = False
            resp.message = f"busy in state={self._state}"
            return resp
        # F53/LL-045: 零力矩/重力补偿下使能=执行层重锚当前位，但 zero_torque 仍生效
        # （kp 0, enable 不退出该模式）——臂继续保持悬浮，enable 成“伪成功”。拒绝。
        if self._mode in BLOCKED_MODES:
            resp.success = False
            resp.message = (
                f"mode={self._mode}: 先 /a3/zero_torque/stop 恢复闭环再 enable"
            )
            return resp
        # F44: COOLING 下重新使能门禁（降温到 protect−hysteresis 才放行）
        if self._state == STATE_COOLING:
            can_cool, why = self._check_cooling()
            if not can_cool:
                resp.success = False
                resp.message = why
                return resp
        # F48: 使能前读数限位门禁——环绕读数（断电多圈 +2π 推算，LL-019）超
        # URDF 限位时 kp×误差会瞬间猛拉，拒绝使能；恢复零位走 /a3/arm/init。
        if self.get_parameter("enable_position_check").value:
            ok_lim, viol = self._check_positions_in_limits()
            if not ok_lim:
                resp.success = False
                resp.message = (
                    "position check failed: " + "; ".join(viol)
                    + " (possible multi-turn wrap after power cycle — restore URDF"
                    " zero pose then /a3/arm/init)"
                )
                return resp
        ok, msg = self._motor_command(self._enable_cli, 1)
        if ok:
            self._set_state(STATE_READY, "enabled")
            # F51：执行层把目标重锚到当前反馈位（丢弃任何陈旧目标），摘要随响应返回
            self.get_logger().info(f"enable ok: {msg}")
        else:
            self.get_logger().error(f"enable refused: {msg}")
        resp.success = ok
        resp.message = msg
        return resp

    def _disable_cb(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        """F40: 失能保护——不在 home 容差内先平滑回 home 再失能，避免掉臂。

        服务语义：success=true ⟺ 已失能（或本就已失能）。拒绝时消息必含可执行下一步：
        · INIT/TEACH/SERVO/AI 状态 busy → 提示 /a3/motor/reset 紧急失能；
        · F53: mode∈BLOCKED_MODES（zero_torque/重力补偿）下执行层丢弃轨迹，safe park
        无法回家；且直接 reset 会撤重力补偿让臂垂落——提示先 /a3/zero_torque/stop。
        """
        if self._state in (STATE_INIT, STATE_TEACH, STATE_SERVO, STATE_AI):
            resp.success = False
            resp.message = f"busy in state={self._state} (use /a3/motor/reset for emergency)"
            return resp
        if self._state == STATE_SAFE_PARK:
            resp.success = False
            resp.message = "already safe parking"
            return resp
        if self._state in (STATE_DISABLED, STATE_COOLING):
            resp.success = True
            resp.message = "already disabled"
            return resp

        # F53/LL-045（主缺口）: 零力矩/重力补偿模式下安全区——执行层会丢弃轨迹，
        # safe park 的回家轨迹被静默丢弃，臂在悬浮中干等 ~4-5s 后看门狗 FOLLOW_STUCK
        # →stop→3s→reset 阶梯把臂中途切断（误报「电机带外失能」→DISABLED）。
        # 且此时 reset 会撤掉重力补偿让臂在重力下垂落。正确退出：zero_torque/stop
        # （先恢复 kp 锚定当前位）→ 再 disable；紧急失能才直接 /a3/motor/reset
        # （之后仍需 zero_torque/stop 清零标志，否则重使能后臂保持悬浮）。
        if self._mode in BLOCKED_MODES:
            resp.success = False
            resp.message = (
                f"mode={self._mode}: 先 /a3/zero_torque/stop 恢复闭环再 disable；"
                f"紧急失能 /a3/motor/reset（此后需 zero_torque/stop 才能正常重使能）"
            )
            return resp

        if not self._have_js:
            self.get_logger().warn("disable without /joint_states: resetting directly")
            ok, msg = self._motor_command(self._reset_cli, 2)
            if ok:
                self._set_state(STATE_DISABLED, "disabled (no js)")
                self._publish_mode("IDLE")
            resp.success = ok
            resp.message = msg
            return resp

        # IDLE（上电未使能）/FAULT：直达 reset；READY/TRAJ：home 内直达，否则先 park
        if self._state in (STATE_IDLE, STATE_FAULT):
            ok, msg = self._motor_command(self._reset_cli, 2)
            if ok:
                self._set_state(STATE_DISABLED, "disabled")
                self._publish_mode("IDLE")
            resp.success = ok
            resp.message = msg
            return resp

        if self.get_parameter("require_gate").value and not self._gate_open:
            resp.success = False
            resp.message = "gate closed: cannot safe park (open gate first)"
            return resp

        tol = float(self.get_parameter("disable_home_tol_rad").value)
        at_home, _ = self._at_home(tol)
        if at_home:
            ok, msg = self._motor_command(self._reset_cli, 2)
            if ok:
                self._set_state(STATE_DISABLED, "disabled")
                self._publish_mode("IDLE")
            resp.success = ok
            resp.message = msg
            return resp

        # TRAJ 中（运动/回放/jog）：home 轨迹抢占活跃轨迹（执行层 OnTrajectory 替换）
        ok, msg = self._safe_park_then_disable()
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
        elif len(q1) > self._n_joints:
            q1 = q1[: self._n_joints]
        q0 = list(self._positions)
        # F41: 时长下限 + ≥50Hz 插值点
        duration = min(max(
            float(self.get_parameter("goto_duration_s").value),
            float(self.get_parameter("move_to_min_duration_s").value),
        ), 60.0)
        n = self._traj_point_count(duration)

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

    def _move_to_cb(
        self, req: MoveToJointPositions.Request, resp: MoveToJointPositions.Response
    ) -> MoveToJointPositions.Response:
        """F39: 通用平滑移动——当前位姿到任意目标位姿，duration_s 内多点插值。"""
        if len(req.positions) != self._n_joints:
            resp.success = False
            resp.message = f"need {self._n_joints} positions, got {len(req.positions)}"
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

        q1 = [float(v) for v in req.positions]
        q0 = list(self._positions)
        # F41: 最短时长兜底（可配置，默认 3s）+ ≥50Hz 插值点（3s→150 点）
        duration = float(req.duration_s) if req.duration_s and req.duration_s > 0 else 1.0
        duration = min(max(duration, float(self.get_parameter("move_to_min_duration_s").value)), 60.0)
        n = self._traj_point_count(duration)

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
        self._set_state(STATE_TRAJ, f"move_to ({duration:.1f}s)")
        self._schedule_back_to_ready(duration + 0.3)

        resp.success = True
        resp.message = f"move_to ({duration:.1f}s, {n} pts)"
        return resp

    def _save_named_pose_cb(
        self, req: SaveNamedPose.Request, resp: SaveNamedPose.Response
    ) -> SaveNamedPose.Response:
        """F39: 保存命名点位（positions 留空 = 当前位姿）到 ~/.a3/poses.yaml 并即时生效。"""
        name = _sanitize_name(req.name)
        if not name:
            resp.success = False
            resp.message = "empty pose name"
            return resp
        if req.positions:
            if len(req.positions) != self._n_joints:
                resp.success = False
                resp.message = f"need {self._n_joints} positions, got {len(req.positions)}"
                return resp
            q = [float(v) for v in req.positions]
        else:
            if not self._have_js:
                resp.success = False
                resp.message = "no /joint_states yet"
                return resp
            q = [float(p) for p in self._positions]

        path = self._user_poses_path()
        data: Dict[str, Any] = {}
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
            except Exception as exc:  # noqa: BLE001
                resp.success = False
                resp.message = f"load existing poses failed: {exc}"
                return resp
        data.setdefault("poses", {})[name] = q
        os.makedirs(os.path.dirname(path), exist_ok=True)
        try:
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f)
        except Exception as exc:  # noqa: BLE001
            resp.success = False
            resp.message = f"save failed: {exc}"
            return resp

        self._poses[name] = q
        resp.success = True
        resp.message = f"saved pose '{name}'"
        resp.path = path
        self.get_logger().info(f"named pose saved: {name} -> {[round(v, 4) for v in q]}")
        return resp

    def _set_joint_positions_cb(
        self, req: SetJointPositions.Request, resp: SetJointPositions.Response
    ) -> SetJointPositions.Response:
        """Web 滑动条 jog 直驱：设 7 关节目标位置，短插值下发执行层（需求 F23 扩展）。"""
        if not self._have_js:
            resp.success = False
            resp.message = "no /joint_states yet"
            return resp
        if self._state in (
            STATE_INIT, STATE_TEACH, STATE_AI, STATE_FAULT, STATE_SERVO,
            STATE_SAFE_PARK, STATE_DISABLED, STATE_COOLING,
        ):
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
        n = len(self._record)
        # F54: 停止即自动保存 —— latest.yaml（滚动最新槽）+ 时间戳备份（防覆盖丢失）。
        # 样本 < teach_auto_save_min_samples 时认为是误触发（start 后立刻 stop），
        # 不写 latest、不覆盖上一条好的录制。
        min_samples = int(self.get_parameter("teach_auto_save_min_samples").value)
        auto_msgs: List[str] = []
        if n >= min_samples:
            latest = _traj_path_for(self._traj_dir, "")
            if self._save_recording_to(latest):
                auto_msgs.append("auto-saved latest.yaml")
                ts = time.strftime("%Y%m%d_%H%M%S")
                hist = os.path.join(self._traj_dir, f"teach_{ts}.yaml")
                if self._save_recording_to(hist):
                    auto_msgs.append(f"backup teach_{ts}.yaml")
            else:
                auto_msgs.append("auto-save FAILED (see log)")
        else:
            auto_msgs.append(
                f"auto-save skipped ({n} samples < {min_samples}, keep previous latest)"
            )
        self._set_state(STATE_READY, f"teach stopped ({n} samples)")
        resp.success = True
        resp.message = f"recorded {n} samples; " + ", ".join(auto_msgs) + (
            f" (zero_torque/stop: {msg})" if not ok else ""
        )
        return resp

    def _dump_recording(self) -> dict:
        """F54: 把当前内存录制(self._record: (t, pos) 列表)序列化为磁盘 YAML 数据。"""
        data = {
            "joint_names": list(self._joint_names),
            "points": [
                {
                    "positions": [float(p) for p in pos],
                    "time_from_start_sec": float(t),
                }
                for t, pos in self._record
            ],
        }
        return data

    def _save_recording_to(self, path: str) -> bool:
        """把当前录制写到 path，返回是否成功（异常已记录日志，不抛）。"""
        try:
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(self._dump_recording(), f)
            self.get_logger().info(f"trajectory saved to {path}")
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"trajectory save failed to {path}: {exc}")
            return False

    def _save_cb(
        self, req: SaveTrajectory.Request, resp: SaveTrajectory.Response
    ) -> SaveTrajectory.Response:
        path = _traj_path_for(self._traj_dir, req.name)
        if not self._record:
            resp.success = False
            resp.message = "no recording to save"
            return resp
        if not self._save_recording_to(path):
            resp.success = False
            resp.message = f"save failed: {path}"
            return resp
        resp.success = True
        resp.message = f"saved {len(self._record)} pts"
        resp.path = path
        return resp

    def _playback_cb(
        self, req: PlaybackTrajectory.Request, resp: PlaybackTrajectory.Response
    ) -> PlaybackTrajectory.Response:
        # F54: 空名 = 回放 latest 槽（分支在 sanitize 之前，_sanitize_name("")→"trajectory" 会撞纸面名）
        is_latest = not (req.name or "").strip()
        label = "latest" if is_latest else _sanitize_name(req.name)
        path = _traj_path_for(self._traj_dir, req.name)
        if not os.path.exists(path):
            if is_latest:
                resp.success = False
                resp.message = (
                    "no latest trajectory yet —— 先 start_teach → 拖动 → stop_teach 录制,"
                    "或显式指定 name (如 teach_jog2)"
                )
            else:
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

        # F38: 回放前先从当前位姿插值到首记录点（playback_ramp_duration_s），
        # 避免回放起始位 ≠ 记录起始位时机械臂突然跳变；随后原样回放记录轨迹。
        ramp_s = float(self.get_parameter("playback_ramp_duration_s").value)
        if ramp_s > 0.05 and self._have_js:
            q1 = [float(v) for v in traj.points[0].positions]
            q0: List[float] = []
            for jn in traj.joint_names:
                if jn in self._joint_names:
                    q0.append(float(self._positions[self._joint_names.index(jn)]))
                elif len(q0) < len(q1):
                    q0.append(q1[len(q0)])  # 轨迹含未知关节名：该关节不插值，取首点值
                else:
                    q0.append(0.0)
            q0 = q0[: len(q1)] + q1[len(q0):]
            # F41: ramp 段同步 ≥50Hz（2.5s→125 点）
            n = self._traj_point_count(ramp_s)
            ramp_pts: List[JointTrajectoryPoint] = []
            for i in range(n):
                alpha = i / (n - 1)
                pt = JointTrajectoryPoint()
                pt.positions = [a + alpha * (b - a) for a, b in zip(q0, q1)]
                pt.time_from_start = _duration(ramp_s * alpha)
                ramp_pts.append(pt)
            for pt in traj.points:
                t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                pt.time_from_start = _duration(t + ramp_s)
            traj.points = ramp_pts + traj.points
            self.get_logger().info(
                f"playback ramp: {ramp_s:.2f}s from current pose to first recorded "
                f"point ({n} pts)"
            )

        smooth_s = int(self.get_parameter("playback_smooth_samples").value)
        if smooth_s >= 3 and len(traj.points) >= 4:
            traj.points = _smooth_points(traj.points, smooth_s)
            self.get_logger().info(
                f"playback smooth: {smooth_s}-pt moving avg applied "
                f"({len(traj.points)} pts)"
            )

        duration = (
            traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9
        )
        self._publish_mode("TRAJ_RUNNING")
        self._traj_pub.publish(traj)
        self._set_state(STATE_TRAJ, f"playback {label}")
        self._schedule_back_to_ready(duration + 0.3)

        resp.success = True
        resp.message = f"playback {label} ({len(traj.points)} pts, {duration:.1f}s)"
        return resp

    def _enter_ai_cb(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._state in (STATE_TRAJ, STATE_SERVO, STATE_TEACH, STATE_AI):
            resp.success = False
            resp.message = f"busy in state={self._state}"
            return resp
        # F53/LL-045: 零力矩/重力补偿下 AI 发的轨迹会被执行层丢弃（静默无动作）
        # ——进入 AI 只会让命令石沉大海。先退出模式再进。
        if self._mode in BLOCKED_MODES:
            resp.success = False
            resp.message = (
                f"mode={self._mode}: 先 /a3/zero_torque/stop 恢复闭环再 enter_ai"
            )
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
        # F43: 退出前尽力落盘最大力矩统计
        node._save_torque_stats()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
