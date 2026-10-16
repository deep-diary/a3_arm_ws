#!/usr/bin/env python3
"""
A3 arm orchestration layer (facade).

统一对外交互门面：状态机、电机初始化闭环、使能/失能、预设点、状态聚合、
示教录制/回放/保存、AI 模式、模式仲裁。底层能力全部通过调用现有服务/话题复用，
不重写 CAN 编解码/插值/规划（需求 F21）。
"""

from __future__ import annotations

import bisect
import collections
import math
import os
import random
import re
import shutil
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Deque, Dict, List, Optional, Tuple

import rclpy
import yaml
try:  # F129: LibYAML C loader —— 1365 点回放文件 ~0.17s vs 纯 Python SafeLoader ~1s（满载节点内 3–4s）
    from yaml import CSafeLoader as _YamlSafeLoader
except ImportError:  # pragma: no cover - 无 libyaml 的环境退回纯 Python
    from yaml import SafeLoader as _YamlSafeLoader
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers, SetHardwareComponentState, SwitchController
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosbag2_interfaces.srv import Snapshot
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup, MoveGroupSequence
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MotionSequenceItem,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive

from a3_can_bridge.msg import MotorStates
from a3_can_bridge.srv import MotorCommand
from a3_msgs.msg import ArmStatus, MonitorStatus, TorqueComponents
from a3_msgs.srv import (
    GotoNamedPose,
    MotorIdCommand,
    MoveToJointPositions,
    PlaybackTrajectory,
    RandomPoseTour,
    RetimeTrajectory,
    SaveNamedPose,
    SaveTrajectory,
    SetJointPositions,
    SetPayload,
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

try:  # F174: 系统级模型配置（可选依赖；缺失时回退到 a3_description）
    from a3_robot_model import robot_model as _robot_model
except Exception:  # noqa: BLE001  (包未安装/未 source)
    _robot_model = None


def model_description_package() -> str:
    """默认描述包名（系统级模型配置 a3_robot_model，F174）。"""
    if _robot_model is not None:
        try:
            return str(_robot_model.load()["description_package"])
        except Exception:  # noqa: BLE001
            pass
    return "a3_description"


def model_urdf_path() -> str:
    """静态 URDF 路径（系统级模型配置；缺失时回退 a3_description）。"""
    if _robot_model is not None:
        try:
            return str(_robot_model.urdf_path())
        except Exception:  # noqa: BLE001
            pass
    return os.path.join(
        get_package_share_directory("a3_description"), "urdf", "el_a3.urdf"
    )


def model_include_gripper() -> bool:
    """是否含夹爪关节（系统级模型配置；缺失时按 7 轴 True 回退）。"""
    if _robot_model is not None:
        try:
            return bool(_robot_model.include_gripper())
        except Exception:  # noqa: BLE001
            pass
    return True


# 编排状态机（F45：11 态）
STATE_IDLE = "IDLE"            # 上电未初始化
STATE_INIT = "INIT"
STATE_READY = "READY"
STATE_TRAJ = "TRAJ"
STATE_SERVO = "SERVO"
STATE_TEACH = "TEACH"
STATE_WAYPOINT_TEACH = "WAYPOINT_TEACH"  # F131: 路点示教（L2 打点；Options 结束）
STATE_AI = "AI"
STATE_SAFE_PARK = "SAFE_PARK"  # F40: 回安全位中，拒绝新运动指令
STATE_DISABLED = "DISABLED"    # F45: 已失能，须显式 enable
STATE_COOLING = "COOLING"      # F44: 超温保护后降温中，enable 被拒直到降温
STATE_FAULT = "FAULT"

# EL05 电机 fault_mask（= 原始 fault_code，can_id bit16-21）位定义：
#   bit0=欠压 bit1=驱动 bit2=过温 bit3=磁编码 bit4=堵转过载 bit5=未标定
FAULT_UNDERVOLTAGE = 0x01
FAULT_DRIVER = 0x02
FAULT_OVERTEMP = 0x04
FAULT_MAGNET = 0x08
FAULT_STALL = 0x10
FAULT_UNCALIBRATED = 0x20
# 硬故障（立即 emergency reset + FAULT）：驱动 / 磁编码 / 堵转过载
FAULT_HARD_MASK = FAULT_DRIVER | FAULT_MAGNET | FAULT_STALL
# 软故障（计时 → safe park → disable，可自恢复不 reset）：欠压 / 过温 / 未标定
# 过温主路径仍是 F44 温度值保护（safe park → COOLING）；此处兜底 fault 位单独置位。
FAULT_SOFT_MASK = FAULT_UNDERVOLTAGE | FAULT_OVERTEMP | FAULT_UNCALIBRATED

# 禁止运动类命令的底层控制模式
BLOCKED_MODES = {"ZERO_TORQUE", "SERVO", "GRAVITY_COMP"}


def _duration(sec: float) -> Duration:
    d = Duration()
    d.sec = int(math.floor(sec))
    d.nanosec = int(round((sec - d.sec) * 1e9))
    return d


def _tfs_seconds(tfs: Duration) -> float:
    return float(tfs.sec) + tfs.nanosec * 1e-9


def _interp_series(values: List[float], times: List[float], t: float) -> float:
    """对严格递增时间序列做线性插值（用于 L7 重采样到 retime 时间轴写回）。"""
    if not values:
        return 0.0
    if t <= times[0]:
        return float(values[0])
    if t >= times[-1]:
        return float(values[-1])
    i = bisect.bisect_right(times, t) - 1
    t0, t1 = times[i], times[i + 1]
    if t1 <= t0:
        return float(values[i])
    f = (t - t0) / (t1 - t0)
    return values[i] + (values[i + 1] - values[i]) * f


def _geometric_smooth_positions(
    positions: List[List[float]], times: List[float], eps: float
) -> List[List[float]]:
    """
    F137: 逐关节五次 B 样条几何去噪（平滑因子 s = N·ε²，RMS 残差 ≈ ε）。

    去除录制点的编码器量化锯齿（几何噪声），位置包络不变、时间轴不动。ε>0 启用，
    典型 0.005（保形）~0.02（更平滑）。scipy 缺失/拟合失败时逐关节退化为原值，
    保证回放链路不被去噪卡死。
    """
    if eps <= 0 or len(positions) < 6:
        return list(positions)
    try:
        import numpy as np
        from scipy.interpolate import UnivariateSpline
    except Exception:
        return list(positions)
    raw = np.asarray(positions, dtype=float)
    n, m = raw.shape
    t0, t1 = times[0], times[-1]
    span = (t1 - t0) if t1 > t0 else 1.0
    u = np.asarray([(t - t0) / span for t in times], dtype=float)
    s = float(n) * eps * eps
    out = np.empty_like(raw)
    for j in range(m):
        try:
            out[:, j] = UnivariateSpline(u, raw[:, j], k=5, s=s)(u)
        except Exception:
            out[:, j] = raw[:, j]
    return [list(row) for row in out]


def _interpolate_via_spline(
    via_points: List[List[float]], resample_n: int
) -> Optional[List[List[float]]]:
    """
    F161: 稀疏 via 点 → 三次（cubic）过点样条，C2 稠密路径。

    首点为当前位、随后为各目标点（每点 L1–L6）。逐关节按累计弦长参数化做
    `make_interp_spline(k=3)` 三次样条插值——C2 连续（加速度连续，不震动），
    且过冲远小于 quintic（Runge 振荡随阶数加剧，真机实测 quintic L2 过冲 9 rad、
    cubic 1.3 rad）。过冲由调用方 `_spline_bounds_violation` 兜底（>0.3 rad 或越限
    则回退 blend/per-leg）。n<4 时降阶（k=n-1），仍优于 PCHIP 的 C1 硬拐。
    """
    import numpy as np
    from scipy.interpolate import make_interp_spline

    via = np.asarray(via_points, dtype=float)
    n, m = via.shape
    if n < 3 or m < 1:
        return None
    # 去掉相邻重复点（零弦长会让插值报错；固定序列可能显式重复）
    keep = [0]
    for i in range(1, n):
        if float(np.max(np.abs(via[i] - via[i - 1]))) > 1e-9:
            keep.append(i)
    via = via[keep]
    n = len(via)
    if n < 3:
        return None

    seg = np.linalg.norm(np.diff(via, axis=0), axis=1)
    u = np.concatenate([[0.0], np.cumsum(seg)])
    if float(u[-1]) <= 0.0:
        u = np.linspace(0.0, 1.0, n)
    else:
        u = u / u[-1]
    k = min(3, n - 1)  # cubic（C2），n<4 降阶
    grid = np.unique(np.concatenate([np.linspace(0.0, 1.0, resample_n), u]))
    out = np.empty((len(grid), m))
    try:
        for j in range(m):
            out[:, j] = make_interp_spline(u, via[:, j], k=k)(grid)
    except Exception:
        return None
    return [list(row) for row in out]


def _linear_dense(via_points: List[List[float]], resample_n: int = 200) -> Optional[List[List[float]]]:
    """稀疏 via 点 → 直线段连接 + 按段均匀重采样的稠密路径（保形，不过冲）。

    L1 保形/L2 梯形用（与 cubic 样条的过冲/绕路区分）；线性插值 dq/ds 在拐角不连续，
    但几何不过冲（真机 cubic 样条过冲达 9 rad，线性保形更稳）。
    """
    import numpy as np

    via = np.asarray(via_points, dtype=float)
    n, m = via.shape
    if n < 2 or m < 1:
        return None
    per = max(2, int(resample_n / (n - 1)))
    out: List[List[float]] = [list(via[0])]
    for i in range(n - 1):
        for k in range(1, per + 1):
            out.append(list(via[i] + (via[i + 1] - via[i]) * (k / per)))
    return out


def _round_corners(
    via_points: List[List[float]],
    blend_r: "float | List[float]",
    samples_per_corner: int = 20,
) -> Optional[List[List[float]]]:
    """F165/F166 工业 G64 连续路径：关节空间折线尖角 → 二次 Bezier 圆角（C1 连续）。

    保留直线段（保形、不过冲），仅在内部顶点 V_i 切角：进入点 P_in = V_i - r·d_in、
    离开点 P_out = V_i + r·d_out，两点间用控制点 V_i 的二次 Bezier B(t) 过渡——B 在
    P_in/P_out 处切线分别沿 d_in/d_out（速度方向连续），消除折线尖角导致的加速度尖峰。
    blend_r：标量（所有内部顶点统一半径）或**逐顶点列表**（长度 = 顶点数；F166 逐点 exact：
    某顶点半径 0 → 该点**精确过点**不做圆角）。r_i = min(blend_r_i, 0.45·min(前后段长))，
    与 pilz 自适应 blend 同口径。全 0 时原样返回。返回稀疏路径（直线段 2 端点 + 拐角密集）。
    """
    import numpy as np

    via = np.asarray(via_points, dtype=float)
    n, m = via.shape
    if n < 2 or m < 1:
        return None
    if isinstance(blend_r, (list, tuple)):
        rb = [float(x) for x in blend_r]
        if len(rb) < n:
            rb = rb + [0.0] * (n - len(rb))
    else:
        rb = [float(blend_r)] * n
    if max(rb) <= 1e-6:
        return [list(p) for p in via]

    samples = max(2, int(samples_per_corner))
    out: List[List[float]] = [list(via[0])]
    for i in range(1, n - 1):
        v_in = via[i] - via[i - 1]
        v_out = via[i + 1] - via[i]
        l_in = float(np.linalg.norm(v_in))
        l_out = float(np.linalg.norm(v_out))
        if l_in < 1e-9 or l_out < 1e-9:
            out.append(list(via[i]))
            continue
        r = min(rb[i], 0.45 * min(l_in, l_out))
        if r < 1e-6:
            out.append(list(via[i]))  # 半径 0（或段太短）→ 精确过点、不切角
            continue
        d_in = v_in / l_in
        d_out = v_out / l_out
        p_in = via[i] - r * d_in
        p_out = via[i] + r * d_out
        for k in range(samples + 1):
            t = k / samples
            b = (1.0 - t) ** 2 * p_in + 2.0 * (1.0 - t) * t * via[i] + t ** 2 * p_out
            out.append(list(b))
    out.append(list(via[n - 1]))
    # 去相邻重复点（固定序列可能显式重复路点；零长段会让弧长 s 出现相等相邻值，
    # 使 _arc_eval 的 np.interp / np.gradient 产生伪尖峰）。
    dedup: List[List[float]] = [out[0]]
    for p in out[1:]:
        if float(np.max(np.abs(np.asarray(p) - np.asarray(dedup[-1])))) > 1e-9:
            dedup.append(p)
    return dedup


def _approx_spline(
    via_points: List[List[float]], resample_n: int, ctrl_ratio: float = 0.7
) -> Optional[List[List[float]]]:
    """F166 逼近样条：夹紧端点的最小二乘 B 样条（端点精确插值、中间逼近不过点）。

    与 `_interpolate_via_spline`（插值样条，**精确过所有点**、可能过冲）相对——本函数用
    scipy `make_lsq_spline`（clamped 结点向量）拟合：控制点数 `nc < 数据点数 n`（由
    ctrl_ratio 决定，下限 k+1=4），故为**最小二乘逼近**（中间不要求过点，更平滑、过冲小）；
    再把首、末控制点强制设为首、末端点 → **首末精确**。n 越多逼近效果越明显（点少时逼近≈插值）。
    n<4 或退化返回 None。
    """
    import numpy as np
    from scipy.interpolate import make_lsq_spline

    via = np.asarray(via_points, dtype=float)
    n, m = via.shape
    if n < 4 or m < 1:
        return None
    # 去相邻重复点
    keep = [0]
    for i in range(1, n):
        if float(np.max(np.abs(via[i] - via[i - 1]))) > 1e-9:
            keep.append(i)
    via = via[keep]
    n = len(via)
    if n < 4:
        return None
    # 弦长参数化
    seg = np.linalg.norm(np.diff(via, axis=0), axis=1)
    u = np.concatenate([[0.0], np.cumsum(seg)])
    if float(u[-1]) <= 0.0:
        u = np.linspace(0.0, 1.0, n)
    else:
        u = u / u[-1]
    k = 3
    nc = max(k + 1, min(n, int(round(n * ctrl_ratio))))  # 控制点数 < n → 逼近（下限 4）
    n_int = max(0, nc - k - 1)  # 内部结点数
    interior = np.linspace(u[0], u[-1], n_int + 2)[1:-1] if n_int > 0 else np.array([])
    t = np.concatenate([[u[0]] * (k + 1), interior, [u[-1]] * (k + 1)])
    out = np.empty((resample_n, m))
    grid = np.linspace(0.0, 1.0, resample_n)
    try:
        for j in range(m):
            spl = make_lsq_spline(u, via[:, j], t, k=k)
            # clamped 首末控制点 = 首末端点 → 曲线首末精确（中间为最小二乘逼近）
            spl.c[0] = via[0, j]
            spl.c[-1] = via[-1, j]
            out[:, j] = spl(grid)
    except Exception:
        return None
    # 采样端对齐到原端点，消除数值残差
    out[0, :] = via[0]
    out[-1, :] = via[-1]
    return [list(row) for row in out]


# ═══════════════════════════════════════════════════════════════════════════
# F166: 轨迹模式注册表（统一命名 + 抽象调度）
#   canonical 规范名 / aliases 兼容旧名 / space 空间 / kind 执行机制
#   / profile 软件平滑剖面 / exact 精确过点能力 / stop_each 是否逐点停
# ═══════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class TrajModeSpec:
    canonical: str
    aliases: Tuple[str, ...]
    space: str      # "joint" | "cartesian"
    kind: str       # "ptp_legs"|"pilz_ptp_seq"|"pilz_lin_seq"|"lin_single"|"circ_single"|"soft_smooth"
    profile: str    # soft_smooth 用: "l1p5"|"l2"|"l3"|"l3_fit"
    exact: str      # "all" | "per_point" | "none"
    stop_each: bool
    note: str = ""


_TRAJ_MODE_LIST: Tuple[TrajModeSpec, ...] = (
    TrajModeSpec("ptp", (), "joint", "ptp_legs", "", "all", True,
                 "关节 PTP 逐腿（OMPL+TOTG，逐点精确停）"),
    TrajModeSpec("ptp_blend", ("ptp_seq",), "joint", "pilz_ptp_seq", "", "per_point", False,
                 "关节 PTP Sequence + blend（不停车，切角）"),
    TrajModeSpec("lin_exact", ("pilz",), "cartesian", "lin_single", "", "all", True,
                 "笛卡尔 LIN（pilz，blend=0 逐点精确停）"),
    TrajModeSpec("lin_blend", ("lin", "blend"), "cartesian", "pilz_lin_seq", "", "per_point", False,
                 "笛卡尔 LIN Sequence + blend（不停车，切角）"),
    TrajModeSpec("jnt_g64_quintic", ("l1p5",), "joint", "soft_smooth", "l1p5", "per_point", False,
                 "G64 拐角圆滑 + quintic 加-巡-减"),
    TrajModeSpec("jnt_g64_trapezoid", ("l2",), "joint", "soft_smooth", "l2", "per_point", False,
                 "G64 拐角圆滑 + 梯形速度"),
    TrajModeSpec("jnt_spline_bell", ("l3",), "joint", "soft_smooth", "l3", "all", False,
                 "三次插值样条 + 钟形（精确过点、点间可能过冲）"),
    TrajModeSpec("circ", (), "cartesian", "circ_single", "", "all", False,
                 "pilz CIRC 圆弧（3 点定弧，弧过中间点）"),
    TrajModeSpec("jnt_spline_approx", ("l3_fit",), "joint", "soft_smooth", "l3_fit", "none", False,
                 "夹紧端点最小二乘 B 样条（首末精确、中间逼近不过点）"),
)

TRAJ_MODES: Dict[str, TrajModeSpec] = {s.canonical: s for s in _TRAJ_MODE_LIST}
_MODE_ALIAS: Dict[str, str] = {}
for _s in _TRAJ_MODE_LIST:
    _MODE_ALIAS[_s.canonical] = _s.canonical
    for _a in _s.aliases:
        _MODE_ALIAS[_a] = _s.canonical


def resolve_traj_mode(name: str) -> Optional[TrajModeSpec]:
    """规范名 / 别名（大小写不敏感）→ TrajModeSpec；未知返回 None。"""
    key = str(name or "").strip().lower()
    return TRAJ_MODES.get(_MODE_ALIAS.get(key, ""))


def mode_exact_ok(spec: TrajModeSpec, need_exact_pattern: List[bool]) -> bool:
    """F166: 给定内部点的 exact 需求模式，判断该模式是否可行。

    need_exact_pattern[i]=True 表示第 i 个内部 via 点必须精确过。规则：
      * 无需任何精确点 → 任何模式可行；
      * 全部需要精确 → exact ∈ {all, per_point} 可行（none 不可行）；
      * 混合 → 仅 per_point 可行（all 要求每点都过、none 都不过，均不满足）。
    """
    if not any(need_exact_pattern):
        return True
    if all(need_exact_pattern):
        return spec.exact in ("all", "per_point")
    return spec.exact == "per_point"




def _jerk_limited_timed_traj(
    dense: List[List[float]], vmax: float, amax: float
) -> Optional[Tuple[List[float], List[List[float]]]]:
    """
    F161: C2 样条稠密路径 → quintic 加速 + 巡航 + quintic 减速重定时。

    沿弧长：quintic 加速（0→vmax，jerk 连续）→ 巡航 vmax → quintic 减速（对称）。
    有巡航段故平均速度接近 vmax（比单段 quintic 快 ~1.8 倍），加速/减速段 jerk 连续
    （优于 S-curve 的 jerk 阶跃）。缩放循环通过降巡航速度确保峰值速度/加速度 ≤
    vmax/amax（沿弧长样条 dq/ds 过冲时自动放慢）。返回 (times, positions)，退化返回 None。
    """
    import numpy as np
    from scipy.interpolate import make_interp_spline

    q = np.asarray(dense, dtype=float)
    n, m = q.shape
    if n < 3 or m < 1:
        return None
    seg = np.linalg.norm(np.diff(q, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    S = float(s[-1])
    if S <= 1e-9:
        return None
    vmax = max(vmax, 1e-3)
    amax = max(amax, 1e-3)
    # 线性插值（不用 cubic 样条）：稠密点下 dq/ds ≤ 1 天然成立；cubic 样条 q(s)
    # 会导数过冲（dq/ds 达 2.5 倍），导致缩放循环把巡航速度压到极低、时长爆炸。
    def _arc_eval(s_t: np.ndarray) -> np.ndarray:
        return np.column_stack([np.interp(s_t, s, q[:, j]) for j in range(m)])

    def _quintic_blend(v: float) -> Tuple[float, float]:
        # quintic 0→v：峰值加速度 7.5v/T_a，位移 0.5v·T_a（平均速度 0.5v）
        t_a = 7.5 * v / amax
        return t_a, 0.5 * v * t_a

    def _profile(S: float, v: float):
        t_a, s_a = _quintic_blend(v)
        if 2.0 * s_a >= S:
            lo, hi = 0.0, v
            for _ in range(60):
                mid = (lo + hi) / 2.0
                _, sa = _quintic_blend(mid)
                if 2.0 * sa >= S:
                    hi = mid
                else:
                    lo = mid
            v = (lo + hi) / 2.0
            t_a, s_a = _quintic_blend(v)
            s_c = 0.0
        else:
            s_c = S - 2.0 * s_a
        t_c = s_c / v if v > 0.0 else 0.0
        return 2.0 * t_a + t_c, t_a, s_a, s_c, t_c, v

    v_u = vmax
    for _ in range(12):
        T, t_a, s_a, s_c, t_c, v_u = _profile(S, v_u)
        dt = 0.01
        n_t = max(40, int(round(T / dt)) + 1)
        t = np.linspace(0.0, T, n_t)
        s_t = np.zeros(n_t)
        for i, tt in enumerate(t):
            if tt < t_a:
                u = tt / t_a
                s_t[i] = s_a * (2.0 * u ** 6 - 6.0 * u ** 5 + 5.0 * u ** 4)
            elif tt < t_a + t_c:
                s_t[i] = s_a + v_u * (tt - t_a)
            else:
                u = (tt - t_a - t_c) / t_a
                # 减速段须镜像加速段：s = (s_a+s_c) + s_a·(1 - f(1-u))，起点速度 = v（巡航
                # 速度）、终点 0。原实现复用加速段 f(u)（f'(0)=0 → 起点速度 0），造成巡航→
                # 减速处速度从 v 突降到 0 的假加速度尖峰（≈v/(2dt)≈50v），使缩放循环把 v
                # 压到 ~amax/50、整条轨迹时长爆炸（F165 修复）。
                w = 1.0 - u
                s_t[i] = s_a + s_c + s_a * (
                    1.0 - (2.0 * w ** 6 - 6.0 * w ** 5 + 5.0 * w ** 4))
        s_t[-1] = S
        q_t = _arc_eval(np.clip(s_t, 0.0, S))
        vel = np.gradient(q_t, t, axis=0)
        acc = np.gradient(vel, t, axis=0)
        vpeak = float(np.max(np.abs(vel)))
        apeak = float(np.max(np.abs(acc)))
        if vpeak <= vmax * 1.001 and apeak <= amax * 1.001:
            return t.tolist(), [list(row) for row in q_t]
        # 超限则降低巡航速度（等比），下轮重新生成剖面
        factor = max(vpeak / vmax, math.sqrt(apeak / amax))
        v_u = v_u / max(factor, 1.05)
        if v_u < 1e-3:
            break
    return t.tolist(), [list(row) for row in q_t]


def _s_curve_profile(
    S: float, vmax: float, amax: float, jmax: float, dt: float = 0.01
) -> Tuple[List[float], List[float]]:
    """L2 沿弧长的 7 段式对称 jerk-limited S-curve：返回 (times, arc_positions)。

    速度剖面 = 加速（jerk 限幅梯形加速度）→ 巡航 vmax → 减速（对称）；有巡航段
    故平均速度接近 vmax（比 quintic 单峰快 ~1.8 倍），加加速度在加速/减速段为
    有限阶跃（介于 blend 的无穷大阶跃与 quintic 的连续之间）。
    """
    import numpy as np

    def accel_segment(v, amx, jmx, dtf):
        tj = amx / jmx
        if v >= amx * tj:
            t_flat = v / amx - tj
            t_a = 2.0 * tj + t_flat
            a_peak = amx
        else:
            a_peak = math.sqrt(v * jmx)
            tj = a_peak / jmx
            t_a = 2.0 * tj
        nt = max(8, int(round(t_a / dtf)) + 1)
        tt = np.linspace(0.0, t_a, nt)
        a = np.where(tt < tj, jmx * tt,
                     np.where(tt < t_a - tj, a_peak, jmx * (t_a - tt)))
        vel = np.cumsum(a) * dtf
        s = np.cumsum(vel) * dtf
        return t_a, float(s[-1]), tj, a_peak

    tj = amax / jmax
    t_a, s_a, tj, a_peak = accel_segment(vmax, amax, jmax, dt / 4.0)
    if 2.0 * s_a >= S:
        lo, hi = 0.0, vmax
        for _ in range(50):
            mid = (lo + hi) / 2.0
            _, sa, _, _ = accel_segment(mid, amax, jmax, dt / 4.0)
            if 2.0 * sa >= S:
                hi = mid
            else:
                lo = mid
        vmax_u = (lo + hi) / 2.0
        t_cruise = 0.0
    else:
        vmax_u = vmax
        t_cruise = (S - 2.0 * s_a) / vmax

    tj = amax / jmax
    if vmax_u >= amax * tj:
        t_flat = vmax_u / amax - tj
        t_a = 2.0 * tj + t_flat
        a_peak = amax
    else:
        a_peak = math.sqrt(vmax_u * jmax)
        tj = a_peak / jmax
        t_a = 2.0 * tj
    T = 2.0 * t_a + t_cruise
    nt = max(40, int(round(T / dt)) + 1)
    t = np.linspace(0.0, T, nt)
    a = np.zeros(nt)
    for i in range(nt):
        tt = t[i]
        if tt < tj:
            a[i] = jmax * tt
        elif tt < t_a - tj:
            a[i] = a_peak
        elif tt < t_a:
            a[i] = jmax * (t_a - tt)
        elif tt < t_a + t_cruise:
            a[i] = 0.0
        elif tt < t_a + t_cruise + tj:
            a[i] = -jmax * (tt - t_a - t_cruise)
        elif tt < 2 * t_a + t_cruise - tj:
            a[i] = -a_peak
        else:
            a[i] = -jmax * (2 * t_a + t_cruise - tt)
    vel = np.cumsum(a) * dt
    vel[-1] = 0.0
    s = np.cumsum(vel) * dt
    if s[-1] > 1e-9:
        s = s * (S / s[-1])
    s[0] = 0.0
    s[-1] = S
    s = np.maximum.accumulate(np.clip(s, 0.0, S))
    return t.tolist(), s.tolist()


def _s_curve_timed_traj(
    dense: List[List[float]], vmax: float, amax: float
) -> Optional[Tuple[List[float], List[List[float]]]]:
    """L2: C2 样条稠密路径 → S-curve（jerk-limited 梯形速度 + 巡航段）重定时。

    与 quintic 版共用几何路径（过点样条），仅时间参数化不同——S-curve 有巡航段，
    巡游速度明显更快。返回 (times, positions)，退化返回 None。
    """
    import numpy as np

    q = np.asarray(dense, dtype=float)
    n, m = q.shape
    if n < 3 or m < 1:
        return None
    seg = np.linalg.norm(np.diff(q, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    S = float(s[-1])
    if S <= 1e-9:
        return None
    vmax = max(vmax, 1e-3)
    amax = max(amax, 1e-3)
    jmax = 6.0 * amax  # jerk ramp 时间 = 1/6 s
    t, s_t = _s_curve_profile(S, vmax, amax, jmax)
    # 线性插值（不用 cubic 样条）：稠密点下 dq/ds ≤ 1 天然成立，避免 cubic 样条 q(s)
    # 导数过冲（dq/ds 达 2.5 倍）导致关节速度超限。
    s_t = np.clip(np.asarray(s_t), 0.0, S)
    q_t = np.column_stack([np.interp(s_t, s, q[:, j]) for j in range(m)])
    return t, [list(row) for row in q_t]


def _trapezoidal_timed_traj(
    dense: List[List[float]], vmax: float, amax: float
) -> Optional[Tuple[List[float], List[List[float]]]]:
    """L2 梯形：dense 路径 → 纯梯形速度剖面（恒加速 → 恒速巡航 → 恒减速）重定时。

    加速度方波（bang-bang，加加速度在起止为无穷脉冲），速度剖面严格梯形——工业最
    基础梯形速度曲线。位移不足以达 vmax 时退化为三角形（无巡航段）。线性插值
    `np.interp` 保形（避免样条过冲），缩放循环降巡航速度保证峰值 ≤ vmax/amax。
    返回 (times, positions)，退化返回 None。
    """
    import numpy as np

    q = np.asarray(dense, dtype=float)
    n, m = q.shape
    if n < 2 or m < 1:
        return None
    seg = np.linalg.norm(np.diff(q, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    S = float(s[-1])
    if S <= 1e-9:
        return None
    vmax = max(vmax, 1e-3)
    amax = max(amax, 1e-3)

    def _arc_eval(s_t: np.ndarray) -> np.ndarray:
        return np.column_stack([np.interp(s_t, s, q[:, j]) for j in range(m)])

    def _profile(S: float, v: float):
        t_a = v / amax
        s_a = 0.5 * v * t_a
        if 2.0 * s_a >= S:
            v = math.sqrt(S * amax)  # 三角形剖面（无巡航）
            t_a = v / amax
            s_a = 0.5 * v * t_a
            s_c = 0.0
        else:
            s_c = S - 2.0 * s_a
        t_c = s_c / v if v > 0.0 else 0.0
        return 2.0 * t_a + t_c, t_a, s_a, s_c, t_c, v

    v_u = vmax
    for _ in range(12):
        T, t_a, s_a, s_c, t_c, v_u = _profile(S, v_u)
        dt = 0.01
        n_t = max(40, int(round(T / dt)) + 1)
        t = np.linspace(0.0, T, n_t)
        s_t = np.zeros(n_t)
        for i, tt in enumerate(t):
            if tt < t_a:
                s_t[i] = 0.5 * amax * tt * tt
            elif tt < t_a + t_c:
                s_t[i] = s_a + v_u * (tt - t_a)
            else:
                tr = T - tt
                s_t[i] = S - 0.5 * amax * tr * tr
        s_t[-1] = S
        q_t = _arc_eval(np.clip(s_t, 0.0, S))
        vel = np.gradient(q_t, t, axis=0)
        acc = np.gradient(vel, t, axis=0)
        vpeak = float(np.max(np.abs(vel)))
        apeak = float(np.max(np.abs(acc)))
        if vpeak <= vmax * 1.001 and apeak <= amax * 1.001:
            return t.tolist(), [list(row) for row in q_t]
        factor = max(vpeak / vmax, math.sqrt(apeak / amax))
        v_u = v_u / max(factor, 1.05)
        if v_u < 1e-3:
            break
    return t.tolist(), [list(row) for row in q_t]


def _bell_timed_traj(
    dense: List[List[float]], vmax: float, amax: float
) -> Optional[Tuple[List[float], List[List[float]]]]:
    """L3 钟形：dense 路径 → 纯 quintic 钟形速度剖面（无巡航段）重定时。

    沿弧长 s(t)=S·quintic(t/T)，quintic 0→1 起止零速/零加速度，速度钟形、加速度
    连续、加加速度连续（C2）。初始 T 用 quintic 峰值系数（vpeak=1.875·S/T、
    apeak=5.7735·S/T²）估算，缩放循环保证 ≤ vmax/amax。线性插值 `np.interp` 保形。
    返回 (times, positions)，退化返回 None。
    """
    import numpy as np

    q = np.asarray(dense, dtype=float)
    n, m = q.shape
    if n < 3 or m < 1:
        return None
    seg = np.linalg.norm(np.diff(q, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    S = float(s[-1])
    if S <= 1e-9:
        return None
    vmax = max(vmax, 1e-3)
    amax = max(amax, 1e-3)

    def _arc_eval(s_t: np.ndarray) -> np.ndarray:
        return np.column_stack([np.interp(s_t, s, q[:, j]) for j in range(m)])

    T = max(1.875 * S / vmax, math.sqrt(5.7735 * S / amax), 0.05)
    for _ in range(12):
        dt = 0.01
        n_t = max(40, int(round(T / dt)) + 1)
        t = np.linspace(0.0, T, n_t)
        u = t / T
        s_t = S * (10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5)
        s_t[-1] = S
        q_t = _arc_eval(np.clip(s_t, 0.0, S))
        vel = np.gradient(q_t, t, axis=0)
        acc = np.gradient(vel, t, axis=0)
        vpeak = float(np.max(np.abs(vel)))
        apeak = float(np.max(np.abs(acc)))
        if vpeak <= vmax * 1.001 and apeak <= amax * 1.001:
            return t.tolist(), [list(row) for row in q_t]
        factor = max(vpeak / vmax, math.sqrt(apeak / amax))
        T = T * max(factor, 1.05)
        if T > 1e5:
            break
    return t.tolist(), [list(row) for row in q_t]


def _finite_diff_metrics(
    times: List[float], positions: List[List[float]]
) -> Optional[Tuple[float, float]]:
    """命令轨迹（噪声自由）有限差分平滑度：返回 (a_rms, j_rms)，跨关节 RMS 均值。

    F161 A/B 用——/joint_states 反馈 50Hz 采样做三阶差分噪声淹没真实 jerk，故在命令
    轨迹上算（同 traj_smoothness_calibration.compute_metrics 的 a_rms/j_rms 口径）。
    """
    n = len(positions)
    m = len(positions[0]) if n else 0
    if n < 5 or m == 0:
        return None
    a_rms = j_rms = 0.0
    for j in range(m):
        q = [p[j] for p in positions]
        v = [0.0] * n
        a = [0.0] * n
        jk = [0.0] * n
        for i in range(n):
            if i == 0:
                dt = times[1] - times[0]
                v[i] = (q[1] - q[0]) / dt if n > 1 and dt > 0 else 0.0
            elif i == n - 1:
                dt = times[i] - times[i - 1]
                v[i] = (q[i] - q[i - 1]) / dt if dt > 0 else 0.0
            else:
                dt = times[i + 1] - times[i - 1]
                v[i] = (q[i + 1] - q[i - 1]) / dt if dt > 0 else 0.0
        for i in range(n):
            dt = (times[1] - times[0]) if i == 0 else (times[i] - times[i - 1])
            if i == 0:
                a[i] = (v[1] - v[0]) / dt if n > 1 and dt > 0 else 0.0
            else:
                a[i] = (v[i] - v[i - 1]) / dt if dt > 0 else 0.0
        for i in range(n):
            dt = (times[1] - times[0]) if i == 0 else (times[i] - times[i - 1])
            if i == 0:
                jk[i] = (a[1] - a[0]) / dt if n > 1 and dt > 0 else 0.0
            else:
                jk[i] = (a[i] - a[i - 1]) / dt if dt > 0 else 0.0
        a_rms += math.sqrt(sum(x * x for x in a) / n)
        j_rms += math.sqrt(sum(x * x for x in jk) / n)
    return a_rms / m, j_rms / m


# F94: JTC VARIABLE_DEGREE_SPLINE 两点（端点 v/a=0）实测峰值速度/平均速度≈2.0–2.09
# （mock 全栈 200 Hz 实测），地板时长须按该形状系数放大，否则只约束了平均值。
_SPLINE_PEAK_FACTOR = 2.2


def _smooth_points(
    points: List[JointTrajectoryPoint], window: int
) -> List[JointTrajectoryPoint]:
    """
    LL-047: 中心滑动平均低通（仅动 positions，保留 time_from_start）.

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


def _time_warp_points(
    points: List[JointTrajectoryPoint],
    vmax_rad_s: float,
    dt_min_s: float,
    accel_max_rad_s2: float,
) -> List[JointTrajectoryPoint]:
    """
    F59/LL-057: 回放时间轴匀速重排（两段式：弧长均匀重采样 + 加速度限幅膨胀）.

    F57 初版缺陷：`dt = max(s/v_eff, dt_min)` 保留全部点——零位移停顿段每点吃
    dt_min 地板，停顿被重新充气回时间轴（实测 16.6s 示教 → 30.9s 回放，违反「只
    压缩不拉伸」），且停顿↔运动交界速度单 tick 0→v_eff 阶跃（加速度尖峰 11.4
    rad/s²，叠加 F56 V 域前馈后下伺服追不上）。重写：

    1. 弧长均匀重采样——逐段最大关节位移累计弧长 S，v_eff=min(S/原时长, vmax)，
       按 ds=v_eff*dt_min 等弧长步行走、位置在段上线性插值（不新增路径）。停顿段
       （弧长 0）不占时间 → 停顿天然折叠；输出时长 ≈ S/v_eff ≤ 原时长。
    2. 加速度限幅时间膨胀——在步长 5 的粗网格（0.1s 窗）上跑逐窗口恰解，只拉
       dt、不动位置：细步差分混有 ±1 mrad 编码器噪声与「最大关节换帅」离散伪影
       （伪加速度 ~5 rad/s²，逐点恰解会被噪声带飞、往返震荡，实测 21.6s 输入
       膨胀到 38.8s）；粗网格把噪声底降到 ~0.4 rad/s²，只响应真实拐角。窗口内
       dt 均匀回填细网格（50Hz 网格地板保留）。

    与 _smooth_points 配合：先平滑压录制尖峰，再重排压停顿/限加速度。
    """
    n = len(points)
    if n < 3:
        return points
    m = len(points[0].positions)
    dq = [
        max(abs(points[i + 1].positions[j] - points[i].positions[j]) for j in range(m))
        for i in range(n - 1)
    ]
    total = sum(dq)
    if total <= 1e-6:
        return points
    t_end = (
        points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
    )
    v_eff = min(total / t_end, vmax_rad_s) if t_end > 0 else vmax_rad_s

    # ---- 第 1 段：弧长均匀重采样（等弧长步行走，位置段上线性插值）----
    ds = v_eff * dt_min_s
    n_grid = int(math.floor(total / ds)) + 1
    res: List[List[float]] = []
    seg, csum = 0, 0.0  # 当前段索引与「段起点处的累计弧长」
    for k in range(n_grid):
        s = min(k * ds, total)
        while seg < n - 1 and s > csum + dq[seg] + 1e-12:  # 零位移段弧长 0，直接滑过
            csum += dq[seg]
            seg += 1
        if seg >= n - 1:
            res.append(list(points[-1].positions))
            continue
        frac = (s - csum) / dq[seg] if dq[seg] > 1e-12 else 0.0
        frac = min(max(frac, 0.0), 1.0)
        res.append(
            [
                points[seg].positions[j]
                + frac * (points[seg + 1].positions[j] - points[seg].positions[j])
                for j in range(m)
            ]
        )
    n2 = len(res)

    # ---- 第 2 段：加速度限幅时间膨胀（只拉时间、不动位置）----
    # 粗网格（步长 5 锚点，0.1s 窗）上跑「前后窗口分工恰解」：
    #   前向窗口：k 的进窗口侧速度阶跃由 dt_k 吸收（恰解 v_k = v_{k-1} + amax*dt_k）；
    #   后向窗口：k-1 的出窗口侧减速阶跃由 dt_{k-1} 吸收。
    # 恰解式 amax*dt² + vp*dt - dq = 0 的正根；判别式 <0（数值噪声）走有界 ×1.5
    # 增长兜底保证终止。边界窗口按 sqrt(|Δq|/amax) 起停（零初速）。窗口内 dt 均匀
    # 回填细网格，保留 50Hz 地板。
    dts = [dt_min_s] * (n2 - 1)
    if accel_max_rad_s2 > 0.0 and n2 > 2:
        stride = 5
        anchors = list(range(0, n2, stride))
        if anchors[-1] != n2 - 1:
            anchors.append(n2 - 1)
        na = len(anchors)
        cpos = [res[i] for i in anchors]
        cdts = [float(anchors[i + 1] - anchors[i]) * dt_min_s for i in range(na - 1)]
        cdts[0] = max(
            cdts[0],
            max(
                math.sqrt(abs(cpos[1][j] - cpos[0][j]) / accel_max_rad_s2)
                for j in range(m)
            ),
        )
        cdts[-1] = max(
            cdts[-1],
            max(
                math.sqrt(abs(cpos[-1][j] - cpos[-2][j]) / accel_max_rad_s2)
                for j in range(m)
            ),
        )
        for _ in range(40):
            vels = [
                [(cpos[k + 1][j] - cpos[k][j]) / cdts[k] for j in range(m)]
                for k in range(na - 1)
            ]
            changed = False
            for k in range(1, na - 1):
                while True:
                    worst, wj = 0.0, -1
                    for j in range(m):
                        viol = (cpos[k + 1][j] - cpos[k][j]) / cdts[k] - vels[k - 1][j]
                        if viol > worst:
                            worst, wj = viol, j
                    if worst <= accel_max_rad_s2 * cdts[k] * (1.0 + 1e-9):
                        break
                    vp = vels[k - 1][wj]
                    dqj = cpos[k + 1][wj] - cpos[k][wj]
                    disc = vp * vp + 4.0 * accel_max_rad_s2 * dqj
                    if disc >= 0:
                        cdts[k] = (-vp + math.sqrt(disc)) / (2.0 * accel_max_rad_s2)
                    else:
                        while (
                            (cpos[k + 1][wj] - cpos[k][wj]) / cdts[k] - vp
                            > accel_max_rad_s2 * cdts[k] * (1.0 + 1e-9)
                        ):
                            cdts[k] *= 1.5
                    changed = True
                    vels[k] = [
                        (cpos[k + 1][j] - cpos[k][j]) / cdts[k] for j in range(m)
                    ]
            for k in range(na - 2, 1, -1):
                while True:
                    worst, wj = 0.0, -1
                    for j in range(m):
                        viol = (cpos[k][j] - cpos[k - 1][j]) / cdts[k - 1] - vels[k][j]
                        if viol > worst:
                            worst, wj = viol, j
                    if worst <= accel_max_rad_s2 * cdts[k - 1] * (1.0 + 1e-9):
                        break
                    vp = vels[k][wj]
                    dqj = cpos[k][wj] - cpos[k - 1][wj]
                    disc = vp * vp + 4.0 * accel_max_rad_s2 * dqj
                    if disc >= 0:
                        cdts[k - 1] = (-vp + math.sqrt(disc)) / (2.0 * accel_max_rad_s2)
                    else:
                        while (
                            (cpos[k][wj] - cpos[k - 1][wj]) / cdts[k - 1] - vp
                            > accel_max_rad_s2 * cdts[k - 1] * (1.0 + 1e-9)
                        ):
                            cdts[k - 1] *= 1.5
                    changed = True
                    vels[k - 1] = [
                        (cpos[k][j] - cpos[k - 1][j]) / cdts[k - 1] for j in range(m)
                    ]
            if not changed:
                break
        dts = []
        for i in range(na - 1):
            per = cdts[i] / (anchors[i + 1] - anchors[i])
            dts += [per] * (anchors[i + 1] - anchors[i])

    ts: List[float] = []
    acc = 0.0
    for i in range(n2 - 1):
        acc += dts[i]
        ts.append(acc)
    out: List[JointTrajectoryPoint] = []
    for i in range(n2):
        q = JointTrajectoryPoint()
        q.positions = list(res[i])
        q.time_from_start = _duration(ts[i - 1] if i > 0 else 0.0)
        out.append(q)
    return out


def _sanitize_name(name: str) -> str:
    s = re.sub(r"[^A-Za-z0-9_-]", "_", (name or "").strip())
    return s or "trajectory"


def _traj_path_for(traj_dir: str, name: str) -> str:
    """F54: 空名 ≡ latest 槽位（save/playback 都读写 latest.yaml）；非空名 → {name}.yaml."""
    if not (name or "").strip():
        return os.path.join(traj_dir, "latest.yaml")
    return os.path.join(traj_dir, f"{_sanitize_name(name)}.yaml")


def _upsert_pose_lines(lines: List[str], name: str, pos_line: str) -> List[str]:
    """F114: 文本级 upsert 包内 named_poses.yaml（yaml.safe_dump 全量重写会丢注释）.

    同名点位 → 原位替换其 positions 行；否则文件末尾追加带时间戳注释的新点位块。
    """
    name_re = re.compile(rf"^  {re.escape(name)}:\s*$")
    idx = next((i for i, l in enumerate(lines) if name_re.match(l)), None)
    if idx is None:
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if lines and not lines[-1].endswith("\n"):
            lines[-1] += "\n"
        return lines + [
            f"\n  # F114: saved via /a3/arm/save_named_pose at {stamp}\n",
            f"  {name}:\n",
            pos_line + "\n",
        ]
    for j in range(idx + 1, len(lines)):
        if re.match(r"^    positions:", lines[j]):
            lines[j] = pos_line + "\n"
            return lines
        if re.match(r"^  \S", lines[j]):  # 下一个点位键：本名点缺 positions，插在其后
            break
    lines.insert(idx + 1, pos_line + "\n")
    return lines


class ArmController(Node):
    def __init__(self) -> None:
        super().__init__("a3_arm_controller")

        # 可重入回调组：服务回调内会同步调用电机服务（_wait_future 轮询等待），
        # 若用默认 MutuallyExclusiveCallbackGroup，回调阻塞期间 client 响应回调无法
        # 并发执行，导致 init/enable/disable/teach 全部死锁超时。
        self._cb_group = ReentrantCallbackGroup()
        # 夹爪 action 独立互斥回调组：L7 时序步进高频抢占 goal（50Hz），若与其它
        # 可重入回调并发，rclpy ActionClient 的 _goal_handles 清理会竞态抛 KeyError
        # 打崩编排节点。单独互斥组让夹爪 goal 的 execute/result 串行化，消除竞态。
        self._gripper_cb_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("joint_names", JOINTS)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("control_mode_topic", "/a3/control_mode")
        self.declare_parameter("arm_status_topic", "/a3/arm_status")
        # F160: 力矩分量分解遥测（硬件插件发布，本节点镜像进 /a3/arm_status）
        self.declare_parameter("torque_components_topic", "/a3/hardware/torque_components")
        # F51/LL-039：消费看门狗确认故障（电机被带外失能 → 编排层不得停在 READY/TRAJ）
        self.declare_parameter("monitor_status_topic", "/a3/monitor/status")
        self.declare_parameter("unexpected_disable_guard", True)
        self.declare_parameter("unexpected_disable_sustain_s", 1.0)
        self.declare_parameter("gate_topic", "/power_sequence/gate_open")
        self.declare_parameter("power_state_topic", "/power_sequence/state")
        self.declare_parameter("traj_topic", "/joint_group_effort_controller/joint_trajectory")
        # F74: 轨迹执行后端。topic = 旧栈（话题直发 motor_protocol_node）；
        # fjt_action = 标准栈（ros2_control）：L1–L6 → arm_controller FJT，
        # L7 → gripper_controller GripperCommand action（F87）。
        self.declare_parameter("control_backend", "topic")
        self.declare_parameter("arm_fjt_action", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_action", "/gripper_controller/gripper_cmd")
        self.declare_parameter("gripper_default_effort", 1.0)
        # F75: 使能后端。can_service（默认，真机/旧栈，零行为变化）走 /a3/motor/*
        # 服务；controller_switch（标准 ros2_control 栈）走 /controller_manager/
        # switch_controller，硬件插件 on_activate/deactivate 内完成 reset→enable。
        self.declare_parameter("motor_service_backend", "can_service")
        self.declare_parameter("controller_manager_switch_srv",
                               "/controller_manager/switch_controller")
        # F174: 无夹爪形态（系统级模型配置）不含 gripper_controller——
        # STRICT 切换会因未知控制器被整单拒绝，故默认值随形态。
        self.declare_parameter(
            "switch_controllers",
            ["arm_controller"] + (["gripper_controller"] if model_include_gripper() else []),
        )
        # F89b: 自由拖动（示教）在标准栈上是 arm_controller ↔ zero_torque_controller
        # 的原子 STRICT 切换；gripper_controller 不动，拖动中夹爪仍可用。
        self.declare_parameter("freedrive_arm_controller", "arm_controller")
        self.declare_parameter("freedrive_controller", "zero_torque_controller")
        # F127: 示教自由拖动（ZERO_TORQUE）期间夹爪 L7 一并释放——enter 时 gripper
        # 与 arm 同一原子 STRICT swap 出、exit 按进入时是否真带 g 恢复（LL-136）
        self.declare_parameter("freedrive_gripper_controller", "gripper_controller")
        self.declare_parameter("freedrive_switch_timeout_s", 5.0)
        # F83/LL-086: Humble switch_controller does NOT auto-activate hardware
        # booted INACTIVE; arm/disarm must drive the component lifecycle
        # explicitly via set_hardware_component_state (on_activate runs the
        # vendor enable choreography, on_deactivate the reset/coast).
        self.declare_parameter("hardware_component_name", "RsA3System")
        self.declare_parameter("controller_manager_hw_state_srv",
                               "/controller_manager/set_hardware_component_state")
        self.declare_parameter("status_hz", 10.0)
        self.declare_parameter("init_zero_tol_rad", 0.05)
        self.declare_parameter("init_timeout_s", 5.0)
        self.declare_parameter("require_gate", False)
        self.declare_parameter("goto_duration_s", 3.0)
        # F67: goto/move_to/safe-park 默认走 move_group action（OMPL + TOTG adapter），
        # move_group 不可用/规划失败/超时 → WARN 并退回下面的本地线性插值。
        self.declare_parameter("goto_use_moveit", True)
        self.declare_parameter("moveit_goto_timeout_s", 15.0)
        # F134: Sequence 整序列规划+执行等待上限（含 N 段执行，需比单段长）
        self.declare_parameter("sequence_move_timeout_s", 90.0)
        self.declare_parameter("moveit_action_name", "move_action")
        self.declare_parameter("moveit_planning_group", "arm")
        self.declare_parameter("moveit_allowed_planning_time_s", 3.0)
        self.declare_parameter("moveit_num_planning_attempts", 5)
        self.declare_parameter("moveit_velocity_scaling", 0.3)
        self.declare_parameter("moveit_acceleration_scaling", 0.3)
        self.declare_parameter("moveit_goal_tolerance_rad", 0.01)
        # F116: 随机点位巡游（默认抽 5 个；zero 为上电机械零位，真机巡游排除）
        self.declare_parameter("random_tour_count", 5)
        self.declare_parameter("random_tour_exclude_poses", ["zero"])
        # F116: 单腿规划/门禁失败时换抽点位重试次数（0=首次失败即中止巡游）
        self.declare_parameter("random_tour_leg_retries", 3)
        # F134: 巡游整序列一次规划（true，pilz PTP blend，中间点不停车）；
        # false 或 sequence 不可用 → 回退逐腿规划。
        self.declare_parameter("random_tour_use_sequence", True)
        self.declare_parameter("random_tour_blend_radius_m", 0.02)
        # F161: 巡游轨迹平滑三档（L1 blend / L2 S-curve / L3 quintic）。
        # blend = F134 pilz Sequence（转角 blend，加速度不连续）；
        # s_curve = 过点样条 + jerk-limited S-curve（梯形速度含巡航段，快）；
        # quintic = 过点样条 + quintic 单峰时间剖面（jerk 连续，最平滑但慢）。
        # 任一新链环节失败 WARN 并回退 F134 sequence / F116 逐腿链。
        self.declare_parameter("random_tour_smooth_mode", "quintic")
        self.declare_parameter("random_tour_smooth_max_vel_rad_s", 1.5)
        self.declare_parameter("random_tour_smooth_max_acc_rad_s2", 2.0)
        self.declare_parameter("random_tour_smooth_resample_n", 200)
        # F163: 轨迹段 via 过点（gotoby 原语）——当前位→中间过点→目标 一条 C2 平滑轨迹。
        # 峰值速度/加速度上限（rad/s、rad/s²）与过点样条稠密重采样点数；speed(0~1) 线性缩放 v/a。
        self.declare_parameter("goto_via_max_vel_rad_s", 0.6)
        self.declare_parameter("goto_via_max_acc_rad_s2", 2.0)
        self.declare_parameter("goto_via_resample_n", 200)
        # F165: 工业 G64 连续路径——l1p5/l2 直线折线的 via 拐角做二次 Bezier 圆角半径
        # （关节空间弧度）；直线段保形不变，仅切角消除加速度尖峰。<=0 关闭圆角。
        self.declare_parameter("goto_via_corner_blend_rad", 0.15)
        # F167: 软件模式（jnt_g64_*/jnt_spline_*）时间律后端——software=手写剖面（默认，
        # 作为回退保留）；totg/ruckig=调 /a3/arm/retime_trajectory 用成熟库换时间律
        # （几何路径不变，仅时间律形状变），服务不可用/失败自动回退 software。
        self.declare_parameter("goto_via_time_backend", "software")
        self.declare_parameter("goto_via_retime_timeout_s", 5.0)
        self.declare_parameter("playback_ramp_duration_s", 2.5)
        # F68: 回放（ramp + 录制点）默认调 /a3/arm/retime_trajectory 做保几何重定时
        # （Ruckig jerk-limited，失败退化 TOTG）；服务不可用/失败才走旧的
        # smooth + time_warp 链。
        self.declare_parameter("playback_retime", True)
        self.declare_parameter("playback_retime_backend", "ruckig")
        # 默认把重定时后的总时长拉到录制时长（手拖的自然节奏），而不是按关节极限
        # 「越快越好」；关闭则用下面的固定缩放。
        self.declare_parameter("playback_match_recorded_duration", True)
        self.declare_parameter("playback_velocity_scaling", 0.2)
        self.declare_parameter("playback_acceleration_scaling", 0.2)
        self.declare_parameter("playback_retime_min_duration_s", 1.0)
        # F137: 回放前对录制点做几何去噪（逐关节五次 B 样条，RMS 残差 ≈ ε rad）。
        # ε=0 关闭；0.005 保形、0.02 更平滑。去噪在 F68 重定时之前，L1-L6 进 retime、
        # L7 进序列下发，均为去噪后几何；重定时成功烘焙时落盘即 F137+F68 结果。
        self.declare_parameter("playback_geometric_smoothing_eps", 0.005)
        # F133: Ruckig 重定时成功后把优化轨迹烘焙回 latest.yaml（原文件备份
        # *.preretime.yaml）；下次回放直接复用平滑轨迹。仅 latest 槽，时间戳
        # 备份文件永不改写。
        self.declare_parameter("playback_retime_writeback", True)
        # F124: 回放回首点段走 move_group 轨迹规划（true，灯 cyan）；false 或规划失败
        # 回落几何 ramp 插值（F38b，无跳变）
        self.declare_parameter("playback_return_use_moveit", True)
        # LL-047: 回放低通平滑窗口（中心滑动平均，@50Hz 采样数）。7 点把录制 L2/L3 最大
        # 加速度 119/86 → ~8 rad/s²（-93%），几何扰动 ≤20 mrad；0/1 关闭。手拖录制天然带
        # 加速度尖峰，伺服忠实复现即"抖"——平滑压尖峰而非改路径。热设置回放前读取。
        self.declare_parameter("playback_smooth_samples", 7)
        # F57/F59/LL-053: 回放时间轴匀速重排（弧长均匀重采样压停顿 + 加速度限幅膨胀
        # 消速度跳变，保几何路径）。vmax 是单关节速度上限——实测 0.6 rad/s 跟踪干净；
        # dt_min 是 @50Hz 网格地板；accel_max 只拉时间不动位置（0 = 关闭膨胀 pass）。
        self.declare_parameter("playback_time_warp", True)
        self.declare_parameter("playback_warp_vmax_rad_s", 0.6)
        self.declare_parameter("playback_warp_dt_min_s", 0.02)
        self.declare_parameter("playback_warp_accel_max_rad_s2", 2.0)
        # F57/LL-053: 保存时轻量平滑（中心滑动平均窗口采样数；0/1/2 = 不保存平滑）。
        # 落盘即干净（latest.yaml / teach_*.yaml 任何消费方受益），时间轴不变。
        self.declare_parameter("teach_save_smooth_samples", 5)
        # F54: 示教停止自动保存的最小样本数——start 后立刻 stop 的误触发（1~2 个样本）不许
        # 用退化单点文件覆盖上一条好的 latest。~0.2s 拖动即可超过（@50Hz 10 样本）。
        self.declare_parameter("teach_auto_save_min_samples", 10)
        # F131: 路点示教参数
        # 停止保存的最少路点数（少于则视为误触发，保留上一条好的任务）
        self.declare_parameter("waypoint_min_count", 2)
        # 相邻打点最小关节增量（rad），小于则视为重复点跳过（不增加计数）
        self.declare_parameter("waypoint_min_capture_delta_rad", 0.01)
        # F131: 单段规划/执行失败重试次数（段间同步滞后的瞬态失败重等 current 后
        # 重试；0=段失败即整体中止，2026-10 前旧行为）
        self.declare_parameter("waypoint_segment_retries", 2)
        # F134: 路点任务整序列一次规划/执行（true；pilz Sequence，blend 过中间点
        # 不停车）；false 或 sequence 不可用 → 回退逐段规划。
        self.declare_parameter("waypoint_use_sequence", True)
        # 单段规划/执行超时（s）；0 = 复用 moveit_goto_timeout_s
        self.declare_parameter("waypoint_segment_timeout_s", 0.0)
        # F134: 拐角 blend 半径（pilz Sequence；0 = 每点到位即停精确过点；
        # 默认 0.02m 圆滑过弯，轨迹在该圆盘内偏离中间点）
        self.declare_parameter("waypoint_blend_radius_m", 0.02)
        # F131: pilz LIN 的 request 缩放只缩放笛卡尔速度剖面,不缩放关节限位检查;
        # 靠近奇异/位形翻转的直线段关节加速度尖峰需靠降低笛卡尔速度(v^2)来压住。
        self.declare_parameter("waypoint_lin_velocity_scaling", 0.1)
        self.declare_parameter("waypoint_lin_acceleration_scaling", 0.1)
        # LIN 目标位置/姿态容差（传给约束构造）
        self.declare_parameter("waypoint_lin_pos_tol_m", 0.002)
        self.declare_parameter("waypoint_lin_rot_tol_rad", 0.002)
        self.declare_parameter("waypoint_base_link", "base_link")
        self.declare_parameter("waypoint_ee_link", "end_effector")
        # F131: LIN 规划组（SRDF arm_lin：pick_ik global；arm 组 local 留给 servo）
        self.declare_parameter("waypoint_lin_planning_group", "arm_lin")
        # 打点操作员反馈话题（ds4_feedback_node 白闪+弱震消费）
        self.declare_parameter("operator_event_topic", "/a3/arm/operator_event")
        self.declare_parameter("trajectories_dir", "~/.a3/trajectories")
        # F174: 留空 = 跟随系统级模型配置的描述包（a3_robot_model/config/robot_model.yaml）
        self.declare_parameter("named_poses_pkg", model_description_package())
        # F41/F88: move_to/goto 兜底——最短时长；两点轨迹由 JTC 样条插值
        self.declare_parameter("move_to_min_duration_s", 3.0)
        # F94: 两点轨迹速度限幅 = URDF velocity × scale（1.0=只堵无限速漏洞）
        self.declare_parameter("joint_velocity_scale", 1.0)
        # F40: 失能保护（不在 idle 容差内先平滑回 idle 再失能）；F113 home→idle
        self.declare_parameter("disable_home_pose_name", "idle")
        self.declare_parameter("disable_home_tol_rad", 0.15)
        self.declare_parameter("disable_home_duration_s", 3.0)
        self.declare_parameter("disable_home_confirm_s", 0.5)
        self.declare_parameter("disable_park_timeout_s", 8.0)
        # F75: 工业级落定判据——位置 AND 速度同时落定才允许失能
        self.declare_parameter("disable_home_settle_tol_rad", 0.02)
        self.declare_parameter("disable_home_settle_vel_rad_s", 0.05)
        # F130: 真机 L2/L3 在 idle(~0) 存在 ~0.02–0.03 rad 自然下垂/机械止挡残差，
        # 严格 0.02 容差永远够不到 → park 超时 FAULT。新增稳态无进展判定：静止 +
        # 残差≤residual + 持续 plateau_s 位置不再改善（物理受限，非运动中）才放行；
        # 残差 >residual_hard_floor 绝不失能。
        self.declare_parameter("disable_park_residual_tol_rad", 0.08)
        self.declare_parameter("disable_park_residual_hard_floor_rad", 0.12)
        self.declare_parameter("disable_park_plateau_s", 0.8)
        self.declare_parameter("disable_park_no_progress_rad", 0.01)
        # 稳态静止速度门：独立于严格 settle_vel(0.05)。真机 idle 保持时速度反馈噪声/
        # 微调常达 0.05–0.12，用 0.05 会让 plateau 计时反复清零（紫灯 ~11s 且偶发超时）。
        # 0.15 容忍噪声，真正运动时速度远大于此。
        self.declare_parameter("disable_park_plateau_vel_rad_s", 0.15)
        # F119: R3 软失能前退出 BLOCKED_MODES（SERVO/零力矩/重力补偿）的等待上限
        self.declare_parameter("disable_mode_exit_timeout_s", 2.0)
        # F43: 最大力矩持久化
        self.declare_parameter("motor_states_topic", "/a3/motor/states")
        self.declare_parameter("torque_stats_file", "~/.a3/stats/torque_stats.yaml")
        self.declare_parameter("torque_stats_save_interval_s", 10.0)
        # F43 防护：超量程/非有限力矩判定为异常反馈，不污染历史峰值统计
        # （历史 L7 曾记下 23.8 Nm 脏值——超 EL05 ±6 量程 4 倍）
        self.declare_parameter("torque_stats_max_nm", 20.0)
        # 回放 L7 夹爪按录制时间序列步进下发（GripperCommand 抢占式），复现录制时
        # 的张合时序，而非只取末点（GAC 无轨迹接口，见 _dispatch_trajectory）。
        # 20ms 步进 + 相邻点线性插值消除阶梯跳变；l7_max_velocity_rad_s 硬限速。
        self.declare_parameter("l7_seq_tick_s", 0.02)
        self.declare_parameter("l7_seq_min_delta_rad", 0.01)
        self.declare_parameter("l7_max_velocity_rad_s", 1.8)
        # F44: 温度管理（warn 仅告警；protect 自动回 idle 失能降温；迟滞恢复）
        # 默认阈值 2026-09-13 调高：官方电机自带 130°C 保护兜底，初版 65°C 过低（LL-023）
        self.declare_parameter("temp_protect_enabled", True)
        self.declare_parameter("temp_warn_c", 90.0)
        self.declare_parameter("temp_protect_c", 95.0)
        self.declare_parameter("temp_hysteresis_c", 5.0)
        self.declare_parameter("fault_mask_reset_on_fault", True)
        # 软故障（欠压/未标定）持续时长阈值：超过才触发 safe park → disable。
        # 欠压常为锂电瞬时压降，给自恢复留时间；不自恢复才降级失能。
        self.declare_parameter("soft_fault_grace_s", 1.5)
        # F48: 使能前读数限位门禁（环绕读数超限时拒绝使能，见 LL-019）
        self.declare_parameter("enable_position_check", True)
        # 限位比较裕量：set_zero 后编码器量化噪声 ±0.0002（L2/L3/L7 下界为 0），
        # 1e-6 不够；0.001 rad ≈ 0.057°，远小于环绕量 2π
        self.declare_parameter("position_check_margin_rad", 0.001)
        # /joint_states 最大陈旧时长：桥异常（refresh 停发）时 js 会冻结在旧值
        # （LL-020），旧值校验形同虚设——超过此时长视为不新鲜，拒绝使能
        self.declare_parameter("js_max_stale_s", 1.0)
        # F81: 硬件插件单电机反馈看门狗锁存话题（freeze-hold 期间为 true）
        self.declare_parameter("feedback_stale_topic", "/a3/hardware/feedback_stale")
        # F90: 故障黑匣子——rosbag2 snapshot-mode 录制器就绪时，进入 FAULT 的边沿
        # 异步触发一次落盘；录制器不在（use_rosbag:=false）则静默跳过，不阻塞故障处置
        self.declare_parameter("blackbox_snapshot_enabled", True)
        self.declare_parameter("blackbox_snapshot_service", "/rosbag2_recorder/snapshot")
        # F107: 额定负载（对标工业机器人 rated payload）
        self.declare_parameter("payload_mass_kg", 0.0)
        self.declare_parameter("payload_com_m", [0.0, 0.0, 0.0])
        self.declare_parameter("rated_payload_kg", 1.5)
        self.declare_parameter("payload_parent_link", "gripper_link")
        # F107: 静态力矩门禁参考力矩（N·m；已从连续额定 5.0/1.8 放宽，见 yaml）
        self.declare_parameter(
            "joint_rated_torque", [7, 7, 7, 3, 3, 3, 2])
        self.declare_parameter("static_torque_margin_ratio", 1.0)
        # F107: 占空比门禁（滚动窗口热保护前馈）
        self.declare_parameter("duty_gate_enabled", True)
        self.declare_parameter("duty_window_s", 600.0)
        self.declare_parameter("duty_max_ratio", 0.8)

        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self._n_joints = len(self._joint_names)
        # F52 档位配对自检：收到首条 /joint_states 后校验实际关节数与配置一致
        self._profile_check_done = False
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
        # F160: 力矩分量分解缓存（按 self._joint_names 顺序；缺省 0）
        self._ff_gravity: List[float] = [0.0] * self._n_joints
        self._ff_coriolis: List[float] = [0.0] * self._n_joints
        self._ff_inertia: List[float] = [0.0] * self._n_joints
        self._ff_total: List[float] = [0.0] * self._n_joints
        self._pd_feedback: List[float] = [0.0] * self._n_joints
        self._command_torque: List[float] = [0.0] * self._n_joints
        self._last_js_stamp = None  # F48: /joint_states 新鲜度检查（LL-020）
        # F81: 硬件反馈看门狗锁存（任一电机反馈超时即 true，插件侧整臂 freeze-hold）
        self._feedback_stale = False
        # jog（滑动条直驱）进行中标志：区分 TRAJ 是 jog 还是 goto/playback
        self._jogging = False

        # F107: 额定负载状态（当前负载质量/质心）+ pinocchio 模型缓存
        self._payload_mass: float = float(self.get_parameter("payload_mass_kg").value)
        self._payload_com: List[float] = [
            float(v) for v in self.get_parameter("payload_com_m").value]
        self._pin_model = None          # 反映当前负载的 pinocchio 模型
        self._pin_q_idx: List[int] = []  # self._joint_names -> model q 索引
        self._pin_v_idx: List[int] = []  # self._joint_names -> model v 索引
        self._pin_ready = False
        # F107: 占空比滚动窗，元素 (segment_start_monotonic, duration_s)
        self._duty_segments: Deque[Tuple[float, float]] = collections.deque()

        # F43: 每关节最大力矩统计（正负双向 + 绝对值，节流落盘，重启恢复）
        self._torque_stats: Dict[str, Dict[str, Any]] = self._load_torque_stats()
        self._torque_stats_dirty = False
        self._torque_stats_saved_at = 0.0
        # 本次运行峰值（仅内存，重启清零，不进历史持久化文件）
        self._torque_session_max: Dict[str, float] = {}
        # F44: 温度/故障监视（fresh 门控；无反馈温度=0.0，勿当 NaN 判读）
        self._temperatures: List[float] = [0.0] * self._n_joints
        self._temp_fresh: List[bool] = [False] * self._n_joints
        # F60：电机使能态（来自 /a3/motor/states，fresh 门控）——电源序列
        # EnableInit 已使能电机时，/a3/arm/enable 不再重复发 F32 门禁互锁的
        # /a3/motor/enable 调试写。
        self._motor_enabled: List[bool] = [False] * self._n_joints
        self._motor_state_fresh: List[bool] = [False] * self._n_joints
        self._temp_warn = False
        self._temp_protect_pending = False
        # LL-146: ReentrantCallbackGroup 下 _trigger_temp_protect 阻塞期间可能被再次
        # 触发（safe-park 中 move_group error -4 会短暂置 READY，绕过 SAFE_PARK 守卫），
        # 重入 _safe_park_then_disable 对已失能控制器再 deactivate → STRICT 拒绝。busy 锁防重入。
        self._temp_protect_busy = False
        self._fault_reset_pending = False
        self._pending_fault: Tuple[str, int] = ("", 0)
        # 软故障（欠压/未标定）计时：首次出现时间 + 关节 + mask；持续超阈值才降级
        self._soft_fault_since: Optional[float] = None
        self._soft_fault_joint = ""
        self._soft_fault_mask = 0

        # 示教录制
        self._recording = False
        self._record_start = 0.0
        self._record: List[Tuple[float, List[float]]] = []
        # F131: 路点示教——每项 {"positions": [7], "pose": {...}}（pose 供 LIN 使用）
        self._waypoints: List[Dict[str, Any]] = []
        # F127: freedrive enter 是否真把 gripper 换出——INACTIVE 无法区分「被我们
        # 停」与「未配置」，故记标志而非 exit 再探测（LL-136）
        self._freedrive_gripper_swapped = False

        # 回放 L7 夹爪时间序列下发（GripperCommand 抢占式步进）
        self._l7_seq: List[Tuple[float, float]] = []
        self._l7_seq_idx = 0
        self._l7_seq_t0 = 0.0
        self._l7_seq_timer = None
        # L7 目标流硬限速（l7_max_velocity_rad_s）状态
        self._l7_last_cmd_pos: Optional[float] = None
        self._l7_last_cmd_t = 0.0

        # 轨迹持久化目录
        self._traj_dir = self._resolve_trajectories_dir()
        os.makedirs(self._traj_dir, exist_ok=True)
        # F131: 路点任务子目录
        self._wp_dir = os.path.join(self._traj_dir, "waypoints")
        os.makedirs(self._wp_dir, exist_ok=True)

        # 命名预设点
        self._poses = self._load_poses()
        # 关节限位（来自 URDF，与前端滑动条上下限同源）
        self._joint_vel_limits: Dict[str, float] = {}
        self._joint_limits = self._load_joint_limits()
        if self._joint_vel_limits:
            self.get_logger().info(
                "F94 joint velocity limits (rad/s): "
                + ", ".join(f"{n}={v:g}" for n, v in self._joint_vel_limits.items())
            )

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
            String, str(self.get_parameter("power_state_topic").value),
            self._on_power_state, latched_qos,
            callback_group=self._cb_group,
        )
        # F43/F44: 电机原始状态（力矩统计 + 温度/故障监视）；发布端 SensorDataQoS，
        # best_effort 订阅兼容（js_qos 同）
        self.create_subscription(
            MotorStates, str(self.get_parameter("motor_states_topic").value),
            self._on_motor_states,
            js_qos, callback_group=self._cb_group,
        )
        # F160: 力矩分量分解（前馈 + PD + 实测）；镜像进 /a3/arm_status
        self.create_subscription(
            TorqueComponents, str(self.get_parameter("torque_components_topic").value),
            self._on_torque_components,
            js_qos, callback_group=self._cb_group,
        )
        # F51/LL-039: 看门狗状态（消费 UNEXPECTED_DISABLE——本节点在电机被带外失能后
        # 不得继续停在 READY/TRAJ 并保留陈旧保持目标）
        self.create_subscription(
            MonitorStatus, str(self.get_parameter("monitor_status_topic").value),
            self._on_monitor_status, 10, callback_group=self._cb_group,
        )
        # F81: 硬件插件反馈看门狗（latched；无插件的旧栈下话题不存在，不影响）
        self.create_subscription(
            Bool, str(self.get_parameter("feedback_stale_topic").value),
            self._on_feedback_stale, latched_qos, callback_group=self._cb_group,
        )

        # 发布
        self._arm_status_pub = self.create_publisher(ArmStatus, self._arm_status_topic, 10)
        self._traj_pub = self.create_publisher(JointTrajectory, self._traj_topic, 10)
        self._mode_pub = self.create_publisher(
            String, str(self.get_parameter("control_mode_topic").value), 10
        )
        # F131: 打点/任务事件 → ds4_feedback_node（白闪+弱震等操作员反馈）
        self._operator_event_pub = self.create_publisher(
            String, str(self.get_parameter("operator_event_topic").value), 10
        )
        # /a3/control_mode 是 VOLATILE 且仅状态变化时发布。本节点重启后须主动
        # 清掉其他节点（如夹爪力控互锁）锁存的旧 TRAJ_RUNNING：启动即发一次 +
        # 2 s 后重发一次（首次发布可能早于订阅发现、被静默丢弃）。
        self._publish_mode("IDLE")
        self._startup_mode_timer = self.create_timer(2.0, self._announce_startup_mode)

        # 服务（对外门面）
        self.create_service(
            Trigger, "/a3/arm/init", self._init_cb, callback_group=self._cb_group)
        self.create_service(
            Trigger, "/a3/arm/enable", self._enable_cb, callback_group=self._cb_group)
        self.create_service(
            Trigger, "/a3/arm/disable", self._disable_cb, callback_group=self._cb_group)
        self.create_service(
            SetPayload, "/a3/arm/set_payload", self._set_payload_cb,
            callback_group=self._cb_group)
        self.create_service(
            GotoNamedPose, "/a3/arm/goto_named_pose", self._goto_cb,
            callback_group=self._cb_group)
        self.create_service(
            SetJointPositions, "/a3/arm/set_joint_positions", self._set_joint_positions_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            MoveToJointPositions, "/a3/arm/move_to", self._move_to_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            SaveNamedPose, "/a3/arm/save_named_pose", self._save_named_pose_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/a3/arm/start_teach", self._start_teach_cb,
            callback_group=self._cb_group)
        self.create_service(
            Trigger, "/a3/arm/stop_teach", self._stop_teach_cb,
            callback_group=self._cb_group)
        self.create_service(
            Trigger, "/a3/arm/start_waypoint_teach", self._start_waypoint_teach_cb,
            callback_group=self._cb_group)
        self.create_service(
            SaveTrajectory, "/a3/arm/save_trajectory", self._save_cb,
            callback_group=self._cb_group)
        self.create_service(
            PlaybackTrajectory, "/a3/arm/playback", self._playback_cb,
            callback_group=self._cb_group)
        self.create_service(
            RandomPoseTour, "/a3/arm/random_pose_tour", self._random_pose_tour_cb,
            callback_group=self._cb_group)
        self.create_service(
            Trigger, "/a3/arm/enter_ai", self._enter_ai_cb,
            callback_group=self._cb_group)
        self.create_service(
            Trigger, "/a3/arm/exit_ai", self._exit_ai_cb,
            callback_group=self._cb_group)

        # 底层服务客户端
        self._motor_cli = self.create_client(
            MotorCommand, "/a3/motor/set_zero", callback_group=self._cb_group)
        self._enable_cli = self.create_client(
            MotorCommand, "/a3/motor/enable", callback_group=self._cb_group)
        # F91 扩展：标准栈运行时设零（health_node_ 的 /a3/hardware/set_zero，
        # 复用已占用的 can1；仅电机未使能时放行）。
        self._hw_set_zero_cli = self.create_client(
            MotorIdCommand, "/a3/hardware/set_zero", callback_group=self._cb_group)
        self._reset_cli = self.create_client(
            MotorCommand, "/a3/motor/reset", callback_group=self._cb_group)
        self._zt_start_cli = self.create_client(
            Trigger, "/a3/zero_torque/start", callback_group=self._cb_group)
        self._zt_stop_cli = self.create_client(
            Trigger, "/a3/zero_torque/stop", callback_group=self._cb_group)
        # F119: R3 任意模式软失能——退出 SERVO / 重力补偿的 stop 客户端
        # （stop_servo 是真 Trigger server（servo_node.h），不是 pause（会重锚定测量））
        self._servo_stop_cli = self.create_client(
            Trigger, "/servo_node/stop_servo", callback_group=self._cb_group)
        self._gravity_stop_cli = self.create_client(
            Trigger, "/a3/gravity_compensation/stop", callback_group=self._cb_group)
        # F75: 标准栈使能客户端（motor_service_backend=controller_switch 时用）
        self._switch_cli = self.create_client(
            SwitchController,
            str(self.get_parameter("controller_manager_switch_srv").value),
            callback_group=self._cb_group,
        )
        # F127/LL-136: 探测控制器 active 集合 → freedrive enter 是否连 gripper 一并 swap
        self._list_cli = self.create_client(
            ListControllers, "/controller_manager/list_controllers",
            callback_group=self._cb_group,
        )
        self._hw_state_cli = self.create_client(
            SetHardwareComponentState,
            str(self.get_parameter("controller_manager_hw_state_srv").value),
            callback_group=self._cb_group,
        )
        # F90: rosbag2 snapshot-mode 黑匣子触发服务（注意类型是
        # rosbag2_interfaces/srv/Snapshot，不是 std_srvs/Trigger，见 LL-101）
        self._snapshot_cli = self.create_client(
            Snapshot,
            str(self.get_parameter("blackbox_snapshot_service").value),
            callback_group=self._cb_group,
        )

        # F67: MoveGroup action client（节点启动时不阻塞等待；用前查 server_is_ready）
        self._moveit_cli = ActionClient(
            self,
            MoveGroup,
            str(self.get_parameter("moveit_action_name").value),
            callback_group=self._cb_group,
        )
        # F68: 保几何重定时服务
        self._retime_cli = self.create_client(
            RetimeTrajectory, "/a3/arm/retime_trajectory", callback_group=self._cb_group
        )
        # F134: 整序列一次规划/执行 action（pilz Sequence；不可用时回退逐段链）
        self._sequence_cli = ActionClient(
            self,
            MoveGroupSequence,
            "/sequence_move_group",
            callback_group=self._cb_group,
        )
        # F131: LL-103 段间同步——读 move_group 内部当前状态
        self._scene_cli = self.create_client(
            GetPlanningScene, "/get_planning_scene", callback_group=self._cb_group
        )
        # F74: 标准 FJT action 客户端（control_backend=fjt_action 时用）
        self._fjt_arm_cli = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("arm_fjt_action").value),
            callback_group=self._cb_group,
        )
        self._gripper_cli = ActionClient(
            self,
            GripperCommand,
            str(self.get_parameter("gripper_action").value),
            callback_group=self._gripper_cb_group,
        )

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
            pkg = str(self.get_parameter("named_poses_pkg").value) or model_description_package()
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
        # F39: 用户层点位覆盖包内点位（同名覆盖）。F113 起 ~/.a3/poses.yaml 弃用、
        # F114 起 save_named_pose 改写包内 named_poses.yaml，此处仅为兼容旧文件保留。
        # F39 落盘格式是 {poses: {name: [q...]}}（扁平列表），包级是 {name: {positions: [..]}}，
        # 两种都兼容，避免「list indices must be integers」误告警导致点位回退全零。
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

    def _named_poses_pkg_path(self) -> str:
        """F114: 包内 named_poses.yaml 路径（symlink-install 下写入即落 src）."""
        pkg = str(self.get_parameter("named_poses_pkg").value) or model_description_package()
        share = get_package_share_directory(pkg)
        return os.path.join(share, "config", "named_poses.yaml")

    # ------------------------------------------------------ F43 torque stats

    def _torque_stats_path(self) -> str:
        return os.path.expanduser(str(self.get_parameter("torque_stats_file").value))

    def _load_torque_stats(self) -> Dict[str, Dict[str, Any]]:
        """启动恢复 ~/.a3/stats/torque_stats.yaml（不存在则空）."""
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
        max_nm = float(self.get_parameter("torque_stats_max_nm").value)
        if not math.isfinite(tau) or abs(tau) > max_nm:
            return  # 超量程/非有限值：异常反馈，不污染 F43 历史峰值统计
        # 本次运行峰值（仅内存，重启清零）
        prev = self._torque_session_max.get(jn, 0.0)
        if abs(tau) > prev:
            self._torque_session_max[jn] = abs(tau)
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
        """失能安全位：按 disable_home_pose_name 取点位（F113 起默认 idle），缺失回退全零."""
        name = str(self.get_parameter("disable_home_pose_name").value)
        q = self._poses.get(name)
        if q is None:
            return [0.0] * self._n_joints
        q = list(q)
        if len(q) < self._n_joints:
            q += [0.0] * (self._n_joints - len(q))
        return [float(v) for v in q[: self._n_joints]]

    def _at_home(self, tol: float) -> Tuple[bool, float]:
        """全部关节 |q_i - home_i| ≤ tol 视为已在 home；返回 (是否, 最大偏差)."""
        if not self._have_js:
            return False, float("inf")
        home = self._home_pose()
        worst = 0.0
        for p, h in zip(self._positions, home):
            worst = max(worst, abs(p - h))
        return worst <= tol, worst

    def _load_joint_limits(self) -> Dict[str, Tuple[float, float]]:
        """
        从系统级模型配置的静态 URDF（默认 a3_description_6j/urdf/el_a3.urdf）读取
        各关节 limit lower/upper（与前端滑动条同源）；F174 起路径由 a3_robot_model 决定。
        F94: 同时把 velocity 存入 self._joint_vel_limits（两点轨迹地板时长的唯一来源）。
        """
        import xml.etree.ElementTree as ET

        limits: Dict[str, Tuple[float, float]] = {}
        vel_limits: Dict[str, float] = {}
        try:
            path = model_urdf_path()
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
                try:
                    vel = float(limit.get("velocity"))
                except (TypeError, ValueError):
                    vel = 0.0
                if vel > 0.0:
                    vel_limits[name] = vel
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cannot load joint limits from URDF: {exc}")
        self._joint_vel_limits = vel_limits
        return limits

    def _velocity_floor_duration(
        self,
        q0: List[float],
        q1: List[float],
        joint_names: Optional[List[str]] = None,
    ) -> Tuple[float, str]:
        """
        F94: 两点轨迹峰值速度不超过 URDF velocity 所需的最小时长
        _SPLINE_PEAK_FACTOR × max_i |Δq_i| / (vmax_i × scale)。
        返回 (地板时长, 最慢关节名)；无速度限数据时返回 (0.0, "")——不阻断运动。
        """
        names = list(joint_names or self._joint_names)
        scale = float(self.get_parameter("joint_velocity_scale").value)
        if scale <= 0.0 or not self._joint_vel_limits:
            return 0.0, ""
        floor = 0.0
        slowest = ""
        for a, b, jn in zip(q0, q1, names):
            vmax = self._joint_vel_limits.get(jn)
            if vmax is None:
                continue
            need = _SPLINE_PEAK_FACTOR * abs(float(b) - float(a)) / (vmax * scale)
            if need > floor:
                floor = need
                slowest = jn
        return floor, slowest

    def _check_positions_in_limits(self) -> Tuple[bool, List[str]]:
        """
        F48: 使能前读数限位检查.

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

    # ------------------------------------------------------------------
    # F107: 额定负载 + 静态力矩门禁 + 占空比门禁
    # ------------------------------------------------------------------

    def _read_urdf_text(self) -> str:
        with open(model_urdf_path(), "r") as fh:
            return fh.read()

    def _ensure_pin_model(self) -> None:
        """
        懒构建反映当前负载的 pinocchio 模型。负载以「附加 link + fixed joint」
        注入 URDF 文本（本平台 Python pinocchio 无 appendBodyToJoint，见 LL-123）。
        """
        import numpy as np  # noqa: F401  (pinocchio 运行期依赖)
        import pinocchio as pin  # type: ignore

        base = self._read_urdf_text()
        mass = self._payload_mass
        com = self._payload_com
        if mass > 0.0:
            parent = str(self.get_parameter("payload_parent_link").value)
            injection = (
                "  <link name=\"a3_payload_link\">\n"
                "    <inertial>\n"
                "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
                f"      <mass value=\"{mass:.9g}\"/>\n"
                "      <inertia ixx=\"1e-6\" ixy=\"0\" ixz=\"0\""
                " iyy=\"1e-6\" iyz=\"0\" izz=\"1e-6\"/>\n"
                "    </inertial>\n"
                "  </link>\n"
                "  <joint name=\"a3_payload_joint\" type=\"fixed\">\n"
                f"    <parent link=\"{parent}\"/>\n"
                "    <child link=\"a3_payload_link\"/>\n"
                f"    <origin xyz=\"{com[0]:.9g} {com[1]:.9g} {com[2]:.9g}\""
                " rpy=\"0 0 0\"/>\n"
                "  </joint>\n"
                "</robot>"
            )
            base = base.replace("</robot>", injection, 1)
        model = pin.buildModelFromXML(base)
        q_idx: List[int] = []
        v_idx: List[int] = []
        for jn in self._joint_names:
            jid = model.getJointId(jn)
            q_idx.append(int(model.idx_qs[jid]))
            v_idx.append(int(model.idx_vs[jid]))
        self._pin_model = model
        self._pin_q_idx = q_idx
        self._pin_v_idx = v_idx
        self._pin_ready = True

    def _static_torque_violations(self, positions: List[float]) -> List[str]:
        """
        F107: 给定位形下静态重力矩 |rnea(q,0,0)| 是否超过
        连续额定 × margin_ratio。返回违规描述列表（空=通过）。
        pinocchio 不可用时不阻断（warn 一次）——不得让模型故障静默放行运动，
        但也不能因环境缺包让既有零负载流程全部瘫痪：仅在声明了负载时硬拒。
        """
        if not self._pin_ready:
            try:
                self._ensure_pin_model()
            except Exception as exc:  # noqa: BLE001
                if self._payload_mass > 0.0:
                    return [f"payload model unavailable: {exc}"]
                self.get_logger().warn(
                    f"F107 static gate disabled (model build failed): {exc}",
                    throttle_duration_sec=60.0)
                return []
        import numpy as np  # type: ignore
        import pinocchio as pin  # type: ignore

        rated = [float(v) for v in self.get_parameter("joint_rated_torque").value]
        ratio = float(self.get_parameter("static_torque_margin_ratio").value)
        if len(rated) < len(self._joint_names):
            rated = rated + [rated[-1]] * (len(self._joint_names) - len(rated))
        q = np.zeros(self._pin_model.nq)
        for pos, qi in zip(positions, self._pin_q_idx):
            q[qi] = pos
        tau = pin.rnea(self._pin_model, self._pin_model.createData(), q,
                       np.zeros(self._pin_model.nv), np.zeros(self._pin_model.nv))
        viol: List[str] = []
        for i, jn in enumerate(self._joint_names):
            val = float(tau[self._pin_v_idx[i]])
            limit = rated[i] * ratio
            if abs(val) > limit:
                viol.append(
                    f"{jn} static torque {val:+.2f} N·m exceeds "
                    f"{limit:.2f} (rated {rated[i]:.1f} x {ratio:g})")
        return viol

    def _duty_prune(self, now: float) -> None:
        window = float(self.get_parameter("duty_window_s").value)
        while self._duty_segments and now - self._duty_segments[0][0] > window:
            self._duty_segments.popleft()

    def _duty_used_s(self, now: float) -> float:
        self._duty_prune(now)
        window = float(self.get_parameter("duty_window_s").value)
        used = 0.0
        for start, dur in self._duty_segments:
            # 段与当前窗口左沿重叠的部分才计入
            used += min(dur, start + dur - (now - window))
        return max(0.0, used)

    def _duty_check(self, duration_s: float) -> Tuple[bool, float, float]:
        """
        返回 (允许, 窗口内已用秒, 需冷却秒)。冷却秒 = 最早段滚出窗口、
        使 used+proposed <= max 所需等待时间。
        """
        if not self.get_parameter("duty_gate_enabled").value:
            return True, 0.0, 0.0
        now = time.monotonic()
        window = float(self.get_parameter("duty_window_s").value)
        max_s = float(self.get_parameter("duty_max_ratio").value) * window
        used = self._duty_used_s(now)
        if used + duration_s <= max_s:
            return True, used, 0.0
        # 需要等多久：从头弹出运动段，直到余量容得下 proposed
        need_cut = used + duration_s - max_s
        cooling = 0.0
        for start, dur in self._duty_segments:
            cut = min(dur, start + dur - (now - window))
            need_cut -= cut
            # 段末端越过窗口左沿所需的等待（不是越过 now）
            cooling = (start + dur) - (now - window)
            if need_cut <= 0.0:
                break
        return False, used, max(0.0, cooling)

    def _duty_record(self, duration_s: float) -> None:
        if not self.get_parameter("duty_gate_enabled").value or duration_s <= 0.0:
            return
        self._duty_segments.append((time.monotonic(), float(duration_s)))

    def _set_payload_cb(
        self,
        request: SetPayload.Request,
        response: SetPayload.Response,
    ) -> SetPayload.Response:
        mass = float(request.mass_kg)
        com = [float(v) for v in request.com_m]
        if len(com) != 3:
            response.success = False
            response.message = "com_m must have exactly 3 elements"
            return response
        rated_max = float(self.get_parameter("rated_payload_kg").value)
        if mass < 0.0 or mass > rated_max:
            response.success = False
            response.message = (
                f"payload mass {mass:.3f} kg out of range [0, {rated_max:.2f}]")
            return response
        old_mass, old_com = self._payload_mass, self._payload_com
        self._payload_mass = mass
        self._payload_com = com
        self._pin_ready = False
        # 已使能/有当前读数时，新负载必须在当前位形静态可行，否则回滚
        if self._have_js:
            viol = self._static_torque_violations(self._positions)
            if viol:
                self._payload_mass, self._payload_com = old_mass, old_com
                self._pin_ready = False
                response.success = False
                response.message = (
                    "payload rejected: current pose static gate: " + "; ".join(viol))
                return response
        response.success = True
        response.message = f"payload set to {mass:.3f} kg"
        self.get_logger().info(response.message)
        return response

    def _set_state(self, state: str, message: str = "") -> None:
        with self._lock:
            prev = self._state
            self._state = state
            if message:
                self._message = message
        self.get_logger().info(f"state -> {state}" + (f" ({message})" if message else ""))
        # 状态边沿必须即刻发布：INIT 等短寿命状态（sim 里仅几毫秒）在周期
        # status_hz 节拍间生灭，下游（DS4 白闪/ web / MQTT）永远收不到。LL-064。
        if hasattr(self, "_arm_status_pub"):
            self._arm_status_pub.publish(self._build_status_msg())
        # F90: 所有故障入口（FAULT 及过温保护 COOLING）都汇聚于此，
        # 只在进入 FAULT/COOLING 的边沿触发一次黑匣子落盘
        if state in (STATE_FAULT, STATE_COOLING) and prev != state:
            self._fire_blackbox_snapshot()

    def _fire_blackbox_snapshot(self) -> None:
        # F90: rosbag2 snapshot-mode 黑匣子——把录制器内存循环缓冲写成分片 bag。
        # fire-and-forget：未使能/录制器未发现时直接跳过，绝不阻塞故障处置路径。
        if not self.get_parameter("blackbox_snapshot_enabled").value:
            return
        if not self._snapshot_cli.service_is_ready():
            self.get_logger().info("blackbox recorder not available, skip snapshot")
            return

        def _done(fut) -> None:
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"blackbox snapshot call failed: {exc}")
                return
            if resp is not None and not resp.success:
                self.get_logger().warn("blackbox snapshot returned success=false")
            else:
                self.get_logger().info("blackbox snapshot flushed")

        self._snapshot_cli.call_async(Snapshot.Request()).add_done_callback(_done)

    def _build_status_msg(self):
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
        st.temperatures = [float(t) for t in self._temperatures]
        st.max_torques = [
            float(self._torque_stats.get(jn, {}).get("max_abs", 0.0))
            for jn in self._joint_names
        ]
        st.session_max_torques = [
            float(self._torque_session_max.get(jn, 0.0))
            for jn in self._joint_names
        ]
        # F160: 力矩分量分解（缺省 0；随 /a3/hardware/torque_components 更新）
        st.ff_gravity = [float(v) for v in self._ff_gravity]
        st.ff_coriolis = [float(v) for v in self._ff_coriolis]
        st.ff_inertia = [float(v) for v in self._ff_inertia]
        st.ff_total = [float(v) for v in self._ff_total]
        st.pd_feedback = [float(v) for v in self._pd_feedback]
        st.command_torque = [float(v) for v in self._command_torque]
        st.temp_warn = bool(self._temp_warn)
        return st

    def _publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    def _announce_startup_mode(self) -> None:
        """启动 2 s 后重发当前状态模式，覆盖订阅发现窗口（一次性）."""
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
        """
        轮询等待 future 完成.

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

    def _all_motors_enabled(self) -> bool:
        """所有有新鲜反馈的电机均已使能（且至少一路 fresh）."""
        fresh_enabled = [
            en for en, fr in zip(self._motor_enabled, self._motor_state_fresh) if fr
        ]
        return bool(fresh_enabled) and all(fresh_enabled)

    def _using_controller_switch(self) -> bool:
        return str(self.get_parameter("motor_service_backend").value) == "controller_switch"

    def _cm_switch(self, command: int) -> Tuple[bool, str]:
        """
        F75/F83: 标准 ros2_control 使能/失能编排.

        command: 1 enable(activate) / 2 reset(deactivate) / 3 set_zero（绝对编码
        帧无此操作，跳过返回成功）。

        Humble 下 switch_controller 不会自动激活 INACTIVE 启动的硬件（LL-086），
        必须显式驱动组件生命周期：
          enable  = set_hardware_component_state ACTIVE（on_activate 内执行
                    reset→反馈校验→清故障/RUN_MODE/enable→软启动，阻塞约 1 s）
                    → switch_controller activate（失败则把硬件退回 INACTIVE）
          disable = switch_controller deactivate
                    → set_hardware_component_state INACTIVE（on_deactivate 内
                    零增益刷新 + reset-all 自由滑行）
        """
        if command == 3:
            return True, "ros2_control absolute frame: set_zero skipped"
        activate = command == 1
        names = list(self.get_parameter("switch_controllers").value)
        hw_name = str(self.get_parameter("hardware_component_name").value)

        def set_hw(target_id: int, timeout: float) -> Tuple[bool, str]:
            if not self._wait_service(self._hw_state_cli, 3.0):
                return False, "set_hardware_component_state unavailable"
            self.get_logger().info(
                f"set_hardware_component_state: {hw_name} -> {target_id}")
            req = SetHardwareComponentState.Request()
            req.name = hw_name
            req.target_state = LifecycleState(id=target_id)
            future = self._hw_state_cli.call_async(req)
            if not self._wait_future(future, timeout):
                return False, "set_hardware_component_state timeout"
            res = future.result()
            if not bool(res.ok):
                return False, f"hardware {hw_name} -> state {target_id} rejected"
            return True, ""

        def do_switch(act: bool) -> Tuple[bool, str]:
            if not self._wait_service(self._switch_cli, 3.0):
                return False, "controller_manager switch service unavailable"
            req = SwitchController.Request()
            if act:
                req.activate_controllers = names
            else:
                req.deactivate_controllers = names
            # LL-146: deactivate 用 BEST_EFFORT——控制器已 inactive 时不拒绝（reset/失能
            # 幂等，过温保护重入或 FAULT 下重复失能不再 STRICT 拒绝）；activate 保持
            # STRICT（确保 arm+gripper 都成功激活，否则回退硬件）。
            req.strictness = (
                SwitchController.Request.STRICT if act
                else SwitchController.Request.BEST_EFFORT
            )
            self.get_logger().info(
                f"switch_controller: {'activate' if act else 'deactivate'} {names}")
            future = self._switch_cli.call_async(req)
            if not self._wait_future(future, 5.0):
                return False, "switch_controller timeout"
            res = future.result()
            if not bool(res.ok):
                return False, "switch_controller rejected (STRICT)"
            return True, ""

        if activate:
            ok, msg = set_hw(LifecycleState.PRIMARY_STATE_ACTIVE, 10.0)
            if not ok:
                return False, msg
            ok, msg = do_switch(True)
            if not ok:
                self.get_logger().error(
                    f"controllers failed to activate after hardware enable ({msg}); "
                    "returning hardware to INACTIVE")
                set_hw(LifecycleState.PRIMARY_STATE_INACTIVE, 5.0)
                return False, msg
            return True, "hardware active, controllers activated"

        ok, msg = do_switch(False)
        if not ok:
            return False, msg
        ok, msg = set_hw(LifecycleState.PRIMARY_STATE_INACTIVE, 5.0)
        if not ok:
            return False, msg
        return True, "controllers deactivated, hardware inactive"

    def _cm_freedrive_switch(self, enter: bool) -> Tuple[bool, str]:
        """
        F89b/F127: 标准栈自由拖动 = 一次原子 STRICT switch_controller 请求.

        enter: deactivate arm_controller(+gripper_controller，若 realtime active) +
               activate zero_torque_controller；
        exit:  反向恢复。switch_controller 在同一请求内完成互换，任一名无效即整体
        拒绝、控制器保持原状，不会出现「夹爪/臂既无闭环也无重力补偿」窗口。
        F127/LL-136: gripper 与 zero_torque 都 claim L7/effort，STRICT 互斥 → enter
        需先 probe gripper 是否 active 才能决定是否一并 swap 出去；INACTIVE 无法区分
        「被我们停」与「未配置」，故 exit 凭 _freedrive_gripper_swapped 标志恢复。
        """
        if not self._wait_service(self._switch_cli, 3.0):
            return False, "controller_manager switch service unavailable"
        arm = str(self.get_parameter("freedrive_arm_controller").value)
        free = str(self.get_parameter("freedrive_controller").value)
        grp = str(self.get_parameter("freedrive_gripper_controller").value)
        with_g = self._freedrive_gripper_swapped
        req = SwitchController.Request()
        if enter:
            if not self._wait_service(self._list_cli, 3.0):
                return False, "controller_manager list service unavailable"
            lf = self._list_cli.call_async(ListControllers.Request())
            if not self._wait_future(lf, 3.0):
                return False, "list_controllers timeout"
            with_g = False
            for c in lf.result().controller:
                if c.name == grp and c.state == "active":
                    with_g = True
                    break
            req.deactivate_controllers = [arm] + ([grp] if with_g else [])
            req.activate_controllers = [free]
        else:
            req.activate_controllers = [arm] + ([grp] if with_g else [])
            req.deactivate_controllers = [free]
        req.strictness = SwitchController.Request.STRICT
        self.get_logger().info(
            f"switch_controller freedrive {'enter' if enter else 'exit'}: "
            f"deactivate={list(req.deactivate_controllers)} "
            f"activate={list(req.activate_controllers)}")
        future = self._switch_cli.call_async(req)
        timeout = float(self.get_parameter("freedrive_switch_timeout_s").value)
        if not self._wait_future(future, timeout):
            return False, "switch_controller timeout"
        res = future.result()
        if not bool(res.ok):
            return False, "switch_controller rejected (STRICT)"
        self._freedrive_gripper_swapped = enter and with_g
        return True, ""

    def _motor_command(self, client, command: int) -> Tuple[bool, str]:
        if self._using_controller_switch():
            return self._cm_switch(command)
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

    def _call_hw_set_zero(self, motor_id: int = 255) -> Tuple[bool, str]:
        """F91 扩展：栈运行时设零（/a3/hardware/set_zero，电机未使能时放行）。"""
        if not self._wait_service(self._hw_set_zero_cli):
            return False, "/a3/hardware/set_zero unavailable"
        req = MotorIdCommand.Request()
        req.motor_id = motor_id
        future = self._hw_set_zero_cli.call_async(req)
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
        if self._feedback_stale:
            # F81: 插件已 freeze-hold，所有轨迹会被硬件丢弃，拒绝运动命令
            return False, "feedback stale: protective freeze-hold active"
        # F45/F53: DISABLED/COOLING/FAULT 下运动命令被拒（原 IDLE 允许 move_to 语义混乱；
        # FAULT 电机已复位关断，发轨迹只被静默接受、臂不动）
        if self._state in (
            STATE_INIT, STATE_TEACH, STATE_WAYPOINT_TEACH, STATE_AI, STATE_TRAJ, STATE_SERVO,
            STATE_SAFE_PARK, STATE_DISABLED, STATE_COOLING, STATE_FAULT,
        ):
            return False, f"state={self._state}"
        return True, ""

    def _two_point_trajectory(
        self,
        q0: List[float],
        q1: List[float],
        duration: float,
        joint_names: Optional[List[str]] = None,
    ) -> JointTrajectory:
        """
        F88: 起点(t=0)+终点(t=duration)两点轨迹；两端 velocity/acceleration=0.

        JTC VARIABLE_DEGREE_SPLINE 据此生成 quintic S 曲线（位置-only 两点会退化为
        匀速线性、端点速度跳变）。motor_protocol topic 后端忽略 v/a、按位置线性插值。

        F94: 在此集中强制速度地板时长（URDF velocity × joint_velocity_scale），
        所有调用点（jog/park/纠偏）均不得绕过。
        """
        names = list(joint_names or self._joint_names)
        floor, slowest = self._velocity_floor_duration(q0, q1, names)
        if floor > duration:
            self.get_logger().warn(
                f"F94 duration extended: {duration:.3f}s -> {floor:.3f}s "
                f"(slowest={slowest}, joint_velocity_scale="
                f"{float(self.get_parameter('joint_velocity_scale').value):g})",
                throttle_duration_sec=1.0,
            )
            duration = floor
        n = len(q0)
        traj = JointTrajectory()
        traj.joint_names = names
        p0 = JointTrajectoryPoint()
        p0.positions = [float(v) for v in q0]
        p0.velocities = [0.0] * n
        p0.accelerations = [0.0] * n
        traj.points.append(p0)
        p1 = JointTrajectoryPoint()
        p1.positions = [float(v) for v in q1]
        p1.velocities = [0.0] * n
        p1.accelerations = [0.0] * n
        p1.time_from_start = _duration(duration)
        traj.points.append(p1)
        return traj

    def _dispatch_trajectory(
        self, traj: JointTrajectory, duty_exempt: bool = False,
        l7_traj: Optional[JointTrajectory] = None,
    ) -> Tuple[bool, str]:
        """
        按 control_backend 下发轨迹：旧栈话题直发 / 标准栈 action.

        fjt_action 后端拆分：L1–L6 → arm_controller FJT；L7 →
        gripper_controller GripperCommand（取末点位置；末点 effort 非 0
        时作为 max_effort，否则用 gripper_default_effort）。新 goal 抢占
        旧 goal，与旧栈话题替换语义一致；异步发送，不阻塞服务回调。

        l7_traj 非空时（retime 排除 L7）：L7 用该独立序列（原始录制时间戳），
        而非从 traj 的 L7 列取点。

        F107: 下发前逐点静态重力矩门禁 + 占空比门禁。duty_exempt 用于
        SAFE_PARK 等安全回零路径（静态力矩门禁仍生效）。
        返回 (是否下发, 说明)。
        """
        # F107 静态力矩门禁：未覆盖关节沿用当前读数
        names = list(traj.joint_names)
        for point in traj.points:
            if not point.positions:
                continue
            full = list(self._positions)
            for jn, pos in zip(names, point.positions):
                if jn in self._joint_names:
                    full[self._joint_names.index(jn)] = float(pos)
            viol = self._static_torque_violations(full)
            if viol:
                msg = "static torque gate rejected: " + "; ".join(viol)
                self.get_logger().warn(msg)
                return False, msg

        duration = _tfs_seconds(traj.points[-1].time_from_start) if traj.points else 0.0
        if not duty_exempt:
            allowed, used_s, cooling_s = self._duty_check(duration)
            if not allowed:
                msg = (
                    f"duty gate rejected: {used_s:.0f}s motion in window, "
                    f"wait {cooling_s:.0f}s before retrying")
                self.get_logger().warn(msg)
                return False, msg
            self._duty_record(duration)

        backend = str(self.get_parameter("control_backend").value)
        self.get_logger().info(
            f"[F88] dispatch trajectory backend={backend} points={len(traj.points)} "
            f"joints={names}"
        )
        if backend != "fjt_action":
            self._traj_pub.publish(traj)
            return True, "dispatched (topic)"

        def project(joint_subset: List[str]) -> Optional[JointTrajectory]:
            idx = [names.index(n) for n in joint_subset if n in names]
            used = [n for n in joint_subset if n in names]
            if not idx:
                return None
            out = JointTrajectory()
            out.joint_names = used
            for p in traj.points:
                np = JointTrajectoryPoint()
                np.positions = [p.positions[i] for i in idx]
                if p.velocities:
                    np.velocities = [p.velocities[i] for i in idx]
                if p.accelerations:
                    np.accelerations = [p.accelerations[i] for i in idx]
                if p.effort:
                    np.effort = [p.effort[i] for i in idx]
                np.time_from_start = p.time_from_start
                out.points.append(np)
            return out

        arm_traj = project(JOINTS[:6])
        if arm_traj is not None:
            self._send_fjt_goal(self._fjt_arm_cli, arm_traj, "arm")
        if "L7_joint" in names or l7_traj is not None:
            # L7 夹爪按录制时间序列步进下发（GAC 抢占式），复现张合时序而非只取末点。
            # retime 排除 L7 时用独立 l7_traj（原始录制时间戳）。
            self._start_l7_sequence(l7_traj if l7_traj is not None else traj)
        return True, "dispatched (fjt_action)"

    def _send_gripper_goal(self, position: float, max_effort: float = 0.0) -> None:
        """Send a GripperCommand goal; newer goals preempt (GAC default)."""
        if max_effort <= 0.0:
            max_effort = float(self.get_parameter("gripper_default_effort").value)
        if not self._gripper_cli.server_is_ready() and \
                not self._gripper_cli.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                f"GripperCommand action 不可用，目标 {position:.3f} 未执行"
            )
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        future = self._gripper_cli.send_goal_async(goal)
        future.add_done_callback(self._on_gripper_goal_response)

    def _send_gripper_goal_limited(self, target: float) -> None:
        """L7 单目标下发 + 硬限速（≤ l7_max_velocity_rad_s）。

        GAC 无轨迹接口也没有速度上限，目标流由本层生成；对相邻 tick 目标做
        速率限幅，防止录制异常或外部目标造成的夹爪超速跳变。
        """
        max_v = float(self.get_parameter("l7_max_velocity_rad_s").value)
        now = time.monotonic()
        if self._l7_last_cmd_pos is None or max_v <= 0.0:
            self._l7_last_cmd_pos = float(target)
            self._l7_last_cmd_t = now
            self._send_gripper_goal(target)
            return
        dt = now - self._l7_last_cmd_t
        if dt <= 0.0:
            return
        delta = float(target) - self._l7_last_cmd_pos
        step = max_v * dt
        if abs(delta) > step:
            target = self._l7_last_cmd_pos + math.copysign(step, delta)
        # 目标未变（限幅后）→ 只推进时间基准、不重发 goal。否则恒定目标
        # （夹爪静止段）会被 20ms 定时器反复抢占 GAC，产生海量
        # "GripperCommand 未到位" 日志且 reached_goal 恒 false。
        self._l7_last_cmd_t = now
        if abs(float(target) - self._l7_last_cmd_pos) < 1e-4:
            return
        self._l7_last_cmd_pos = float(target)
        self._send_gripper_goal(target)

    def _on_gripper_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("GripperCommand goal 被拒绝")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_gripper_result)

    def _on_gripper_result(self, future) -> None:
        result = future.result().result
        if not result.reached_goal:
            self.get_logger().warn(
                f"GripperCommand 未到位: stalled={result.stalled} "
                f"position={result.position:.3f} effort={result.effort:.3f}"
            )

    def _send_fjt_goal(
        self, client: ActionClient, traj: JointTrajectory, label: str
    ) -> None:
        if not client.server_is_ready() and not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                f"FJT backend: {label} action 不可用，{len(traj.points)} 点轨迹未执行"
            )
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_fjt_goal_response(f, label))

    def _on_fjt_goal_response(self, future, label: str) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f"FJT backend: {label} goal 被拒绝")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_fjt_result(f, label))

    def _on_fjt_result(self, future, label: str) -> None:
        result = future.result().result
        code = result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            names = {
                v: k for k, v in vars(FollowJointTrajectory.Result).items()
                if k.isupper() and isinstance(v, int)
            }
            self.get_logger().warn(
                f"FJT backend: {label} 结束码 {names.get(code, code)}"
            )

    def _dispatch_l7_linear(self, target: float, speed: float = 0.5) -> float:
        """
        LL-077: move_group 只规划 arm 组（L1–L6），L7 单独补下发.

        fjt_action 后端走 GripperCommand action（末点位置）；topic 后端
        走两点轨迹（F88，motor_protocol 200 Hz 插值）。返回调度用估计
        时长；|Δ|≤1e-3 不下发返回 0.0。
        """
        delta = target - self._positions[6]
        if abs(delta) <= 1e-3:
            return 0.0
        g_dur = min(max(abs(delta) / speed, 1.0), 3.0)
        gtraj = self._two_point_trajectory(
            [self._positions[6]], [target], g_dur, joint_names=[JOINTS[6]])
        g_dur = _tfs_seconds(gtraj.points[1].time_from_start)
        ok, msg = self._dispatch_trajectory(gtraj)
        if not ok:
            self.get_logger().warn(f"L7 linear to {target:+.3f} not dispatched: {msg}")
            return 0.0
        return g_dur

    def _start_l7_sequence(self, traj: JointTrajectory) -> None:
        """按录制时间序列步进下发 L7 GripperCommand，复现夹爪张合时序。

        GAC（GripperActionController）无轨迹接口，只能发单目标；新 goal 抢占旧
        goal。故用一个周期定时器按轨迹点时间戳依次发目标，取代「只取末点」。
        相邻点线性插值 + l7_max_velocity_rad_s 硬限速，消除阶梯跳变。
        """
        if "L7_joint" not in traj.joint_names:
            return
        li = traj.joint_names.index("L7_joint")
        min_delta = float(self.get_parameter("l7_seq_min_delta_rad").value)
        seq: List[Tuple[float, float]] = []
        prev: Optional[float] = None
        n = len(traj.points)
        for i, p in enumerate(traj.points):
            if not p.positions or li >= len(p.positions):
                continue
            pos = float(p.positions[li])
            t = _tfs_seconds(p.time_from_start)
            if prev is None or abs(pos - prev) >= min_delta or i == n - 1:
                seq.append((t, pos))
                prev = pos
        if not seq:
            return
        self._stop_l7_sequence()
        self._l7_seq = seq
        self._l7_seq_idx = 0
        self._l7_seq_t0 = time.monotonic()
        self._l7_last_cmd_pos = None
        self._l7_last_cmd_t = 0.0
        tick = float(self.get_parameter("l7_seq_tick_s").value)
        self._l7_seq_timer = self.create_timer(
            tick, self._on_l7_seq_tick, callback_group=self._cb_group)

    def _stop_l7_sequence(self) -> None:
        if self._l7_seq_timer is not None:
            self._l7_seq_timer.cancel()
            self._l7_seq_timer = None
        self._l7_seq = []
        self._l7_seq_idx = 0

    def _on_l7_seq_tick(self) -> None:
        if not self._l7_seq:
            self._stop_l7_sequence()
            return
        seq = self._l7_seq
        elapsed = time.monotonic() - self._l7_seq_t0
        if elapsed >= seq[-1][0]:
            self._send_gripper_goal_limited(seq[-1][1])
            self._stop_l7_sequence()
            return
        if len(seq) == 2:
            # 两点轨迹（到首点/回 idle）→ quintic S 曲线：起止零速零加加速度，
            # 消除线性插值的起步/结束速度阶跃。多段录制序列仍走下方线性插值。
            t0, p0 = seq[0]
            t1, p1 = seq[1]
            u = (elapsed - t0) / max(t1 - t0, 1e-6)
            u = min(max(u, 0.0), 1.0)
            s = 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5
            target = p0 + (p1 - p0) * s
        else:
            # 相邻点线性插值：目标连续移动，消除阶梯跳变（夹爪无需 Ruckig）。
            i = self._l7_seq_idx
            while i + 1 < len(seq) and seq[i + 1][0] <= elapsed:
                i += 1
            self._l7_seq_idx = i
            t0, p0 = seq[i]
            if i + 1 < len(seq):
                t1, p1 = seq[i + 1]
                target = p0 + (p1 - p0) * (elapsed - t0) / max(t1 - t0, 1e-6)
            else:
                target = p0
        self._send_gripper_goal_limited(target)

    def _safe_park_then_disable(self) -> Tuple[bool, str]:
        """
        F40: 平滑回 home → 连续确认收敛 → 失能（同步阻塞，仿 _init_cb 轮询先例）.

        发布 home 轨迹抢占活跃轨迹（执行层 OnTrajectory 天然支持替换），
        SAFE_PARK 期间拒绝新运动指令；park 超时 → FAULT 且不 reset（保持使能，
        停在半途，需人工介入）。
        """
        home = self._home_pose()
        confirm_s = float(self.get_parameter("disable_home_confirm_s").value)
        timeout_s = float(self.get_parameter("disable_park_timeout_s").value)

        # F67: 优先 move_group（L1–L6 回 home）
        duration = 0.0
        planned = False
        if bool(self.get_parameter("goto_use_moveit").value):
            planned, duration, msg = self._moveit_move(
                home[:6], JOINTS[:6], "safe park -> idle", state=STATE_SAFE_PARK
            )
            if not planned:
                self.get_logger().warn(f"safe park: {msg} -- local linear fallback")

        # move_group 只规划 arm 组（L1–L6）；L7（夹爪）单独走 GripperCommand 回 home
        # （6J 变体无 L7：home 只有 6 个元素，取值前必须判 n_joints）
        if planned and self._n_joints >= 7:
            duration = max(duration, self._dispatch_l7_linear(home[6]))

        if not planned:
            duration = max(
                float(self.get_parameter("disable_home_duration_s").value),
                float(self.get_parameter("move_to_min_duration_s").value),
            )
            q0 = list(self._positions)
            traj = self._two_point_trajectory(q0, home, duration)
            duration = _tfs_seconds(traj.points[1].time_from_start)
            self._publish_mode("TRAJ_RUNNING")
            ok, dmsg = self._dispatch_trajectory(traj, duty_exempt=True)
            if not ok:
                self._set_state(
                    STATE_FAULT, f"safe park rejected by static gate: {dmsg}")
                return False, f"safe park rejected: {dmsg}"
            self._set_state(STATE_SAFE_PARK, f"safe park -> idle ({duration:.1f}s, 2 pts)")

        self._traj_done_at = 0.0  # 防旧 TRAJ 时间戳在 SAFE_PARK 中误触发回 READY
        if planned:
            self._set_state(STATE_SAFE_PARK, f"safe park -> idle (move_group, {duration:.1f}s)")
        t0 = time.monotonic()
        settle_tol = float(self.get_parameter("disable_home_settle_tol_rad").value)
        settle_vel = float(self.get_parameter("disable_home_settle_vel_rad_s").value)
        residual_tol = float(self.get_parameter("disable_park_residual_tol_rad").value)
        hard_floor = float(self.get_parameter("disable_park_residual_hard_floor_rad").value)
        plateau_s = float(self.get_parameter("disable_park_plateau_s").value)
        no_progress = float(self.get_parameter("disable_park_no_progress_rad").value)
        plateau_vel = float(self.get_parameter("disable_park_plateau_vel_rad_s").value)

        def _err_speed() -> Tuple[float, float]:
            err = max(abs(p - h) for p, h in zip(self._positions, home))
            speed = max(abs(v) for v in self._velocities)
            return err, speed

        def settled() -> bool:
            # F75: 位置 AND 速度双落定（宽 tol=0.15 会在臂仍运动时放行失能，LL-077）
            err, speed = _err_speed()
            return err <= settle_tol and speed <= settle_vel

        converge_start = 0.0
        corrective_sent = False
        # F130 plateau gate：静止 + 小残差 + 持续无进展（物理止挡/下垂）才放行
        plateau_since = 0.0
        plateau_ref_err = -1.0
        allow_residual = False
        while time.monotonic() - t0 < duration + timeout_s and rclpy.ok():
            # F51/LL-039: 电机已带外失能 → park 不可能收敛，立即退出并同步
            # ros2_control teardown（保持"已失能"事实，避免控制器残留 active）
            if self._pending_unexpected_disable:
                why = self._pending_unexpected_disable
                self._handle_oob_disable(f"safe park aborted：{why}")
                return False, f"safe park aborted: {why}"
            # move_group 轨迹执行异常未落定时，补一条两点纠偏轨迹（仅一次）
            if not corrective_sent and time.monotonic() - t0 > duration + 1.0 \
                    and not settled():
                gap, _ = _err_speed()
                c_dur = min(max(gap / 0.3, 2.0), 6.0)
                c_traj = self._two_point_trajectory(
                    list(self._positions), home, c_dur)
                c_dur = _tfs_seconds(c_traj.points[1].time_from_start)
                ok, cmsg = self._dispatch_trajectory(c_traj, duty_exempt=True)
                duration = (time.monotonic() - t0) + c_dur
                if not ok:
                    self.get_logger().warn(f"park corrective not dispatched: {cmsg}")
                corrective_sent = True
                plateau_since = 0.0  # 纠偏后重新观测稳态，避免误判
            err, speed = _err_speed()
            if settled():
                if converge_start == 0.0:
                    converge_start = time.monotonic()
                elif time.monotonic() - converge_start >= confirm_s:
                    break
            else:
                converge_start = 0.0
                # F130: 纠偏后若已静止、残差在容限内且位置不再改善 → 物理受限的稳态。
                # 速度用独立 plateau_vel（容忍 idle 保持时反馈噪声），非严格 settle_vel。
                if corrective_sent and err <= residual_tol and speed <= plateau_vel:
                    now = time.monotonic()
                    if plateau_since == 0.0:
                        plateau_since = now
                        plateau_ref_err = err
                    elif abs(plateau_ref_err - err) <= no_progress:
                        if now - plateau_since >= plateau_s:
                            allow_residual = True
                            break
                    else:  # 仍在改善（还在往 idle 走）→ 重置，继续等
                        plateau_since = now
                        plateau_ref_err = err
                else:
                    plateau_since = 0.0
            time.sleep(0.05)

        err, speed = _err_speed()
        residual_ok = allow_residual and err <= residual_tol and err <= hard_floor
        if not (settled() or residual_ok):
            # 残差超硬地板（明显未到位）或仍在运动 → FAULT 保持使能，需人工介入
            self._set_state(STATE_FAULT, "safe park timeout: not settled at idle, still enabled")
            return False, "safe park timeout: not settled at idle, still enabled"
        if allow_residual and not settled():
            worst_jn = self._joint_names[
                max(range(len(home)), key=lambda i: abs(self._positions[i] - home[i]))]
            self.get_logger().info(
                f"safe park: idle physically unreachable, settle at residual "
                f"err={err:.3f} rad speed={speed:.3f} (worst={worst_jn}); disable anyway")
        ok, msg = self._motor_command(self._reset_cli, 2)
        if not ok:
            # reset 被拒（如 gate 互锁）：臂已在 idle 位（安全），回 READY 待人工
            self._set_state(STATE_READY, f"parked at idle but disable refused: {msg}")
            return False, f"{msg} (parked at idle; stop power sequence first)"
        self._set_state(STATE_DISABLED, "safe park -> disabled")
        self._publish_mode("IDLE")
        return True, f"safe park -> disabled ({time.monotonic() - t0:.1f}s)"

    def _handle_oob_disable(self, why: str) -> None:
        """F51/LL-039：带外失能收敛——FSM 置 DISABLED 的同时必须同步 ros2_control
        实际状态：deactivate 残余 active 的控制器 + 硬件退回 INACTIVE。

        否则状态簿与 controller_manager 不一致（控制器仍 active、电机已离线），
        下次 enable 的 STRICT activate 会因 "is not inactive" 被整体拒绝并回退
        硬件，形成「DISABLED 但无法重新使能」死锁。电机已带外失能时复位帧可能
        无响应，teardown 全程 best-effort，失败只告警、不改变 DISABLED 事实。"""
        self._pending_unexpected_disable = ""
        self._all_disabled_since = 0.0
        self._traj_done_at = 0.0
        self._jogging = False
        self._publish_mode("IDLE")
        self._set_state(
            STATE_DISABLED,
            f"{why} → DISABLED（恢复：/a3/arm/enable；执行层 F51 使能会重锚到当前位姿）")
        self.get_logger().error(f"[arm_controller] {why} → state=DISABLED")
        ok, msg = self._motor_command(self._reset_cli, 2)
        if not ok:
            self.get_logger().warn(
                f"oob disable teardown best-effort failed: {msg}")

    def _check_cooling(self) -> Tuple[bool, str]:
        """
        F44: COOLING 下重新使能门禁——全 fresh 关节 < protect−hysteresis 才放行.

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
        """F44: 超温保护——平滑回 home → 失能 → COOLING（在状态发布节拍执行）."""
        # LL-146: ReentrantCallbackGroup 下防重入。safe-park 阻塞期间 move_group error -4
        # 会短暂置 READY，绕过下方 SAFE_PARK 守卫，重入会对已失能控制器再 deactivate。
        if self._temp_protect_busy:
            return
        self._temp_protect_busy = True
        try:
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
        finally:
            self._temp_protect_busy = False

    def _handle_motor_fault(self) -> None:
        """硬故障（驱动/磁编码/堵转过载）→ 紧急 reset + FAULT."""
        jn, mask = self._pending_fault
        self.get_logger().error(f"motor fault: joint={jn} mask=0x{mask:X} -> emergency reset")
        ok, msg = self._motor_command(self._reset_cli, 2)
        if ok:
            self._set_state(STATE_FAULT, f"motor fault: {jn} mask=0x{mask:X}, disabled")
        else:
            self._set_state(
                STATE_FAULT,
                f"motor fault: {jn} mask=0x{mask:X}, reset refused: {msg}")

    def _handle_soft_fault(self, jn: str, mask: int) -> None:
        """软故障（欠压/过温/未标定）→ safe park → disable，可自恢复不 reset。"""
        self.get_logger().warn(
            f"soft motor fault: joint={jn} mask=0x{mask:X} -> safe park + disable")
        ok, msg = self._safe_park_then_disable()
        if ok:
            self._set_state(
                STATE_DISABLED, f"soft motor fault: {jn} mask=0x{mask:X}, disabled")
        else:
            # park 超时→FAULT / reset 被拒→READY 已由 _safe_park_then_disable 置位
            self.get_logger().error(
                f"soft motor fault: {jn} mask=0x{mask:X}, safe park failed: {msg}")

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

    def _on_torque_components(self, msg: TorqueComponents) -> None:
        name_to_idx = {n: i for i, n in enumerate(msg.joint_names)}
        for i, jn in enumerate(self._joint_names):
            j = name_to_idx.get(jn)
            if j is None:
                continue
            if j < len(msg.gravity):
                self._ff_gravity[i] = float(msg.gravity[j])
            if j < len(msg.coriolis):
                self._ff_coriolis[i] = float(msg.coriolis[j])
            if j < len(msg.inertia):
                self._ff_inertia[i] = float(msg.inertia[j])
            if j < len(msg.feedforward):
                self._ff_total[i] = float(msg.feedforward[j])
            if j < len(msg.pd_feedback):
                self._pd_feedback[i] = float(msg.pd_feedback[j])
            if j < len(msg.command):
                self._command_torque[i] = float(msg.command[j])

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

        # F52 档位配对自检（只告警不改行为）：执行层 /joint_states 的关节数与本节点
        # joint_names 不一致 = motor_map_*j.yaml 与 arm_controller_*j.yaml 配对错误
        # （或执行层档位未生效），继续运行会让目标轨迹与电机映射错位。
        if not self._profile_check_done:
            self._profile_check_done = True
            observed = len(msg.name)
            if observed != self._n_joints:
                self.get_logger().error(
                    f"F52 档位配对不一致：arm_controller 配了 {self._n_joints} 关节 "
                    f"{list(self._joint_names)}，但 /joint_states 上报 {observed} 关节 "
                    f"{list(msg.name)} —— 请检查 motor_map_*j.yaml 与 "
                    "arm_controller_*j.yaml 是否配对（7J/6J/5J 三件套）"
                )

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
        """
        F43/F44: 电机原始状态——最大力矩跟踪 + 温度/故障监视.

        无反馈关节温度在 C++ 侧被置 0.0（非 NaN），所有温度判断必须 fresh 门控，
        否则断连会被误判「已冷却」放行使能（LL-011 教训）。6J 臂 MotorStates 仍
        含 7 条（motor 7 fresh=false），按 motor_id-1 索引并越界保护。
        """
        n = self._n_joints
        temps = [0.0] * n
        fresh = [False] * n
        enabled = [False] * n
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
                enabled[idx] = bool(st.enabled)
        self._temperatures = temps
        self._temp_fresh = fresh
        self._motor_enabled = enabled
        self._motor_state_fresh = fresh

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

        # COOLING 降温恢复：全 fresh 关节 < protect−hysteresis 时自动转 DISABLED
        # （实时反映已降温、可重新使能；灯效红双闪 → 橙）。
        if self._state == STATE_COOLING:
            cooled, _ = self._check_cooling()
            if cooled:
                self._set_state(STATE_DISABLED, "overtemp cooled down")

        # 电机故障监视（fault_mask = 原始 fault_code，bit 定义见 FAULT_* 常量）
        if self.get_parameter("fault_mask_reset_on_fault").value and self._state in (
            STATE_READY, STATE_TRAJ, STATE_IDLE,
        ):
            hard: Optional[Tuple[str, int]] = None
            soft: Optional[Tuple[str, int]] = None
            for st in msg.states:
                if not st.fresh or int(st.fault_mask) == 0:
                    continue
                idx = int(st.motor_id) - 1
                jn = self._joint_names[idx] if 0 <= idx < n else f"motor{st.motor_id}"
                mask = int(st.fault_mask)
                if mask & FAULT_HARD_MASK:
                    hard = (jn, mask)
                    break
                if soft is None and (mask & FAULT_SOFT_MASK):
                    soft = (jn, mask)
            if hard is not None:
                self._pending_fault = hard
                self._fault_reset_pending = True
                self._soft_fault_since = None
            elif soft is not None:
                # 软故障计时：持续超 soft_fault_grace_s 才降级，给瞬时压降等自恢复留时间
                if self._soft_fault_since is None:
                    self._soft_fault_since = time.monotonic()
                    self._soft_fault_joint = soft[0]
                    self._soft_fault_mask = soft[1]
            else:
                self._soft_fault_since = None

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
        """
        F51/LL-039：消费看门狗「已确认」故障——只认 TRIGGERED（瞬时 PENDING 不动作）.

        仅在使能期望态挂起待处置标志：DISABLED 等非使能态下收到的 TRIGGERED 是
        上一故障窗口的余流（控制器进 DISABLED 后看门狗仍会补发约 clear_hold 的
        TRIGGERED），挂起会变成跨周期陈旧标志，在下次 enable→READY 的首个状态
        发布 tick 被误消费（LL-072）。
        """
        if not self.get_parameter("unexpected_disable_guard").value:
            return
        if msg.status == "TRIGGERED" and msg.fault == "UNEXPECTED_DISABLE":
            if self._state in (STATE_READY, STATE_TRAJ, STATE_SAFE_PARK):
                if not self._pending_unexpected_disable:
                    self._pending_unexpected_disable = "看门狗确认 UNEXPECTED_DISABLE"
        elif msg.status == "OK" and self._state == STATE_DISABLED:
            self._pending_unexpected_disable = ""

    def _on_feedback_stale(self, msg: Bool) -> None:
        # F81: 硬件插件整臂 freeze-hold 锁存；边沿日志即可，运动门禁在 _can_move
        was = self._feedback_stale
        self._feedback_stale = bool(msg.data)
        if self._feedback_stale and not was:
            self.get_logger().error(
                "[arm_controller] feedback stale -> hardware freeze-hold, blocking new moves")
        elif was and not self._feedback_stale:
            self.get_logger().info("[arm_controller] feedback recovered, freeze-hold cleared")

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

        # F51/LL-039: 电机被带外失能 → 立刻离开 READY/TRAJ，并同步 ros2_control
        # teardown（不再保留保持目标与后续指令）
        if self._pending_unexpected_disable and self._state in (
                STATE_READY, STATE_TRAJ, STATE_SAFE_PARK):
            self._handle_oob_disable(self._pending_unexpected_disable)

        # F44: 超温/故障保护在状态节拍执行（订阅回调只置 pending 标记）
        if self._temp_protect_pending:
            self._temp_protect_pending = False
            self._trigger_temp_protect()
        if self._fault_reset_pending:
            self._fault_reset_pending = False
            self._handle_motor_fault()
        if self._soft_fault_since is not None:
            grace = float(self.get_parameter("soft_fault_grace_s").value)
            if time.monotonic() - self._soft_fault_since >= grace:
                jn, mask = self._soft_fault_joint, self._soft_fault_mask
                self._soft_fault_since = None
                self._handle_soft_fault(jn, mask)

        # F43: 力矩统计节流落盘
        if self._torque_stats_dirty and time.monotonic() - self._torque_stats_saved_at >= float(
            self.get_parameter("torque_stats_save_interval_s").value
        ):
            self._save_torque_stats()

        self._arm_status_pub.publish(self._build_status_msg())

    # ------------------------------------------------------------------ services

    def _init_cb(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._state in (STATE_TRAJ, STATE_SERVO, STATE_TEACH, STATE_WAYPOINT_TEACH, STATE_AI, STATE_SAFE_PARK):
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

        if self._using_controller_switch():
            # F75/F91：标准栈 init = 设零（绝对编码帧写当前位姿为零点），不带使能；
            # 使能仍走 L3 / /a3/arm/enable。设零前电机须未使能（health_node active_ 互锁）。
            if self._state == STATE_READY:
                resp.success = False
                resp.message = (
                    "arm enabled (READY) — disable first (/a3/arm/disable) "
                    "before set_zero"
                )
                return resp
            self._set_state(STATE_INIT, "init: set_zero")
            ok, msg = self._call_hw_set_zero(255)
            if not ok:
                self._set_state(STATE_FAULT, f"set_zero failed: {msg}")
                resp.success = False
                resp.message = msg
                return resp
            self._set_state(
                STATE_DISABLED, "init ok: zero set, motors not enabled")
            resp.success = True
            resp.message = (
                "init ok: zero set (not enabled); enable via /a3/arm/enable"
            )
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
        self.get_logger().info("enable service called")
        if self._state in (STATE_TRAJ, STATE_SERVO, STATE_TEACH, STATE_WAYPOINT_TEACH, STATE_AI, STATE_SAFE_PARK):
            resp.success = False
            resp.message = f"busy in state={self._state}"
            return resp
        # LL-151: READY 下重复 enable 必须幂等返回，不能重走 hardware activate +
        # switch_controller activate——arm_controller 已 active 时 STRICT 拒绝会触发
        # 「硬件退回 INACTIVE → 电机 reset 失能 → 掉臂」（2026-10 真机事故）。
        if self._state == STATE_READY:
            resp.success = True
            resp.message = "already enabled (READY)"
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
        # LL-072：显式 enable 必然代表操作员确认当前无故障——丢弃上一故障窗口
        # 残留的待处置标志，防止陈旧 pending 在 READY 首个 tick 把臂打回 DISABLED。
        self._pending_unexpected_disable = ""
        self._all_disabled_since = 0.0
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
        # F107: 当前位形（含额定负载）静态重力矩门禁
        if self._have_js:
            viol_t = self._static_torque_violations(self._positions)
            if viol_t:
                resp.success = False
                resp.message = "static torque check failed: " + "; ".join(viol_t)
                return resp
        # F60：电源序列 EnableInit 在门禁关闭的预检阶段已使能全部电机。gate 打开
        # 后 F32 互锁会拒绝 /a3/motor/enable 调试写——电机既已使能就无需再写：
        # F48 限位检查已过、执行层在 SoftStand 锚定当前反馈位、看门狗已在
        # none→all 使能沿重基准保持参照，直接转 READY。
        if not self._using_controller_switch() and self._all_motors_enabled():
            self._set_state(
                STATE_READY,
                "motors already enabled by power sequence EnableInit -> READY")
            self.get_logger().info(
                "enable: all motors already enabled by power sequence "
                "(skip motor debug op blocked by F32 gate) -> READY")
            resp.success = True
            resp.message = "already enabled by power sequence"
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
        """
        F40: 失能保护——不在 home 容差内先平滑回 home 再失能，避免掉臂.

        F119: 任意模式软失能——TEACH / SERVO / ZERO_TORQUE / GRAVITY_COMP 先退出
        （stop_teach / stop_servo / zero_torque/stop / gravity_compensation/stop）再走
        下方原流程；退出等待超时（如操作员一直按着 servo 输入）→ 拒绝并给可执行文案，
        不改状态。此时仅 INIT/AI 视为 busy。
        """
        # F119: TEACH / WAYPOINT_TEACH 先停（自动存档是接受的副作用），退出 ZERO_TORQUE
        if self._state in (STATE_TEACH, STATE_WAYPOINT_TEACH):
            tr_resp = self._stop_teach_cb(Trigger.Request(), Trigger.Response())
            if not tr_resp.success:
                resp.success = False
                resp.message = f"cannot exit {self._state}: {tr_resp.message}"
                return resp
        # F119: BLOCKED_MODES 先退出对应模式，再等 _mode 离开（servo 桥 0.5s 超时 /
        # zero_torque/stop / gravity stop 各自发 IDLE）；超时 → 拒绝，不静默继续
        if self._mode in BLOCKED_MODES:
            mode = self._mode
            if mode == "SERVO":
                ok, msg = self._call_trigger(self._servo_stop_cli, "servo/stop")
            elif mode == "ZERO_TORQUE":
                ok, msg = self._call_trigger(self._zt_stop_cli, "zero_torque/stop")
            else:  # GRAVITY_COMP
                ok, msg = self._call_trigger(
                    self._gravity_stop_cli, "gravity_compensation/stop")
            if not ok:
                resp.success = False
                resp.message = (
                    f"exit mode={mode} failed: {msg}; 紧急失能 /a3/motor/reset"
                )
                return resp
            timeout = float(self.get_parameter("disable_mode_exit_timeout_s").value)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and self._mode in BLOCKED_MODES and rclpy.ok():
                time.sleep(0.05)
            if self._mode in BLOCKED_MODES:
                resp.success = False
                resp.message = (
                    f"mode={self._mode} 未在 {timeout:.1f}s 内退出（可能仍在操作 servo "
                    "输入）；松开输入后再试，紧急失能 /a3/motor/reset"
                )
                return resp
        if self._state in (STATE_INIT, STATE_AI):
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

        # F53/LL-045 说明：BLOCKED_MODES（零力矩/重力补偿）下执行层丢弃轨迹，直接
        # safe park 或 reset 会让臂在悬浮中干等 / 重力下垂落。F119 前序已在
        # _state/mode 判断前退出这些模式；此处仅是防御性兜底（正常流程到不了）。
        if self._mode in BLOCKED_MODES:
            resp.success = False
            resp.message = (
                f"mode={self._mode} 仍处于受限模式（F119 前序退出失败）；"
                f"先退出该模式再 disable，紧急失能 /a3/motor/reset"
            )
            return resp

        # F81: 反馈丢失时插件在冻结位整臂 freeze-hold。safe park 无法收敛
        # （硬件丢弃一切轨迹），只会等超时进 FAULT。已在 home 位时直接 reset
        # 安全；否则拒绝并给出可执行下一步，由操作员决定恢复反馈或紧急失能。
        if self._feedback_stale:
            tol = float(self.get_parameter("disable_home_tol_rad").value)
            at_home, _ = self._at_home(tol)
            if at_home:
                ok, msg = self._motor_command(self._reset_cli, 2)
                if ok:
                    self._set_state(STATE_DISABLED, "disabled while stale at idle")
                    self._publish_mode("IDLE")
                resp.success = ok
                resp.message = msg
                return resp
            resp.success = False
            resp.message = (
                "feedback stale: arm is in protective freeze-hold away from home; "
                "restore feedback then disable, or use /a3/motor/reset for emergency "
                "(arm may drop under gravity)"
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

        # IDLE（上电未使能）：直达 reset。FAULT：LL-146 区分「已失能/已在 idle」与
        # 「仍使能且偏离 idle」——后者先 safe-park 再 reset，避免半抬位直接失能掉臂。
        if self._state == STATE_IDLE:
            ok, msg = self._motor_command(self._reset_cli, 2)
            if ok:
                self._set_state(STATE_DISABLED, "disabled")
                self._publish_mode("IDLE")
            resp.success = ok
            resp.message = msg
            return resp

        if self._state == STATE_FAULT:
            tol = float(self.get_parameter("disable_home_tol_rad").value)
            at_home = False
            if self._have_js:
                at_home, _ = self._at_home(tol)
            if not self._have_js or at_home:
                # 读不到反馈或已在 idle → 直达 reset 安全
                ok, msg = self._motor_command(self._reset_cli, 2)
                if ok:
                    self._set_state(STATE_DISABLED, "disabled")
                    self._publish_mode("IDLE")
                resp.success = ok
                resp.message = msg
                return resp
            # 仍使能且偏离 idle → 先 safe-park（内部已处理「带外失能」与超时回 FAULT）
            ok, msg = self._safe_park_then_disable()
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

    # ── F134: pilz Sequence 整序列一次规划/执行 ─────────────────────────────
    def _static_segment_violation(
        self,
        q0_arm: List[float],
        qt_arm: List[float],
        group_joints: List[str],
        seg_i: int = 0,
        samples: int = 21,
    ) -> Optional[str]:
        """
        F134/F107: 单段预检——目标关节须在自身限位内（pilz 拒绝越限 goal
        constraints，error -16），再沿关节直线采样静态重力矩（默认 21 点）。
        选点阶段可用 samples=9 快速过滤（详细门禁仍在平滑/序列链预检做 21 点）。
        拒绝返原因，否则 None。
        """
        if not self._pin_ready:
            try:
                self._ensure_pin_model()
            except Exception:  # noqa: BLE001
                self._pin_ready = False
        if self._pin_ready:
            for jn, tv in zip(group_joints, qt_arm):
                jid = self._pin_model.getJointId(jn)
                qi = int(self._pin_model.idx_qs[jid])
                lo = float(self._pin_model.lowerPositionLimit[qi])
                hi = float(self._pin_model.upperPositionLimit[qi])
                if float(tv) < lo or float(tv) > hi:
                    return (
                        f"goal limits gate rejected at segment {seg_i + 1}: "
                        f"{jn} {float(tv):+.3f} outside [{lo:.3f},{hi:.3f}]")
        samples = max(3, int(samples))

        def seg_full(frac: float) -> List[float]:
            full = list(self._positions)
            for jn, tv in zip(group_joints, qt_arm):
                ci = self._joint_names.index(jn)
                qi = q0_arm[group_joints.index(jn)]
                full[ci] = qi + (float(tv) - qi) * frac
            return full

        for k in range(samples + 1):
            viol = self._static_torque_violations(seg_full(k / samples))
            if viol:
                return (
                    f"static torque gate rejected at segment {seg_i + 1}: "
                    + "; ".join(viol))
        return None

    def _sequence_gate_check(
        self, targets: List[List[float]], group_joints: List[str]
    ) -> Optional[str]:
        """F134: Sequence 下发前逐段 F107 预检（静态重力矩 + 占空比），拒绝返原因."""
        if not self._have_js:
            return None
        qcur = [
            self._positions[self._joint_names.index(jn)] for jn in group_joints]
        for seg_i, qt in enumerate(targets):
            reject = self._static_segment_violation(
                qcur, qt, group_joints, seg_i)
            if reject:
                return reject
            qcur = list(qt)
        est_total = 0.0
        q0 = [
            self._positions[self._joint_names.index(jn)] for jn in group_joints]
        for qt in targets:
            d, _slow = self._velocity_floor_duration(q0, list(qt), group_joints)
            est_total += d
            q0 = list(qt)
        allowed, used_s, cooling_s = self._duty_check(max(est_total, 0.1))
        if not allowed:
            return (
                f"duty gate rejected: {used_s:.0f}s motion in window, "
                f"wait {cooling_s:.0f}s")
        return None

    def _fk_ee_xyz(self, positions7: List[float]) -> Optional[List[float]]:
        """F134: pinocchio end_effector 世界系 xyz；模型不可用返回 None."""
        try:
            if not self._pin_ready:
                self._ensure_pin_model()
        except Exception:  # noqa: BLE001
            return None
        import numpy as np  # type: ignore
        import pinocchio as pin  # type: ignore

        q = np.zeros(self._pin_model.nq)
        for pos, qi in zip(positions7, self._pin_q_idx):
            q[qi] = pos
        data = self._pin_model.createData()
        pin.forwardKinematics(self._pin_model, data, q)
        fid = self._pin_model.getFrameId("end_effector")
        pin.updateFramePlacement(self._pin_model, data, fid)
        return [
            float(data.oMf[fid].translation[0]),
            float(data.oMf[fid].translation[1]),
            float(data.oMf[fid].translation[2]),
        ]

    def _fk_ee_orientation(self, positions6: List[float]):
        """Pinocchio end_effector 世界系姿态（3x3 旋转矩阵）；模型不可用返回 None。"""
        try:
            if not self._pin_ready:
                self._ensure_pin_model()
        except Exception:  # noqa: BLE001
            return None
        import numpy as np  # type: ignore
        import pinocchio as pin  # type: ignore

        q = np.zeros(self._pin_model.nq)
        for pos, qi in zip(positions6, self._pin_q_idx[:6]):
            q[qi] = pos
        data = self._pin_model.createData()
        pin.forwardKinematics(self._pin_model, data, q)
        fid = self._pin_model.getFrameId("end_effector")
        pin.updateFramePlacement(self._pin_model, data, fid)
        return np.array(data.oMf[fid].rotation, dtype=float)

    @staticmethod
    def _pose_ref_rotation(pose: Optional[dict]):
        """F176: 从 pose.orientation（四元数 x,y,z,w）构造 3x3 旋转矩阵；无/异常返回 None。

        供 hold_orientation 段使用——把「示教姿态」作为 6D IK 的姿态目标（_solve_ik_position
        的 ref_orientation），而不是只求末端 xyz 到位导致腕部横着压。
        """
        ori = (pose or {}).get("orientation") if isinstance(pose, dict) else None
        if not isinstance(ori, dict):
            return None
        try:
            import numpy as np  # type: ignore
            import pinocchio as pin  # type: ignore

            quat = pin.Quaternion(float(ori["x"]), float(ori["y"]),
                                  float(ori["z"]), float(ori["w"]))
            return np.array(quat.normalized().matrix(), dtype=float)
        except Exception:  # noqa: BLE001
            return None

    def _solve_ik_position(
        self, pose: dict, ref_orientation=None, seed: Optional[List[float]] = None,
    ) -> Optional[List[float]]:
        """Pinocchio IK，返回 L1..L6 关节角或 None。

        默认 position-only（姿态自由）：用于轨迹流（软件编排），目标点只给末端
        xyz，姿态不可达时不强求（弹琴只要求末端点到达键位）。
        ref_orientation 非 None（3x3 旋转矩阵）时改为 6D IK：末端姿态软约束到
        ref_orientation（log3 旋转误差），用于 via 过点保持相邻按压点的腕部姿态、
        避免 L4/L5 大幅摆动导致关节路径绕远路（弹琴提速关键）。
        seed 非 None（6 关节角）时用作迭代初值，默认用当前关节实测值；F173 多点
        巡游用上一点解作种子，保证多解时腕部连续。
        """
        try:
            if not self._pin_ready:
                self._ensure_pin_model()
            if not self._pin_ready:
                return None
        except Exception:  # noqa: BLE001
            return None
        from scipy.optimize import least_squares
        import numpy as np
        import pinocchio as pin  # type: ignore

        pos = pose.get("position") or {}
        try:
            target = np.array([float(pos["x"]), float(pos["y"]), float(pos["z"])])
        except (KeyError, TypeError):
            return None

        model = self._pin_model
        q_idx = self._pin_q_idx[:6]
        lo = [float(model.lowerPositionLimit[qi]) + 0.05 for qi in q_idx]
        hi = [float(model.upperPositionLimit[qi]) - 0.05 for qi in q_idx]
        x0_src = list(seed[:6]) if seed and len(seed) >= 6 else [
            float(self._positions[i]) for i in range(6)]
        x0 = [min(max(float(x0_src[i]), lo[i]), hi[i]) for i in range(6)]
        fid = model.getFrameId("end_effector")
        ref_R = None if ref_orientation is None else np.asarray(ref_orientation, dtype=float)
        data = model.createData()  # 复用，避免每次迭代重建（6D IK 迭代次数多时省时）

        def residual(q6):
            q = np.zeros(model.nq)
            for v, qi in zip(q6, q_idx):
                q[qi] = float(v)
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacement(model, data, fid)
            trans_err = data.oMf[fid].translation - target
            if ref_R is None:
                return trans_err
            rot_err = pin.log3(ref_R.T @ data.oMf[fid].rotation)
            return np.concatenate([trans_err, rot_err])

        res = least_squares(
            residual, x0, bounds=(lo, hi),
            xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=120,
        )
        if float(np.linalg.norm(res.fun)) < 1e-3:
            return [float(v) for v in res.x]
        return None

    def _adaptive_blend_radii(
        self, xyz_seq: List[List[float]], max_r: float
    ) -> List[float]:
        """
        F134: pilz 要求相邻 blend 圆盘不重叠（r_{i-1}+r_i <= 中间点两侧距离）。
        xyz_seq = [起点xyz, 目标1..目标n xyz]；返回每项 blend 半径，末项恒 0；
        段长不足时该项降为 0（允许该处短暂停车），避免「Blending failed」。
        """
        import math

        n = len(xyz_seq) - 1
        radii = [0.0] * n
        for i in range(n - 1):
            d_prev = math.dist(xyz_seq[i], xyz_seq[i + 1])
            d_next = math.dist(xyz_seq[i + 1], xyz_seq[i + 2])
            r = min(max_r, 0.45 * min(d_prev, d_next))
            radii[i] = r if r >= 0.005 else 0.0
        return radii

    def _build_ptp_sequence_item(
        self, target6: List[float], blend_r: float,
        v_scale: Optional[float] = None, a_scale: Optional[float] = None,
    ) -> MotionSequenceItem:
        """F134: pilz PTP 序列项（关节目标 + blend 半径；最后一项 blend=0）."""
        mpr = MotionPlanRequest()
        mpr.group_name = str(self.get_parameter("moveit_planning_group").value)
        mpr.pipeline_id = "pilz"
        mpr.planner_id = "PTP"
        mpr.num_planning_attempts = 1
        mpr.allowed_planning_time = float(
            self.get_parameter("moveit_allowed_planning_time_s").value)
        mpr.max_velocity_scaling_factor = (
            float(self.get_parameter("moveit_velocity_scaling").value)
            if v_scale is None else v_scale)
        mpr.max_acceleration_scaling_factor = (
            float(self.get_parameter("moveit_acceleration_scaling").value)
            if a_scale is None else a_scale)
        tol = float(self.get_parameter("moveit_goal_tolerance_rad").value)
        constraints = Constraints()
        for jn, val in zip(JOINTS[:6], target6):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(val)
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        mpr.goal_constraints = [constraints]
        item = MotionSequenceItem()
        item.req = mpr
        item.blend_radius = float(blend_r)
        return item

    def _build_lin_sequence_item(
        self, pose: dict, blend_r: float,
        velocity_scaling: Optional[float] = None,
    ) -> MotionSequenceItem:
        """F134: pilz LIN 序列项（约束构造与单段 _lin_move 一致）。

        velocity_scaling: 段级速度缩放（0~1）；None 用全局 waypoint_lin_velocity_scaling。
        """
        pos = pose.get("position") or {}
        ori = pose.get("orientation") or {}
        target_pose = Pose()
        target_pose.position.x = float(pos["x"])
        target_pose.position.y = float(pos["y"])
        target_pose.position.z = float(pos["z"])
        target_pose.orientation.x = float(ori["x"])
        target_pose.orientation.y = float(ori["y"])
        target_pose.orientation.z = float(ori["z"])
        target_pose.orientation.w = float(ori["w"])

        pos_tol = float(self.get_parameter("waypoint_lin_pos_tol_m").value)
        rot_tol = float(self.get_parameter("waypoint_lin_rot_tol_rad").value)
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[pos_tol] * 3)
        pc = PositionConstraint()
        pc.header.frame_id = str(self.get_parameter("waypoint_base_link").value)
        pc.link_name = str(self.get_parameter("waypoint_ee_link").value)
        pc.constraint_region.primitives = [box]
        pc.constraint_region.primitive_poses = [target_pose]
        oc = OrientationConstraint()
        oc.header = pc.header
        oc.orientation = target_pose.orientation
        oc.link_name = pc.link_name
        oc.absolute_x_axis_tolerance = rot_tol
        oc.absolute_y_axis_tolerance = rot_tol
        oc.absolute_z_axis_tolerance = rot_tol
        oc.weight = 1.0
        constraints = Constraints()
        constraints.position_constraints = [pc]
        constraints.orientation_constraints = [oc]

        mpr = MotionPlanRequest()
        mpr.group_name = str(
            self.get_parameter("waypoint_lin_planning_group").value)
        mpr.pipeline_id = "pilz"
        mpr.planner_id = "LIN"
        mpr.num_planning_attempts = 1
        mpr.allowed_planning_time = float(
            self.get_parameter("moveit_allowed_planning_time_s").value)
        mpr.max_velocity_scaling_factor = (
            float(self.get_parameter("waypoint_lin_velocity_scaling").value)
            if velocity_scaling is None else velocity_scaling)
        # F165: 段级 speed 须同时缩放加速度——否则短距离轨迹加速度主导、速度缩放
        # 不改变时长（pilz/blend 在 speed>=0.4 后饱和）。与 _moveit_move 同口径。
        a_global = float(
            self.get_parameter("waypoint_lin_acceleration_scaling").value)
        mpr.max_acceleration_scaling_factor = (
            velocity_scaling * a_global if velocity_scaling is not None else a_global)
        mpr.goal_constraints = [constraints]
        item = MotionSequenceItem()
        item.req = mpr
        item.blend_radius = float(blend_r)
        return item

    def _sequence_move_group(
        self,
        items: List[MotionSequenceItem],
        label: str,
        state: str = STATE_TRAJ,
    ) -> Tuple[bool, float, str]:
        """
        F134: pilz Sequence —— N 段一次规划、一次执行（blend 过中间点不停车）.

        返回 (成功, 总时长 s, 说明)。调用方须先完成 F107 门禁；任何不可用/失败
        返回 False，由调用方回退逐段链。
        """
        if not self._sequence_cli.server_is_ready():
            deadline = time.monotonic() + 0.5
            while (time.monotonic() < deadline
                   and not self._sequence_cli.server_is_ready()):
                time.sleep(0.02)
        if not self._sequence_cli.server_is_ready():
            return False, 0.0, "sequence_move_group action unavailable"

        goal = MoveGroupSequence.Goal()
        goal.request.items = list(items)
        goal.planning_options = PlanningOptions()  # plan_only=False → 规划后直接执行

        timeout_s = float(self.get_parameter("sequence_move_timeout_s").value)
        t0 = time.monotonic()
        send_future = self._sequence_cli.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_s):
            return False, 0.0, "sequence send timeout"
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False, 0.0, "sequence goal rejected"
        self._publish_mode("TRAJ_RUNNING")
        self._set_state(state, f"{label} (sequence planning)")

        result_future = goal_handle.get_result_async()
        remaining = max(0.1, timeout_s - (time.monotonic() - t0))
        if not self._wait_future(result_future, remaining):
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label}: sequence result timeout")
            return False, 0.0, "sequence result timeout"
        seq_resp = result_future.result().result.response
        if seq_resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self._publish_mode("IDLE")
            self._set_state(
                STATE_READY, f"{label}: sequence error {seq_resp.error_code.val}")
            return False, 0.0, f"sequence error_code={seq_resp.error_code.val}"

        # 轨迹（pilz 按组返回，通常单组单条）取末点时间最大值为总时长
        total = 0.0
        npts = 0
        for rt in seq_resp.planned_trajectories:
            jt = rt.joint_trajectory
            npts += len(jt.points)
            if jt.points:
                last = jt.points[-1].time_from_start
                total = max(
                    total,
                    float(last.sec) + float(last.nanosec) * 1e-9)
        total = max(total, 0.1)
        self._duty_record(total)
        metric_s = ""
        if seq_resp.planned_trajectories:
            jt = seq_resp.planned_trajectories[0].joint_trajectory
            jtimes = [
                float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9
                for p in jt.points]
            m = _finite_diff_metrics(jtimes, [list(p.positions) for p in jt.points])
            if m is not None:
                metric_s = f" j_rms={m[1]:.4f} a_rms={m[0]:.4f}"
        return True, total, f"{label}: sequence {npts} pts, {total:.1f}s{metric_s}"

    def _moveit_move(
        self,
        target: List[float],
        joint_names: List[str],
        label: str,
        state: str = STATE_TRAJ,
        velocity_scaling: Optional[float] = None,
    ) -> Tuple[bool, float, str]:
        """
        F67: move_group action 规划 + TOTG 定时 + Execute（经 FJT → 执行层）.

        velocity_scaling: 段级速度缩放（0~1）；None 用全局 moveit_velocity_scaling。
        返回 (成功, 执行时长 s, 说明)。任何不可用/失败均返回 False，调用方走本地兜底。
        """
        if not self._moveit_cli.server_is_ready():
            ready_deadline = time.monotonic() + 0.5
            while time.monotonic() < ready_deadline and not self._moveit_cli.server_is_ready():
                time.sleep(0.02)
        if not self._moveit_cli.server_is_ready():
            return False, 0.0, "move_group action unavailable"

        # F107: 当前→目标关节空间密集采样（≥21 点）静态重力矩预检。
        # move_group 实际路径可能偏离采样直线，属工程近似，局限已记入 SAFETY.md。
        if self._have_js:
            samples = 21
            for k in range(samples + 1):
                frac = k / samples
                full = list(self._positions)
                for jn, tv in zip(joint_names, target):
                    if jn in self._joint_names:
                        ci = self._joint_names.index(jn)
                        full[ci] = self._positions[ci] + (float(tv) - self._positions[ci]) * frac
                viol = self._static_torque_violations(full)
                if viol:
                    return False, 0.0, "static torque gate rejected: " + "; ".join(viol)
            # F107: duty 预检（以速度地板时长保守估计；实际时长在结果后入账）
            q0_arm = [
                self._positions[self._joint_names.index(jn)] for jn in joint_names]
            est_dur, _slow = self._velocity_floor_duration(q0_arm, list(target), joint_names)
            allowed, used_s, cooling_s = self._duty_check(max(est_dur, 0.1))
            if not allowed:
                return False, 0.0, (
                    f"duty gate rejected: {used_s:.0f}s motion in window, "
                    f"wait {cooling_s:.0f}s")

        goal = MoveGroup.Goal()
        mpr = MotionPlanRequest()
        mpr.group_name = str(self.get_parameter("moveit_planning_group").value)
        mpr.num_planning_attempts = int(self.get_parameter("moveit_num_planning_attempts").value)
        mpr.allowed_planning_time = float(
            self.get_parameter("moveit_allowed_planning_time_s").value
        )
        vs = (
            velocity_scaling
            if velocity_scaling is not None
            else float(self.get_parameter("moveit_velocity_scaling").value)
        )
        a_global = float(self.get_parameter("moveit_acceleration_scaling").value)
        mpr.max_velocity_scaling_factor = vs
        # 段级 speed 同时缩放加速度：否则短距离轨迹加速度主导、速度缩放不改变时长
        mpr.max_acceleration_scaling_factor = (
            vs * a_global if velocity_scaling is not None else a_global
        )
        tol = float(self.get_parameter("moveit_goal_tolerance_rad").value)
        constraints = Constraints()
        for name, value in zip(joint_names, target):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        mpr.goal_constraints = [constraints]
        goal.request = mpr
        goal.planning_options = PlanningOptions()

        timeout_s = float(self.get_parameter("moveit_goto_timeout_s").value)
        t0 = time.monotonic()
        send_future = self._moveit_cli.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_s):
            return False, 0.0, "move_group send timeout"
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False, 0.0, "move_group goal rejected"
        self._publish_mode("TRAJ_RUNNING")
        self._set_state(state, f"{label} (planning)")

        result_future = goal_handle.get_result_async()
        remaining = max(0.1, timeout_s - (time.monotonic() - t0))
        if not self._wait_future(result_future, remaining):
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label}: result timeout")
            return False, 0.0, "move_group result timeout"
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label}: error {result.error_code.val}")
            return False, 0.0, f"move_group error_code={result.error_code.val}"

        rt = result.executed_trajectory
        if not rt.joint_trajectory.points:
            rt = result.planned_trajectory
        pts = rt.joint_trajectory.points
        if not pts:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label}: empty trajectory")
            return False, 0.0, "move_group returned empty trajectory"
        last = pts[-1]
        duration = float(last.time_from_start.sec) + float(last.time_from_start.nanosec) * 1e-9
        duration = max(duration, 0.1)
        # F107: 实际执行时长入占空比账（FSM 串行化，预检与入账间无并发运动）
        self._duty_record(duration)
        return True, duration, f"{label}: move_group {len(pts)} pts, {duration:.1f}s"

    def _wait_move_group_current(
        self, expected_q: List[float], timeout: float = 8.0, eps: float = 0.03
    ) -> bool:
        """
        LL-103: 段间等 move_group 内部当前状态追上 expected_q（实际关节）。

        move_group current-state monitor 经话题异步更新；上一段刚执行完时它可能
        仍滞后，下一段（尤其 Pilz LIN）IK 用滞后状态做种子 → -31。
        """
        if not self._scene_cli.wait_for_service(timeout_sec=1.0):
            return False
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            time.sleep(0.1)
            fut = self._scene_cli.call_async(req)
            if not self._wait_future(fut, 2.0):
                continue
            snap = dict(zip(
                fut.result().scene.robot_state.joint_state.name,
                fut.result().scene.robot_state.joint_state.position))
            worst = max(
                abs(snap.get(jn, 0.0) - v)
                for jn, v in zip(self._joint_names, expected_q))
            if worst < eps:
                return True
        return False

    def _wait_joint_arrival(
        self, target_q: Dict[str, float], timeout_s: float, eps: float = 0.02
    ) -> bool:
        """F16x: 等 /joint_states 实际关节追到 target_q（容差 eps rad）。

        topic/fjt 直发段（_goto_via_smooth / cp）下发后是异步执行，若立即返回，
        下一段会抢占/覆盖本段轨迹（sim_motor_node _on_traj 整条覆盖）。这里轮询
        self._positions（订阅 /joint_states 更新）直到追到轨迹终点。
        返回是否在 timeout_s 内到位；超时仅 WARN 不阻断（轨迹已下发成功）。
        """
        if not self._have_js:
            return False
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            worst = 0.0
            for jn, tv in target_q.items():
                if jn in self._joint_names:
                    actual = self._positions[self._joint_names.index(jn)]
                    worst = max(worst, abs(actual - float(tv)))
            if worst < eps:
                return True
            time.sleep(0.05)
        self.get_logger().warn(
            f"_wait_joint_arrival timeout after {timeout_s:.1f}s "
            f"(gap > {eps:.3f} rad)")
        return False

    def _lin_move(
        self,
        pose: dict,
        label: str,
        gate_q: Optional[List[float]] = None,
        velocity_scaling: Optional[float] = None,
    ) -> Tuple[bool, float, str]:
        """
        F131: Pilz LIN —— TCP 沿笛卡尔直线平移 + 姿态线性插值到打点 FK 位姿。

        规划失败不回落 PTP（语义不同：操作员明确要直线）。
        gate_q: 该目标路点的 7 关节位姿（打点时已记录），用于 F107 关节线性
        采样预检；真实笛卡尔路径由 Pilz 生成，偏差属工程近似（同 _moveit_move）。
        velocity_scaling: 段级速度缩放（0~1）；None 用全局 waypoint_lin_velocity_scaling。
        """
        if not self._moveit_cli.server_is_ready():
            ready_deadline = time.monotonic() + 0.5
            while time.monotonic() < ready_deadline and not self._moveit_cli.server_is_ready():
                time.sleep(0.02)
        if not self._moveit_cli.server_is_ready():
            return False, 0.0, "move_group action unavailable"

        # F107: 静态重力矩预检（当前→目标关节线性采样；仅当目标关节位姿可得）
        if self._have_js and gate_q is not None:
            samples = 21
            for k in range(samples + 1):
                frac = k / samples
                full = list(self._positions)
                for j in range(min(len(gate_q), len(full))):
                    full[j] = self._positions[j] + (gate_q[j] - self._positions[j]) * frac
                viol = self._static_torque_violations(full)
                if viol:
                    return False, 0.0, "static torque gate rejected: " + "; ".join(viol)
            q0_arm = list(self._positions[:6])
            est_dur, _slow = self._velocity_floor_duration(
                q0_arm, list(gate_q[:6]), JOINTS[:6])
            allowed, used_s, cooling_s = self._duty_check(max(est_dur, 0.1))
            if not allowed:
                return False, 0.0, (
                    f"duty gate rejected: {used_s:.0f}s motion in window, "
                    f"wait {cooling_s:.0f}s")

        pos = pose.get("position") or {}
        ori = pose.get("orientation") or {}
        try:
            target_pose = Pose()
            target_pose.position.x = float(pos["x"])
            target_pose.position.y = float(pos["y"])
            target_pose.position.z = float(pos["z"])
            target_pose.orientation.x = float(ori["x"])
            target_pose.orientation.y = float(ori["y"])
            target_pose.orientation.z = float(ori["z"])
            target_pose.orientation.w = float(ori["w"])
        except (KeyError, TypeError) as exc:
            return False, 0.0, f"invalid pose dict: {exc}"

        pos_tol = float(self.get_parameter("waypoint_lin_pos_tol_m").value)
        rot_tol = float(self.get_parameter("waypoint_lin_rot_tol_rad").value)
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[pos_tol] * 3)
        pc = PositionConstraint()
        pc.header.frame_id = str(self.get_parameter("waypoint_base_link").value)
        pc.link_name = str(self.get_parameter("waypoint_ee_link").value)
        pc.constraint_region.primitives = [box]
        pc.constraint_region.primitive_poses = [target_pose]
        oc = OrientationConstraint()
        oc.header = pc.header
        oc.orientation = target_pose.orientation
        oc.link_name = pc.link_name
        oc.absolute_x_axis_tolerance = rot_tol
        oc.absolute_y_axis_tolerance = rot_tol
        oc.absolute_z_axis_tolerance = rot_tol
        oc.weight = 1.0
        constraints = Constraints()
        constraints.position_constraints = [pc]
        constraints.orientation_constraints = [oc]

        goal = MoveGroup.Goal()
        mpr = MotionPlanRequest()
        # F131: arm_lin 组（pick_ik global；arm 组 local 种子对远目标 IK -31）
        mpr.group_name = str(
            self.get_parameter("waypoint_lin_planning_group").value)
        mpr.pipeline_id = "pilz"
        mpr.planner_id = "LIN"
        mpr.num_planning_attempts = int(self.get_parameter("moveit_num_planning_attempts").value)
        mpr.allowed_planning_time = float(
            self.get_parameter("moveit_allowed_planning_time_s").value
        )
        vs = (
            velocity_scaling
            if velocity_scaling is not None
            else float(self.get_parameter("waypoint_lin_velocity_scaling").value)
        )
        a_global = float(self.get_parameter("waypoint_lin_acceleration_scaling").value)
        mpr.max_velocity_scaling_factor = vs
        # 段级 speed 同时缩放加速度（同 _moveit_move）
        mpr.max_acceleration_scaling_factor = (
            vs * a_global if velocity_scaling is not None else a_global
        )
        mpr.goal_constraints = [constraints]
        goal.request = mpr
        goal.planning_options = PlanningOptions()

        timeout_s = float(self.get_parameter("moveit_goto_timeout_s").value)
        seg_timeout = float(self.get_parameter("waypoint_segment_timeout_s").value)
        if seg_timeout > 0.0:
            timeout_s = seg_timeout
        t0 = time.monotonic()
        send_future = self._moveit_cli.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_s):
            return False, 0.0, "move_group send timeout"
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False, 0.0, "move_group goal rejected"
        self._publish_mode("TRAJ_RUNNING")
        self._set_state(STATE_TRAJ, f"{label} (planning LIN)")

        result_future = goal_handle.get_result_async()
        remaining = max(0.1, timeout_s - (time.monotonic() - t0))
        if not self._wait_future(result_future, remaining):
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label}: result timeout")
            return False, 0.0, "move_group result timeout"
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label}: error {result.error_code.val}")
            return False, 0.0, f"pilz LIN error_code={result.error_code.val} (no PTP fallback)"

        rt = result.executed_trajectory
        if not rt.joint_trajectory.points:
            rt = result.planned_trajectory
        pts = rt.joint_trajectory.points
        if not pts:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label}: empty trajectory")
            return False, 0.0, "pilz LIN returned empty trajectory"
        last = pts[-1]
        duration = float(last.time_from_start.sec) + float(last.time_from_start.nanosec) * 1e-9
        duration = max(duration, 0.1)
        self._duty_record(duration)
        return True, duration, f"{label}: LIN {len(pts)} pts, {duration:.1f}s"

    def _play_waypoints(
        self,
        data: dict,
        label: str,
        strategy: str,
        resp: PlaybackTrajectory.Response,
    ) -> PlaybackTrajectory.Response:
        """
        F131: 路点任务逐段回放（每段到位即停，blend=0；阶段二再上 Pilz blend）。

        PTP: 逐段关节空间规划（_moveit_move，OMPL/TOTG）；
        LIN: 逐段 Pilz 笛卡尔直线（_lin_move，失败即停，不回落）。
        每段：F107 静态重力矩 + duty 门禁；L7（夹爪）随段末同步。
        """
        file_joint_names = list(data.get("joint_names") or self._joint_names)

        def _to_std(positions: List[float]) -> List[float]:
            if file_joint_names == self._joint_names:
                return [float(v) for v in positions]
            out = []
            for jn in self._joint_names:
                if jn in file_joint_names:
                    out.append(float(positions[file_joint_names.index(jn)]))
                else:
                    out.append(0.0)
            return out

        parsed: List[Dict[str, Any]] = []
        for w in data.get("waypoints") or []:
            if "positions" not in w:
                self._publish_mode("IDLE")
                self._set_state(STATE_READY, f"playback {label}: bad waypoint")
                resp.success = False
                resp.message = "waypoint entry missing positions"
                return resp
            parsed.append({"positions": _to_std(w["positions"]), "pose": w.get("pose") or {}})

        n = len(parsed)
        min_count = int(self.get_parameter("waypoint_min_count").value)
        if n < min_count:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: too few")
            resp.success = False
            resp.message = f"need >= {min_count} waypoints, file has {n}"
            return resp

        # F134: 优先整序列一次规划/执行（pilz Sequence，blend 过中间点不停车）。
        # 不可用/被门禁拒绝/规划失败 → WARN 并回落下方逐段链（F131 行为兜底）。
        if bool(self.get_parameter("waypoint_use_sequence").value):
            blend = float(self.get_parameter("waypoint_blend_radius_m").value)
            lin_poses_ok = all(
                wp["pose"] and "position" in wp["pose"] for wp in parsed)
            if strategy == "lin" and not lin_poses_ok:
                self.get_logger().warn(
                    f"playback {label}: 存在缺 FK pose 路点，无法 LIN 序列 "
                    "-- per-segment fallback")
            else:
                targets = [wp["positions"][:6] for wp in parsed]
                reject = self._sequence_gate_check(targets, JOINTS[:6])
                if reject:
                    self.get_logger().warn(
                        f"playback {label}: {reject} -- per-segment fallback")
                else:
                    xyz_seq = [
                        self._fk_ee_xyz(list(self._positions))]
                    xyz_seq += [
                        self._fk_ee_xyz(wp["positions"]) for wp in parsed]
                    if any(x is None for x in xyz_seq):
                        radii = [
                            blend if i < n - 1 else 0.0 for i in range(n)]
                    else:
                        radii = self._adaptive_blend_radii(xyz_seq, blend)
                    items: List[MotionSequenceItem] = []
                    for i, wp in enumerate(parsed):
                        br = radii[i]
                        if strategy == "lin":
                            items.append(
                                self._build_lin_sequence_item(wp["pose"], br))
                        else:
                            items.append(
                                self._build_ptp_sequence_item(
                                    wp["positions"][:6], br))
                    ok, stotal, smsg = self._sequence_move_group(
                        items, f"waypoint {strategy} {label}")
                    if ok:
                        if self._n_joints >= 7:
                            stotal = max(
                                stotal,
                                self._dispatch_l7_linear(
                                    parsed[-1]["positions"][6]))
                        self._schedule_back_to_ready(0.3)  # 同步已执行完，仅短缓冲
                        resp.success = True
                        resp.message = (
                            f"waypoint task '{label}' {strategy} sequence: "
                            f"{n} pts, {stotal:.1f}s, "
                            f"blend r<={max(radii):.2f}m")
                        return resp
                    self.get_logger().warn(
                        f"playback {label}: {smsg} -- per-segment fallback")

        total = 0.0
        # F131: 单段失败重试——段间同步滞后（LL-103）引发的瞬态规划失败（IK -31 等）
        # 重等 move_group current 后重试通常成功；门禁拒绝（静态力矩/占空比）幂等无副作用。
        seg_retries = max(0, int(self.get_parameter("waypoint_segment_retries").value))
        for i, wp in enumerate(parsed):
            seg_label = f"waypoint {strategy} {i + 1}/{n}"
            pose = wp["pose"]
            if strategy == "lin" and (not pose or "position" not in pose):
                self._publish_mode("IDLE")
                self._set_state(STATE_READY, f"playback {label}: no FK pose")
                resp.success = False
                resp.message = (
                    f"waypoint {i + 1}/{n} missing FK pose —— 该任务录制时"
                    "FK 不可用，无法 LIN；请重新 Share 长按录制或改 PTP")
                return resp
            ok = False
            dur = 0.0
            msg = ""
            for attempt in range(seg_retries + 1):
                if attempt:
                    # 重试前补一次段间同步（current-state monitor 追上实际关节）
                    self._wait_move_group_current(wp["positions"])
                if strategy == "lin":
                    ok, dur, msg = self._lin_move(
                        pose, seg_label, gate_q=wp["positions"])
                else:
                    ok, dur, msg = self._moveit_move(
                        wp["positions"][:6], JOINTS[:6], seg_label)
                if ok:
                    break
                self.get_logger().warn(
                    f"{seg_label} attempt {attempt + 1} failed: {msg}")
            if not ok:
                self._publish_mode("IDLE")
                self._set_state(STATE_READY, f"playback {label}: seg {i + 1} failed")
                resp.success = False
                resp.message = (
                    f"segment {i + 1}/{n} failed after {seg_retries + 1} attempt(s) "
                    f"(arm stopped): {msg}")
                return resp
            # LL-077: arm 规划组只含 L1–L6，L7 随段末同步下发
            if self._n_joints >= 7:
                dur = max(dur, self._dispatch_l7_linear(wp["positions"][6]))
            total += dur
            # LL-103: 段间等 move_group 内部状态追上实际关节（防下段 LIN IK -31）；
            # 同步超时不阻断，交给下段规划结果裁决。
            if i < n - 1 and not self._wait_move_group_current(wp["positions"]):
                self.get_logger().warn(
                    f"after segment {i + 1}/{n}: move_group current sync timeout "
                    "(continue; next segment may fail IK)")

        self._schedule_back_to_ready(0.3)  # 同步已执行完，仅短缓冲
        resp.success = True
        resp.message = (
            f"waypoint task '{label}' {strategy}: {n} points, "
            f"{n - 1} segments, {total:.1f}s")
        return resp

    def _goto_via_smooth(
        self,
        via: List[Dict[str, Any]],
        target: Dict[str, Any],
        speed: Optional[float],
        label: str,
        profile: str = "l1p5",
        exact: Optional[List[bool]] = None,
        hold_orientation: bool = False,
    ) -> Tuple[bool, str, float]:
        """F163/F16x: 当前位 → via 过点数组 → target 规划为一条平滑轨迹（gotoby 原语）。

        via/target 可为 {"positions":[...]}（关节）或 {"pose":{position,orientation}}
        （笛卡尔，走 _solve_ik_position）。speed(0~1) 线性缩放峰值速度/加速度上限；
        None 时用 1.0（即全局 goto_via_* 上限）。中间点不停车、段末速度 0。
        profile 选时间剖面：l1p5=线性折线+G64 圆角+quintic 加-巡-减（默认）、l2=...+梯形、
        l3=cubic 插值样条+钟形、l3_fit=最小二乘逼近样条+quintic（F166）。
        F167: `goto_via_time_backend`=software（默认）/totg/ruckig 决定时间律来源——非
        software 时调 /a3/arm/retime_trajectory 换时间律（几何路径不变），失败回退手写剖面。
        exact（F166，长度=len(via)）：逐 via 点是否**必须精确过点**（True→该顶点不做 G64
        圆角）；None 表示全不精确（统一 corner_r）。端点（当前位/target）恒精确。
        返回 (成功, 说明, 运动时长秒)。失败返回 (False, msg, 0.0)，调用方回落单段 _moveit_move。
        """
        GJ = JOINTS[:6]

        def resolve(point: Dict[str, Any], ref_R=None,
                    seed: Optional[List[float]] = None) -> Optional[List[float]]:
            positions = point.get("positions")
            if positions:
                q = [float(v) for v in positions]
                if len(q) < 6:
                    q = q + [0.0] * (6 - len(q))
                return q[:6]
            pose = point.get("pose")
            if pose and "position" in pose:
                return self._solve_ik_position(pose, ref_orientation=ref_R, seed=seed)
            return None

        q_cur = [self._positions[self._joint_names.index(jn)] for jn in GJ]
        # 先解 target（position-only），其末端姿态作为 via 点的姿态参考，
        # 避免 via 点姿态自由导致腕部 L4/L5 大幅摆动、关节路径绕远路（提速关键）。
        # F176 hold_orientation=True：target/via 各自用**自身 pose.orientation（示教姿态）**
        # 做 6D IK，并以链上前一点的解作种子；不可达时回落参考姿态 / position-only。
        tgt_R = self._pose_ref_rotation(target.get("pose")) if hold_orientation else None
        qt = resolve(target, tgt_R)
        if qt is None and tgt_R is not None:
            qt = resolve(target)
        if qt is None:
            return False, "via target IK/resolve failed", 0.0
        ref_R = tgt_R if tgt_R is not None else self._fk_ee_orientation(qt)
        via_q: List[List[float]] = [q_cur]
        seed: Optional[List[float]] = None
        for p in via:
            p_R = ref_R
            if hold_orientation:
                own_R = self._pose_ref_rotation(p.get("pose"))
                if own_R is not None:
                    p_R = own_R
            q = resolve(p, p_R, seed)
            if q is None and p_R is not None:
                q = resolve(p, None, seed)  # 姿态保持失败时回落 position-only（保底）
            if q is None:
                return False, "via point IK/resolve failed", 0.0
            via_q.append(q)
            if hold_orientation:
                seed = q
        via_q.append(qt)

        resample_n = max(20, int(self.get_parameter("goto_via_resample_n").value))
        spd = 1.0 if speed is None else float(speed)
        vmax = spd * float(self.get_parameter("goto_via_max_vel_rad_s").value)
        amax = spd * float(self.get_parameter("goto_via_max_acc_rad_s2").value)
        # F165：l1p5/l2 的直线折线在 via 拐角是几何尖角（dq/ds 不连续），会让时间
        # 参数化的缩放循环把整条轨迹大幅降速。按工业 G64 连续路径，仅对拐角做二次
        # Bezier 圆角（直线段保形不变）。goto_via_corner_blend_rad<=0 关闭圆角。
        corner_r = float(self.get_parameter("goto_via_corner_blend_rad").value)
        # F166：逐点 exact → 逐顶点半径（0=精确过点）。via_q = [q_cur, *via, target]，
        # 端点（下标 0 与末位）恒精确；内部顶点按 exact 标置 0 或 corner_r。
        if exact is not None:
            n_via = len(via)
            radii_v = ([0.0]
                       + [0.0 if (i < len(exact) and bool(exact[i])) else corner_r
                          for i in range(n_via)]
                       + [0.0])
        else:
            radii_v = corner_r

        # 几何路径（手写，保留）：按 profile 选路径生成器；时间律见下（F167 可切成熟库）。
        if profile == "l3":
            dense = _interpolate_via_spline(via_q, resample_n)
        elif profile == "l3_fit":
            dense = _approx_spline(via_q, resample_n)
            if dense is None:
                dense = _interpolate_via_spline(via_q, resample_n)  # 兜底：退回插值样条
        else:  # l1p5/l2：G64 圆角折线
            dense = _round_corners(via_q, radii_v)
            if dense is None:
                dense = _linear_dense(via_q, resample_n)  # 兜底：圆角不可用则回退直线折线
        if dense is None:
            return False, "via path failed", 0.0

        def _software_timing() -> Optional[Tuple[List[float], List[List[float]]]]:
            if profile == "l3":
                return _bell_timed_traj(dense, vmax, amax)
            if profile == "l2":
                return _trapezoidal_timed_traj(dense, vmax, amax)
            # l1p5 / l3_fit：quintic 加-巡-减
            return _jerk_limited_timed_traj(dense, vmax, amax)

        # F167: 时间律——software=手写剖面；totg/ruckig=调 /a3/arm/retime_trajectory
        # 用成熟库换时间律（几何路径不变），服务不可用/失败自动回退手写剖面。
        timed = None
        time_backend = str(self.get_parameter("goto_via_time_backend").value).strip().lower()
        if time_backend in ("totg", "ruckig"):
            ok_r, rtraj, rmsg = self._call_retime(
                dense, list(GJ), time_backend, 1.0, 1.0,
                velocity_limits=[vmax] * len(GJ),
                acceleration_limits=[amax] * len(GJ),
                timeout_s=float(self.get_parameter("goto_via_retime_timeout_s").value),
            )
            if ok_r and rtraj is not None and len(rtraj.points) >= 2:
                times = [_tfs_seconds(p.time_from_start) for p in rtraj.points]
                tpos = [list(p.positions) for p in rtraj.points]
                timed = (times, tpos)
            else:
                self.get_logger().warn(
                    f"goto_via retime({time_backend}) failed ({rmsg}); "
                    f"fallback to software profile '{profile}'")
        if timed is None:
            timed = _software_timing()
        if timed is None:
            return False, "via timing failed", 0.0
        times, tpos = timed

        traj = JointTrajectory()
        traj.joint_names = list(GJ)
        for t, pos in zip(times, tpos):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in pos]
            pt.time_from_start = _duration(t)
            traj.points.append(pt)

        # JTC 下发（L1–L6；L7 不参与保持 0），静态力矩/占空比门禁在内部生效。
        self._publish_mode("TRAJ_RUNNING")
        ok, dmsg = self._dispatch_trajectory(traj)
        if not ok:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label} via: {dmsg}")
            return False, dmsg, 0.0
        self._set_state(STATE_TRAJ, f"{label} via")
        duration = self._traj_duration(traj)
        # F16x 修复：topic/fjt 直发是异步的，须同步等到 /joint_states 追到轨迹
        # 终点再返回，否则 _play_stream 下一段会抢占/覆盖本段（l1p5/l2/l3 被回程
        # 段顶掉，实际只跑了 ~0.2s）。对齐 _goto_via_lin_blend 的同步语义。
        target_q = dict(zip(list(traj.joint_names), list(traj.points[-1].positions)))
        self._wait_joint_arrival(target_q, timeout_s=duration + 5.0)
        self._schedule_back_to_ready(0.3)  # 同步已执行完，仅短缓冲

        metric_s = ""
        m = _finite_diff_metrics(times, tpos)
        if m is not None:
            metric_s = f" j_rms={m[1]:.4f} a_rms={m[0]:.4f}"
        return True, (
            f"{label}: via {len(via_q) - 1} pts, {len(traj.points)} samples, "
            f"{duration:.2f}s{metric_s}"), duration

    def _goto_via_lin_blend(
        self,
        via: List[Dict[str, Any]],
        target: Dict[str, Any],
        speed: Optional[float],
        label: str,
        blend_radius: Optional[float] = None,
        exact: Optional[List[bool]] = None,
    ) -> Tuple[bool, str, float]:
        """F163/F16x: 当前位 → via 过点 → target 走 pilz LIN Sequence（blend 过中间点不停车）。

        所有 via/target 须为笛卡尔 {"pose":{position,orientation}}（lin 语义）；
        blend 不精确过中间点（F134 语义）。blend_radius=None 用 waypoint_blend_radius_m；
        传 0.0 则逐点精确过点、段末停（笛卡尔 pilz 模式）。exact（F166，长度=len(via)）：
        逐点是否必须精确过点（True→该项 blend=0 精确到位）。返回 (成功, 说明, 运动时长秒)；
        失败返回 (False, msg, 0.0)，调用方回落 _goto_via_smooth。
        """
        def pose_of(point: Dict[str, Any]) -> Optional[dict]:
            pose = point.get("pose")
            if pose and "position" in pose and "orientation" in pose:
                return pose
            return None

        poses: List[dict] = []
        for p in via:
            pose = pose_of(p)
            if pose is None:
                return False, "lin via point missing pose", 0.0
            poses.append(pose)
        tpose = pose_of(target)
        if tpose is None:
            return False, "lin via target missing pose", 0.0
        poses.append(tpose)

        if blend_radius is not None:
            radii = [blend_radius if i < len(poses) - 1 else 0.0 for i in range(len(poses))]
        else:
            xyz_seq = [self._fk_ee_xyz(list(self._positions))]
            for pose in poses:
                pos = pose["position"]
                xyz_seq.append([float(pos["x"]), float(pos["y"]), float(pos["z"])])
            blend = float(self.get_parameter("waypoint_blend_radius_m").value)
            if any(x is None for x in xyz_seq):
                radii = [blend if i < len(poses) - 1 else 0.0 for i in range(len(poses))]
            else:
                radii = self._adaptive_blend_radii(xyz_seq, blend)
        # F166: 逐点 exact → 该项 blend=0（精确过点）
        if exact is not None:
            for i in range(len(radii)):
                if i < len(exact) and bool(exact[i]):
                    radii[i] = 0.0

        items = [
            self._build_lin_sequence_item(pose, radii[i], speed)
            for i, pose in enumerate(poses)]
        ok, stotal, smsg = self._sequence_move_group(
            items, f"via lin {label}")
        if not ok:
            return False, smsg, 0.0
        self._schedule_back_to_ready(0.3)  # 同步已执行完，仅短缓冲
        return True, f"{label}: via lin {len(poses)} pts, {stotal:.1f}s", stotal

    def _goto_via_ptp_blend(
        self,
        via: List[Dict[str, Any]],
        target: Dict[str, Any],
        speed: Optional[float],
        label: str,
        blend_radius: Optional[float] = None,
        exact: Optional[List[bool]] = None,
        hold_orientation: bool = False,
    ) -> Tuple[bool, str, float]:
        """F166: 当前位 → via 过点 → target 走 pilz PTP Sequence（关节空间，blend 不停车）。

        via/target 解析成关节（positions 直取；pose 走 _solve_ik_position）。逐项 pilz PTP +
        自适应 blend 半径（F134 语义）。exact（长度=len(via)）为 True 的点 blend=0 → 精确过点。
        F176 hold_orientation=True：每个点用**其自身 pose.orientation（示教姿态）**做 6D IK，
        并以链上前一点的解作种子（腕部连续）；该点姿态不可达时回落 position-only（WARN）。
        返回 (成功, 说明, 运动时长秒)；失败返回 (False, msg, 0.0)。
        """
        GJ = JOINTS[:6]

        def resolve(point: Dict[str, Any], ref_R=None,
                    seed: Optional[List[float]] = None) -> Optional[List[float]]:
            positions = point.get("positions")
            if positions:
                q = [float(v) for v in positions]
                if len(q) < 6:
                    q = q + [0.0] * (6 - len(q))
                return q[:6]
            pose = point.get("pose")
            if pose and "position" in pose:
                return self._solve_ik_position(pose, ref_orientation=ref_R, seed=seed)
            return None

        def resolve_hold(point: Dict[str, Any],
                         seed: Optional[List[float]] = None) -> Optional[List[float]]:
            if not hold_orientation:
                return resolve(point)
            ref_R = self._pose_ref_rotation(point.get("pose"))
            if ref_R is not None:
                q = resolve(point, ref_R, seed)
                if q is not None:
                    return q
                self.get_logger().warn(
                    f"{label}: 6D IK 未收敛（保持示教姿态不可达），该点回落 position-only")
            return resolve(point)

        chain: List[List[float]] = [
            [self._positions[self._joint_names.index(jn)] for jn in GJ]]
        seed: Optional[List[float]] = None
        for p in via:
            q = resolve_hold(p, seed)
            if q is None:
                return False, "ptp_blend via IK/resolve failed", 0.0
            chain.append(q)
            seed = q
        qt = resolve_hold(target, seed)
        if qt is None:
            return False, "ptp_blend target IK/resolve failed", 0.0
        chain.append(qt)

        if blend_radius is not None:
            radii = [blend_radius] * (len(chain) - 1)
        else:
            xyz_seq = [self._fk_ee_xyz(list(q)) for q in chain]
            blend = float(self.get_parameter("waypoint_blend_radius_m").value)
            if any(x is None for x in xyz_seq):
                radii = [blend] * (len(chain) - 1)
            else:
                radii = self._adaptive_blend_radii(xyz_seq, blend)
        if exact is not None:
            for i in range(len(radii)):
                if i < len(exact) and bool(exact[i]):
                    radii[i] = 0.0
        radii[-1] = 0.0  # 末项恒精确停

        v_s = 1.0 if speed is None else float(speed)
        a_global = float(self.get_parameter("moveit_acceleration_scaling").value)
        items = [
            self._build_ptp_sequence_item(
                chain[i + 1][:6], radii[i], v_scale=v_s, a_scale=v_s * a_global)
            for i in range(len(chain) - 1)]
        ok, stotal, smsg = self._sequence_move_group(items, f"via ptp {label}")
        if not ok:
            return False, smsg, 0.0
        self._schedule_back_to_ready(0.3)
        return True, f"{label}: via ptp {len(chain) - 1} legs, {stotal:.1f}s", stotal

    def _circ_move(
        self,
        aux: Dict[str, Any],
        target: Dict[str, Any],
        speed: Optional[float],
        label: str,
    ) -> Tuple[bool, float, str]:
        """F166: CIRC 圆弧 —— 起点=当前 TCP、辅助点=aux、终点=target，pilz CIRC 规划。

        pilz CIRC 约定：goal_constraints 需含**两个**笛卡尔约束，[0]=辅助(中间)点、[1]=目标点。
        失败不回落（圆弧语义，与 LIN 一致）。返回 (成功, 时长s, 说明)。
        """
        if not self._moveit_cli.server_is_ready():
            ready_deadline = time.monotonic() + 0.5
            while time.monotonic() < ready_deadline and not self._moveit_cli.server_is_ready():
                time.sleep(0.02)
        if not self._moveit_cli.server_is_ready():
            return False, 0.0, "move_group action unavailable"

        def pose_of(p: Dict[str, Any]) -> Optional[dict]:
            pose = p.get("pose")
            if pose and "position" in pose and "orientation" in pose:
                return pose
            return None

        p_aux, p_goal = pose_of(aux), pose_of(target)
        if p_aux is None or p_goal is None:
            return False, 0.0, "circ needs pose (position+orientation) for aux & target"

        pos_tol = float(self.get_parameter("waypoint_lin_pos_tol_m").value)
        rot_tol = float(self.get_parameter("waypoint_lin_rot_tol_rad").value)

        def constraint_of(pose: dict, box_m: float) -> Constraints:
            pos = pose.get("position") or {}
            ori = pose.get("orientation") or {}
            tp = Pose()
            tp.position.x = float(pos["x"])
            tp.position.y = float(pos["y"])
            tp.position.z = float(pos["z"])
            tp.orientation.x = float(ori["x"])
            tp.orientation.y = float(ori["y"])
            tp.orientation.z = float(ori["z"])
            tp.orientation.w = float(ori["w"])
            box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[box_m] * 3)
            pc = PositionConstraint()
            pc.header.frame_id = str(self.get_parameter("waypoint_base_link").value)
            pc.link_name = str(self.get_parameter("waypoint_ee_link").value)
            pc.constraint_region.primitives = [box]
            pc.constraint_region.primitive_poses = [tp]
            oc = OrientationConstraint()
            oc.header = pc.header
            oc.orientation = tp.orientation
            oc.link_name = pc.link_name
            oc.absolute_x_axis_tolerance = rot_tol
            oc.absolute_y_axis_tolerance = rot_tol
            oc.absolute_z_axis_tolerance = rot_tol
            oc.weight = 1.0
            c = Constraints()
            c.position_constraints = [pc]
            c.orientation_constraints = [oc]
            return c

        goal = MoveGroup.Goal()
        mpr = MotionPlanRequest()
        mpr.group_name = str(self.get_parameter("waypoint_lin_planning_group").value)
        mpr.pipeline_id = "pilz"
        mpr.planner_id = "CIRC"
        mpr.num_planning_attempts = int(self.get_parameter("moveit_num_planning_attempts").value)
        mpr.allowed_planning_time = float(
            self.get_parameter("moveit_allowed_planning_time_s").value)
        vs = (float(speed) if speed is not None
              else float(self.get_parameter("waypoint_lin_velocity_scaling").value))
        a_global = float(self.get_parameter("waypoint_lin_acceleration_scaling").value)
        mpr.max_velocity_scaling_factor = vs
        mpr.max_acceleration_scaling_factor = vs * a_global if speed is not None else a_global
        # pilz CIRC：goal_constraints = 目标点（恰好 1 个）；辅助(中间)点放 path_constraints，
        #   且该约束**名必须是 "interim"**（或 "center"）——否则 CIRC 报
        #   "No path constraint named interim/center found"（INVALID_MOTION_PLAN）。
        #   辅助点：只给位置，且用**大包围盒**——path_constraints 会被 MoveGroup 沿整条弧
        #   校验位置/姿态，小盒或带姿态约束会因弧上偏离而报 violated；pilz CIRC 只取该约束
        #   的位姿点定义圆，盒大小不影响其几何。
        mpr.goal_constraints = [constraint_of(p_goal, pos_tol)]
        aux_c = constraint_of(p_aux, 1.0)
        aux_c.orientation_constraints = []
        aux_c.position_constraints[0].constraint_region.primitives[0].dimensions = [1.0] * 3
        aux_c.name = "interim"
        mpr.path_constraints = aux_c
        goal.request = mpr
        goal.planning_options = PlanningOptions()

        timeout_s = float(self.get_parameter("moveit_goto_timeout_s").value)
        seg_timeout = float(self.get_parameter("waypoint_segment_timeout_s").value)
        if seg_timeout > 0.0:
            timeout_s = seg_timeout
        t0 = time.monotonic()
        send_future = self._moveit_cli.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_s):
            return False, 0.0, "move_group send timeout"
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False, 0.0, "move_group goal rejected (CIRC)"
        self._publish_mode("TRAJ_RUNNING")
        self._set_state(STATE_TRAJ, f"{label} (planning CIRC)")

        result_future = goal_handle.get_result_async()
        remaining = max(0.1, timeout_s - (time.monotonic() - t0))
        if not self._wait_future(result_future, remaining):
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label}: result timeout")
            return False, 0.0, "move_group result timeout"
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label}: error {result.error_code.val}")
            return False, 0.0, f"pilz CIRC error_code={result.error_code.val} (no fallback)"
        rt = result.executed_trajectory
        if not rt.joint_trajectory.points:
            rt = result.planned_trajectory
        pts = rt.joint_trajectory.points
        if not pts:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label}: empty trajectory")
            return False, 0.0, "pilz CIRC returned empty trajectory"
        last = pts[-1]
        duration = float(last.time_from_start.sec) + float(last.time_from_start.nanosec) * 1e-9
        duration = max(duration, 0.1)
        self._duty_record(duration)
        return True, duration, f"{label}: CIRC {len(pts)} pts, {duration:.1f}s"

    def _plan_single(self, spec, target: Dict[str, Any], speed, label: str,
                     hold_orientation: bool = False):
        """单腿（无 via）：按 spec.space 走笛卡尔 _lin_move 或关节 _moveit_move。"""
        if spec.space == "cartesian":
            pose = target.get("pose")
            if not pose or "position" not in pose:
                return False, f"{label}: cartesian segment missing pose", 0.0
            ok, dur, msg = self._lin_move(
                pose, label, gate_q=None, velocity_scaling=speed)
            return ok, msg, dur
        positions = target.get("positions")
        if positions:
            q = [float(v) for v in positions]
        else:
            pose = target.get("pose")
            if not pose or "position" not in pose:
                return False, f"{label}: target missing positions/pose", 0.0
            # F176 hold_orientation：用目标点自身示教姿态做 6D IK（不可达回落 position-only）
            ref_R = self._pose_ref_rotation(pose) if hold_orientation else None
            q6 = self._solve_ik_position(pose, ref_orientation=ref_R)
            if q6 is None and ref_R is not None:
                q6 = self._solve_ik_position(pose)
            if q6 is None:
                return False, f"{label}: pose IK failed", 0.0
            q = q6 + [0.0]
        if len(q) < self._n_joints:
            q = q + [0.0] * (self._n_joints - len(q))
        ok, dur, msg = self._moveit_move(
            q[:6], JOINTS[:6], label, velocity_scaling=speed)
        if ok and self._n_joints >= 7:
            self._dispatch_l7_linear(q[6])
        return ok, msg, dur

    def _plan_mode(
        self, spec: "TrajModeSpec", via: List[Dict[str, Any]],
        target: Dict[str, Any], speed, label: str, exact: List[bool],
        hold_orientation: bool = False,
    ) -> Tuple[bool, str, float]:
        """F166 统一分派：按 spec.kind 执行「当前位 → via → target」段，返回 (ok, msg, dur)。

        新增/切换模式只需在 TRAJ_MODES 加一条注册（+ 对应 kind 实现），调用方无需改动。
        F176 hold_orientation：关节空间段（ptp_legs / soft_smooth / 无 via 单腿）用各点自身示教
        姿态做 6D IK；笛卡尔段（lin_*/circ）本就按 6D 位姿走，无需该开关。
        """
        k = spec.kind
        if k == "ptp_legs" and via:
            # 历史兼容：`mode: ptp` + via 旧行为=关节 G64 过点（等价 jnt_g64_quintic）；
            # 真「逐腿停」用多个无 via 的 ptp 段表达（每段 _plan_single）。
            return self._goto_via_smooth(
                via, target, speed, label, profile="l1p5", exact=exact,
                hold_orientation=hold_orientation)
        if not via:
            return self._plan_single(spec, target, speed, label,
                                     hold_orientation=hold_orientation)
        if k == "pilz_ptp_seq":
            return self._goto_via_ptp_blend(via, target, speed, label, exact=exact,
                                            hold_orientation=hold_orientation)
        if k == "pilz_lin_seq":
            return self._goto_via_lin_blend(via, target, speed, label, exact=exact)
        if k == "lin_single":
            return self._goto_via_lin_blend(
                via, target, speed, label, blend_radius=0.0, exact=exact)
        if k == "circ_single":
            if len(via) != 1:
                return False, f"{label}: circ needs exactly 1 via (auxiliary point)", 0.0
            ok, dur, msg = self._circ_move(via[0], target, speed, label)
            return ok, msg, dur
        if k == "soft_smooth":
            return self._goto_via_smooth(
                via, target, speed, label, profile=spec.profile, exact=exact,
                hold_orientation=hold_orientation)
        return False, f"unhandled mode kind '{k}'", 0.0

    def _auto_fallback_spec(self, via: List[Dict[str, Any]]) -> "TrajModeSpec":
        """F166: `mode: auto` 的运行时保守兜底（正式应由离线 traj_plan_auto.py 解析）。

        全 via 点 exact → jnt_spline_bell（精确过点）；否则 jnt_g64_trapezoid（per_point 切角）。
        """
        if via and all(bool(p.get("exact", False)) for p in via):
            return TRAJ_MODES["jnt_spline_bell"]
        return TRAJ_MODES["jnt_g64_trapezoid"]

    def _execute_segment(self, seg: Dict[str, Any], label: str) -> Tuple[bool, str, float]:
        """执行一个「轨迹段」（F166 统一模式分派）。

        段模型：{target, mode, speed, sleep, via:[{positions|pose, exact}], hold_orientation}。
        mode 为规范名/别名（`TRAJ_MODES`）；'auto' 运行时保守兜底（正式由离线
        `scripts/traj_plan_auto.py` 解析为具体模式）。exact=该点是否必须精确过点。
        返回 (成功, 说明, 运行时长秒 = 运动时长 + sleep)。
        """
        target = seg.get("target") or {}
        mode = str(seg.get("mode") or "ptp").strip().lower()
        speed = seg.get("speed")
        if speed is not None:
            speed = float(speed)
        sleep_s = float(seg.get("sleep") or 0.0)
        via = list(seg.get("via") or [])

        spec = resolve_traj_mode(mode)
        if spec is None:
            if mode == "auto":
                spec = self._auto_fallback_spec(via)
                self.get_logger().warn(
                    f"mode=auto 运行时兜底为 '{spec.canonical}' —— "
                    "建议先跑 scripts/traj_plan_auto.py 离线解析")
            else:
                return False, (
                    f"unknown mode '{mode}' (have: {sorted(TRAJ_MODES)})"), 0.0
        exact = [bool(p.get("exact", False)) for p in via]
        # F176: 段级可选开关——true 时各点用自身示教姿态做 6D IK（防"只求末端到位"横压）
        hold_orientation = bool(seg.get("hold_orientation", False))

        ok, msg, vdur = self._plan_mode(spec, via, target, speed, label, exact,
                                        hold_orientation=hold_orientation)
        if not ok:
            return False, msg, 0.0
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        return True, msg, vdur + sleep_s

    def _expand_relative(self, data: dict) -> Tuple[bool, str]:
        """F16x: 相对位置展开——frame=relative 时，把每个 target/via 的 pose.position
        加上 anchor.position（展开成绝对）。原地修改 data。返回 (成功, 错误消息)。

        只对笛卡尔 pose.position 做偏移；positions（关节）与 orientation 无相对语义，
        保持不变。frame 缺省 = absolute（向后兼容）。
        """
        frame = str(data.get("frame") or "absolute").strip().lower()
        if frame == "absolute":
            return True, ""
        if frame != "relative":
            return False, f"unknown frame '{frame}' (absolute/relative)"
        anchor = (data.get("anchor") or {}).get("position") or {}
        try:
            ax, ay, az = float(anchor["x"]), float(anchor["y"]), float(anchor["z"])
        except (KeyError, TypeError):
            return False, "relative frame missing anchor.position {x,y,z}"

        def shift(point: Dict[str, Any]) -> None:
            pose = point.get("pose") if isinstance(point, dict) else None
            if not pose or "position" not in pose:
                return
            p = pose["position"]
            p["x"] = float(p.get("x", 0.0)) + ax
            p["y"] = float(p.get("y", 0.0)) + ay
            p["z"] = float(p.get("z", 0.0)) + az

        for seg in (data.get("segments") or []):
            if seg.get("target"):
                shift(seg["target"])
            for v in (seg.get("via") or []):
                shift(v)
        # cp kind：顶层 via[] + target
        for v in (data.get("via") or []):
            shift(v)
        if data.get("target"):
            shift(data["target"])
        return True, ""

    def _play_stream(
        self, data: dict, label: str, path: str, resp: PlaybackTrajectory.Response
    ) -> PlaybackTrajectory.Response:
        """轨迹流（stream）逐段执行：每段末速度 0，含 mode/speed/sleep。

        F16x: 优先用预烘焙缓存 <path>.baked.yaml（存在则直接下发，跳过逐段 IK+重定时）。
        """
        baked_path = path + ".baked.yaml"
        if os.path.exists(baked_path):
            return self._play_baked(baked_path, label, resp)
        ok_rel, err_rel = self._expand_relative(data)
        if not ok_rel:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: {err_rel}")
            resp.success = False
            resp.message = err_rel
            return resp
        segments = data.get("segments") or []
        if not segments:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: empty stream")
            resp.success = False
            resp.message = "empty stream"
            return resp

        n = len(segments)
        seg_retries = max(0, int(self.get_parameter("waypoint_segment_retries").value))
        total = 0.0
        for i, seg in enumerate(segments):
            seg_label = f"stream {label} {i + 1}/{n}"
            ok = False
            msg = ""
            dur = 0.0
            for attempt in range(seg_retries + 1):
                ok, msg, dur = self._execute_segment(seg, seg_label)
                if ok:
                    break
                self.get_logger().warn(f"{seg_label} attempt {attempt + 1} failed: {msg}")
            if not ok:
                self._publish_mode("IDLE")
                self._set_state(STATE_READY, f"playback {label}: seg {i + 1} failed")
                resp.success = False
                resp.message = (
                    f"segment {i + 1}/{n} failed after {seg_retries + 1} attempt(s): {msg}")
                return resp
            total += dur
            # LL-103: 段间等 move_group 内部状态追上实际关节（防下段 LIN IK -31）
            if i < n - 1:
                self._wait_move_group_current(list(self._positions))

        self._schedule_back_to_ready(0.3)
        resp.success = True
        resp.message = f"stream '{label}': {n} segments, {total:.1f}s"
        return resp

    def _play_baked(
        self, baked_path: str, label: str, resp: PlaybackTrajectory.Response
    ) -> PlaybackTrajectory.Response:
        """F16x: 下发预烘焙关节轨迹缓存（跳过逐段 IK+重定时，提速关键）。"""
        try:
            with open(baked_path, "r", encoding="utf-8") as f:
                baked = yaml.load(f, Loader=_YamlSafeLoader)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"baked cache load failed: {exc}")
            resp.success = False
            resp.message = f"baked cache load failed: {exc}"
            return resp

        points = baked.get("points") or []
        if not points:
            resp.success = False
            resp.message = "baked cache empty"
            return resp

        traj = JointTrajectory()
        traj.joint_names = list(baked.get("joint_names") or JOINTS)
        for p in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in p["positions"]]
            pt.time_from_start = _duration(float(p.get("time_from_start_sec", 0.0)))
            traj.points.append(pt)

        self._publish_mode("TRAJ_RUNNING")
        ok, dmsg = self._dispatch_trajectory(traj)
        if not ok:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"{label} baked: {dmsg}")
            resp.success = False
            resp.message = dmsg
            return resp
        self._set_state(STATE_TRAJ, f"{label} baked")
        duration = self._traj_duration(traj)
        self._schedule_back_to_ready(duration + 0.3)
        resp.success = True
        resp.message = f"stream '{label}' (baked): {len(points)} pts, {duration:.1f}s"
        return resp

    def _play_cp(
        self, data: dict, label: str, resp: PlaybackTrajectory.Response
    ) -> PlaybackTrajectory.Response:
        """F163/F164: 连续笛卡尔路径（cp）——via 点列 + target 规划为一条 C2 轨迹。

        复用 _goto_via_smooth（关节过点样条）；mode=lin 可选走 _goto_via_lin_blend。
        schema: {kind: cp, joint_names, speed, mode, via:[{pose|positions},...], target}。
        """
        ok_rel, err_rel = self._expand_relative(data)
        if not ok_rel:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: {err_rel}")
            resp.success = False
            resp.message = err_rel
            return resp
        via = data.get("via") or []
        target = data.get("target") or {}
        if not via:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: cp empty via")
            resp.success = False
            resp.message = "cp task needs >= 1 via point"
            return resp
        speed = data.get("speed")
        if speed is not None:
            speed = float(speed)
        mode = str(data.get("mode") or "ptp").strip().lower()
        if mode == "lin":
            ok, msg, _ = self._goto_via_lin_blend(list(via), target, speed, label)
        else:
            ok, msg, _ = self._goto_via_smooth(list(via), target, speed, label)
        if not ok:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: cp failed")
            resp.success = False
            resp.message = msg
            return resp
        resp.success = True
        resp.message = f"cp '{label}': {len(via)} via + target ({msg})"
        return resp

    def _resolve_trajectories_dir(self) -> str:
        """Q4: 轨迹持久化目录——默认解析工作区根 <ws>/trajectories（不 git 跟踪），
        解析失败回退 ~/.a3/trajectories。trajectories_dir 参数被显式覆盖（非默认
        ~/.a3/trajectories）时优先用参数。"""
        raw = str(self.get_parameter("trajectories_dir").value)
        if raw != "~/.a3/trajectories":
            return os.path.expanduser(raw)
        try:
            share = get_package_share_directory("a3_arm_controller")
            ws = os.path.normpath(os.path.join(share, "..", "..", "..", ".."))
            if os.path.isdir(ws):
                return os.path.join(ws, "trajectories")
        except Exception:  # noqa: BLE001
            pass
        return os.path.expanduser("~/.a3/trajectories")

    def _stream_path(self, name: str) -> str:
        """轨迹流配置文件路径（Q4 搜索顺序）：
        1) 仓库 trajectories/streams/（运行时/实验槽，untracked）
        2) app_melodion/config/streams/（版本控制，脚本生成源）
        3) ~/.a3/trajectories/streams（旧槽）
        返回首个存在的；都不存在则返回首个候选（便于报错定位）。"""
        fname = _sanitize_name(name) + ".yaml"
        candidates = [os.path.join(self._traj_dir, "streams", fname)]
        try:
            candidates.append(os.path.join(
                get_package_share_directory("app_melodion"), "config", "streams", fname))
        except Exception:  # noqa: BLE001
            pass
        candidates.append(os.path.expanduser(
            os.path.join("~/.a3/trajectories", "streams", fname)))
        for c in candidates:
            if os.path.exists(c):
                return c
        return candidates[0]

    def _cp_path(self, name: str) -> str:
        """cp 路径配置文件路径：与 stream 同槽（软件编排，仓库/版本控制/旧槽搜索顺序）。"""
        return self._stream_path(name)

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

        # F67: 优先 move_group（OMPL + TOTG）；LL-077 move_group 只规划 arm 组，L7 补下发
        if bool(self.get_parameter("goto_use_moveit").value):
            ok, duration, msg = self._moveit_move(q1[:6], JOINTS[:6], f"goto {name}")
            if ok:
                # LL-103: 等 move_group current-state 追上实际关节，防下一段
                # （playback/stream 首段）从旧位规划导致时长/轨迹失真
                self._wait_move_group_current(q1)
                if self._n_joints >= 7:
                    # LL-077: move_group 只规划 arm 组，L7 需单独补下发；6J 变体无此关节
                    duration = max(duration, self._dispatch_l7_linear(q1[6]))
                self._schedule_back_to_ready(duration + 0.3)
                resp.success = True
                resp.message = msg
                return resp
            self.get_logger().warn(f"goto {name}: {msg} -- local linear fallback")

        q0 = list(self._positions)
        # F41/F88: 时长下限；两点轨迹由 JTC splines 插值
        duration = min(max(
            float(self.get_parameter("goto_duration_s").value),
            float(self.get_parameter("move_to_min_duration_s").value),
        ), 60.0)
        traj = self._two_point_trajectory(q0, q1, duration)
        duration = _tfs_seconds(traj.points[1].time_from_start)

        self._publish_mode("TRAJ_RUNNING")
        ok, dmsg = self._dispatch_trajectory(traj)
        if not ok:
            self._publish_mode("IDLE")
            resp.success = False
            resp.message = dmsg
            return resp
        self._set_state(STATE_TRAJ, f"goto {name}")
        self._schedule_back_to_ready(duration + 0.3)

        resp.success = True
        resp.message = f"goto {name} (two-point fallback, {duration:.1f}s, 2 pts)"
        return resp

    def _pad_pose7(self, q) -> List[float]:
        """把命名点位补齐/截断到 7 关节（L7 缺失补 0）。"""
        q = list(q)
        if len(q) < self._n_joints:
            q = q + [0.0] * (self._n_joints - len(q))
        elif len(q) > self._n_joints:
            q = q[: self._n_joints]
        return q

    def _select_tour_targets(
        self, rng: "random.Random", count: int, pool: List[str], leg_retries: int
    ) -> Tuple[List[str], List[List[float]], bool]:
        """F161: 随机抽取 count 个命名点位（固定序列不经此）。逐点静态门禁拒绝即重抽。

        返回 (names, targets, seq_failed)。seq_failed=True 表示重抽后仍凑不满 count，
        由调用方回退逐腿链。
        """
        GJ = JOINTS[:6]
        names: List[str] = []
        targets: List[List[float]] = []
        tprev: Optional[str] = None
        while len(names) < count:
            leg_excluded = {tprev} if tprev else set()
            got = False
            nm = ""
            for _attempt in range(leg_retries + 1):
                ch = [n for n in pool if n not in leg_excluded]
                if not ch:
                    break
                nm = rng.choice(ch)
                q = self._pad_pose7(self._poses[nm])
                q0_arm = (
                    [self._positions[self._joint_names.index(jn)] for jn in GJ]
                    if not targets else list(targets[-1][:6]))
                reject = (
                    self._static_segment_violation(
                        q0_arm, q[:6], GJ, len(names), samples=9)
                    if self._have_js else None)
                if reject:
                    self.get_logger().warn(
                        f"tour sequence leg {len(names) + 1} '{nm}' "
                        f"{reject} -- redraw")
                    leg_excluded.add(nm)
                    continue
                got = True
                break
            if not got:
                return names, targets, True
            names.append(nm)
            targets.append(q)
            tprev = nm
        return names, targets, False

    def _spline_bounds_violation(
        self, dense: List[List[float]], via: List[List[float]], group_joints: List[str]
    ) -> Optional[str]:
        """F161 兜底：稠密路径明显越限或超出 via 点包络 >0.3 rad 时返回原因，否则 None。

        三次样条在 via 点附近有轻微过冲；当 via 点靠近限位（如 home_front L2=0.127 贴
        近 0 下限）时，过冲可能轻微越限（~0.015 rad）。此类轻微越限由 JTC 软限位 clamp
        兜底，不拦截；只有明显越限（>0.05 rad）或大过冲（>0.3 rad，孤立极端 via 点
        的 Runge 振荡）才回退。
        """
        if not dense or not via:
            return None
        m = len(dense[0])
        LIM_TOL = 0.05  # 轻微越限容差（rad），JTC 软限位 clamp 兜底
        for j, jn in enumerate(group_joints):
            if j >= m:
                break
            col = [p[j] for p in dense]
            dmin, dmax = min(col), max(col)
            vmin = min(v[j] for v in via)
            vmax = max(v[j] for v in via)
            lo_hi = self._joint_limits.get(jn)
            if lo_hi is not None and (
                dmin < lo_hi[0] - LIM_TOL or dmax > lo_hi[1] + LIM_TOL
            ):
                return (f"{jn} out of URDF limits [{dmin:+.3f},{dmax:+.3f}] "
                        f"vs [{lo_hi[0]:.3f},{lo_hi[1]:.3f}]")
            over = max(vmin - dmin, dmax - vmax)
            if over > 0.3:
                return f"{jn} overshoot {over:.3f} rad beyond via envelope"
        return None

    def _tour_smooth_chain(
        self, tour_names: List[str], targets: List[List[float]],
        mode: str = "quintic",
    ) -> Optional[RandomPoseTour.Response]:
        """F161: 巡游平滑链（过点样条 + 时间重参数化 + JTC）。失败返回 None（回退）。"""
        GJ = JOINTS[:6]
        # 单 via 点 = 点对点 goto（move_group）
        if len(targets) == 1:
            ok, dur, msg = self._moveit_move(
                targets[0][:6], GJ, f"tour goto {tour_names[0]}")
            if not ok:
                self.get_logger().warn(f"tour goto failed ({msg}) -- fallback")
                return None
            self._schedule_back_to_ready(0.3)
            resp = RandomPoseTour.Response()
            resp.success = True
            resp.message = f"tour goto completed ({dur:.1f}s): {tour_names[0]}"
            resp.sequence = list(tour_names)
            resp.total_duration_s = dur
            return resp

        # 预检：逐段静态力矩 + 占空比（沿用 F134/F107 语义，失败即回退）
        qcur = [self._positions[self._joint_names.index(jn)] for jn in GJ]
        for seg_i, qt in enumerate(targets):
            reject = self._static_segment_violation(qcur, qt[:6], GJ, seg_i)
            if reject:
                self.get_logger().warn(f"tour smooth gate: {reject} -- fallback")
                return None
            qcur = list(qt[:6])
        est_total = 0.0
        q0 = [self._positions[self._joint_names.index(jn)] for jn in GJ]
        for qt in targets:
            d, _slow = self._velocity_floor_duration(q0, list(qt[:6]), GJ)
            est_total += d
            q0 = list(qt[:6])
        allowed, used_s, cooling_s = self._duty_check(max(est_total, 0.1))
        if not allowed:
            self.get_logger().warn(
                f"tour smooth duty gate rejected ({used_s:.0f}s in window, "
                f"wait {cooling_s:.0f}s) -- fallback")
            return None

        # 当前位 + via 点 → 保形插值（PCHIP）稠密路径
        q_meas = [self._positions[self._joint_names.index(jn)] for jn in GJ]
        resample_n = max(
            20, int(self.get_parameter("random_tour_smooth_resample_n").value))
        via_all = [q_meas] + [list(qt[:6]) for qt in targets]
        dense = _interpolate_via_spline(via_all, resample_n)
        if dense is None:
            self.get_logger().warn("tour smooth spline failed -- fallback")
            return None

        # 过冲/越限兜底：任一关节越 URDF 限位，或超出 via 点包络 > 0.3 rad，则回退
        # （PCHIP 保形本应零过冲，此处防御异常 via 点/插值退化）。
        bounds_viol = self._spline_bounds_violation(dense, via_all, GJ)
        if bounds_viol:
            self.get_logger().warn(
                f"tour smooth spline bounds: {bounds_viol} -- fallback")
            return None

        # 时间重参数化：quintic（jerk 连续，慢）或 S-curve（巡航段，快）
        vmax = float(
            self.get_parameter("random_tour_smooth_max_vel_rad_s").value)
        amax = float(
            self.get_parameter("random_tour_smooth_max_acc_rad_s2").value)
        if mode == "s_curve":
            timed = _s_curve_timed_traj(dense, vmax, amax)
        else:
            timed = _jerk_limited_timed_traj(dense, vmax, amax)
        if timed is None:
            self.get_logger().warn("tour smooth timing failed -- fallback")
            return None
        times, tpos = timed

        traj = JointTrajectory()
        traj.joint_names = list(GJ)
        for t, pos in zip(times, tpos):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in pos]
            pt.time_from_start = _duration(t)
            traj.points.append(pt)

        # JTC 下发（L1–L6；L7 不参与保持 0）
        self._publish_mode("TRAJ_RUNNING")
        ok, dmsg = self._dispatch_trajectory(traj)
        if not ok:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"tour smooth: {dmsg}")
            return None
        self._set_state(STATE_TRAJ, "tour smooth")
        duration = self._traj_duration(traj)
        self._schedule_back_to_ready(duration + 0.3)

        resp = RandomPoseTour.Response()
        resp.success = True
        metric_s = ""
        m = _finite_diff_metrics(times, tpos)
        if m is not None:
            metric_s = f" j_rms={m[1]:.4f} a_rms={m[0]:.4f}"
        resp.message = (
            f"tour smooth completed ({len(tour_names)} via, "
            f"{len(traj.points)} pts, {duration:.1f}s{metric_s})")
        resp.sequence = list(tour_names)
        resp.total_duration_s = duration
        self.get_logger().info(
            f"random tour smooth ({len(tour_names)} via, {duration:.1f}s): "
            f"{tour_names}")
        return resp

    def _tour_sequence_chain(
        self, tour_names: List[str], targets: List[List[float]]
    ) -> Optional[RandomPoseTour.Response]:
        """F134: 整序列一次规划/执行（pilz PTP + blend）。失败返回 None（回退逐腿）。"""
        count = len(tour_names)
        blend = float(self.get_parameter("random_tour_blend_radius_m").value)
        GJ = JOINTS[:6]
        est_total = 0.0
        q0 = [self._positions[self._joint_names.index(jn)] for jn in GJ]
        for qt in targets:
            d, _slow = self._velocity_floor_duration(q0, list(qt[:6]), GJ)
            est_total += d
            q0 = list(qt[:6])
        allowed, used_s, cooling_s = self._duty_check(max(est_total, 0.1))
        if not allowed:
            self.get_logger().warn(
                f"tour sequence duty gate rejected ({used_s:.0f}s in window, "
                f"wait {cooling_s:.0f}s) -- per-leg fallback")
            return None
        xyz_seq = [self._fk_ee_xyz(list(self._positions))]
        xyz_seq += [self._fk_ee_xyz(qt) for qt in targets]
        if any(x is None for x in xyz_seq):
            radii = [blend if i < count - 1 else 0.0 for i in range(count)]
        else:
            radii = self._adaptive_blend_radii(xyz_seq, blend)
        seq_items = [
            self._build_ptp_sequence_item(qt[:6], radii[i])
            for i, qt in enumerate(targets)]
        ok, stotal, smsg = self._sequence_move_group(
            seq_items, f"tour {count} legs")
        if not ok:
            self.get_logger().warn(
                f"tour sequence failed ({smsg}) -- per-leg fallback")
            return None
        self._schedule_back_to_ready(0.3)  # 同步已执行完，仅短缓冲
        resp = RandomPoseTour.Response()
        resp.success = True
        metric_s = ""
        if "j_rms=" in smsg:
            metric_s = " " + smsg[smsg.index("j_rms="):]
        resp.message = (
            f"tour sequence completed ({count} legs, {stotal:.1f}s, "
            f"blend r<={max(radii):.2f}): {' -> '.join(tour_names)}{metric_s}")
        resp.sequence = list(tour_names)
        resp.total_duration_s = stotal
        self.get_logger().info(
            f"random tour sequence ({count} legs, {stotal:.1f}s): {tour_names}")
        return resp

    def _tour_perleg_chain(
        self,
        tour_names: List[str],
        targets: List[List[float]],
        count: int,
        rng: "random.Random",
        pool: List[str],
        leg_retries: int,
    ) -> RandomPoseTour.Response:
        """F116: 逐腿规划+重试（回退链）。固定/预选序列按序执行；空序列随机逐腿重抽。"""
        GJ = JOINTS[:6]
        sequence: List[str] = []
        total = 0.0
        prev: Optional[str] = None
        n = len(tour_names) if tour_names else count
        for i in range(n):
            excluded = {prev} if prev else set()
            ok = False
            dur = 0.0
            msg = ""
            name = ""
            for attempt in range(leg_retries + 1):
                if tour_names:
                    name = tour_names[i]
                    target6 = list(targets[i][:6])
                else:
                    choices = [p for p in pool if p not in excluded]
                    if not choices:
                        msg = f"tour pool exhausted ({len(pool)} poses all tried)"
                        break
                    name = rng.choice(choices)
                    target6 = self._pad_pose7(self._poses[name])[:6]
                ok, dur, msg = self._moveit_move(
                    target6, GJ,
                    f"tour {i + 1}/{n} {name}"
                    + (f" (retry {attempt})" if attempt else ""),
                )
                if ok:
                    break
                self.get_logger().warn(
                    f"tour leg {i + 1}/{n} '{name}' attempt {attempt + 1} "
                    f"failed: {msg}")
                excluded.add(name)
            if not ok:
                self._publish_mode("IDLE")
                self._set_state(STATE_READY, f"tour aborted at leg {i + 1}")
                resp = RandomPoseTour.Response()
                resp.success = False
                resp.message = (
                    f"leg {i + 1}/{n} failed after {leg_retries + 1} attempt(s)"
                    f" (last '{name}'): {msg}")
                resp.sequence = sequence
                resp.total_duration_s = total
                return resp
            sequence.append(name)
            total += dur
            prev = name

        self._schedule_back_to_ready(0.3)  # 同步已执行完，仅短缓冲
        resp = RandomPoseTour.Response()
        resp.success = True
        resp.message = f"tour completed: {' -> '.join(sequence)}"
        resp.sequence = sequence
        resp.total_duration_s = total
        self.get_logger().info(f"random tour ({n} legs, {total:.1f}s): {sequence}")
        return resp

    def _cartesian_tour_targets(
        self, req: RandomPoseTour.Request
    ) -> Tuple[Optional[List[str]], Optional[List[List[float]]], str]:
        """F173: 把请求里的笛卡尔 xyz 点解成巡游关节目标。

        返回 (names, targets, err)；err 非空 = 整批拒绝（显式失败，不静默跳点：
        某点不可达时报出序号与坐标，由调用方直接回 fail）。
        姿态策略 orientation_mode：0=free 姿态不限；1=hold_start 锁巡游起始末端姿态；
        2=lock_quat 锁 ref_quat。逐点用上一点解作 IK 种子，保证多解时腕部连续。
        """
        n = len(req.xyz)
        if n == 0 or n % 3 != 0:
            return None, None, f"xyz length {n} not a positive multiple of 3"
        npts = n // 3
        mode = int(req.orientation_mode)
        if mode not in (0, 1, 2):
            return None, None, (
                f"invalid orientation_mode {mode} (0=free/1=hold_start/2=lock_quat)")

        ref_R = None
        if mode == 1:
            ref_R = self._fk_ee_orientation(list(self._positions)[:6])
            if ref_R is None:
                return None, None, "orientation_mode=1 but pinocchio FK unavailable"
        elif mode == 2:
            if len(req.ref_quat) != 4:
                return None, None, (
                    f"orientation_mode=2 needs ref_quat length 4, got {len(req.ref_quat)}")
            import numpy as np  # type: ignore
            import pinocchio as pin  # type: ignore

            quat = pin.Quaternion(*[float(v) for v in req.ref_quat])
            ref_R = np.array(quat.normalized().matrix(), dtype=float)

        names: List[str] = []
        targets: List[List[float]] = []
        seed = [self._positions[self._joint_names.index(jn)] for jn in JOINTS[:6]]
        for i in range(npts):
            x, y, z = (float(req.xyz[3 * i]), float(req.xyz[3 * i + 1]),
                       float(req.xyz[3 * i + 2]))
            q6 = self._solve_ik_position(
                {"position": {"x": x, "y": y, "z": z}}, ref_R, seed=seed)
            if q6 is None and ref_R is not None:
                # 锁姿态是该点无解（位置+姿态不可兼得）：退化为姿态自由并 WARN，
                # 不因一个点的姿态约束否掉整条巡游（姿态锁定语义=尽量锁定）
                q6 = self._solve_ik_position(
                    {"position": {"x": x, "y": y, "z": z}}, seed=seed)
                if q6 is not None:
                    self.get_logger().warn(
                        f"F173 xyz point {i + 1}/{npts} 姿态锁定无解 "
                        "-- 该点退化为姿态自由")
            if q6 is None:
                return None, None, (
                    f"xyz point {i + 1}/{npts} ({x:+.3f},{y:+.3f},{z:+.3f}) IK failed "
                    "(unreachable or joint-limit)")
            names.append(f"xyz{i + 1}[{x:+.3f},{y:+.3f},{z:+.3f}]")
            targets.append(self._pad_pose7(q6))
            seed = list(q6)
        return names, targets, ""

    def _random_pose_tour_cb(
        self, req: RandomPoseTour.Request, resp: RandomPoseTour.Response
    ) -> RandomPoseTour.Response:
        """F116/F134/F161/F173: 巡游（随机命名点 / 固定命名序列 / 笛卡尔 xyz 直给）。

        F161: req.fixed_sequence 非空 → 跳过随机抽取与相邻去重，按给定序列执行；
        F173: req.xyz 非空 → 逐点 IK 成关节目标后走同一链（优先于 fixed_sequence）；
        random_tour_smooth_mode ∈ {blend, s_curve, quintic} 选平滑后端，
        任一环节失败 WARN 并回退 F134 sequence / F116 逐腿链。
        """
        can, why = self._can_move()
        if not can:
            resp.success = False
            resp.message = why
            return resp
        if not self._have_js:
            resp.success = False
            resp.message = "no /joint_states yet"
            return resp
        if not bool(self.get_parameter("goto_use_moveit").value):
            resp.success = False
            resp.message = "random tour requires goto_use_moveit=true (move_group)"
            return resp

        smooth_mode = str(self.get_parameter("random_tour_smooth_mode").value)
        use_smooth = smooth_mode in ("s_curve", "quintic")
        use_sequence = bool(self.get_parameter("random_tour_use_sequence").value)
        leg_retries = max(
            0, int(self.get_parameter("random_tour_leg_retries").value))

        rng = random.Random(None)  # 固定序列时未用
        pool: List[str] = []

        # ── 1. 解析点位序列 ──
        if req.xyz:
            # F173: 笛卡尔直给——逐点 IK；任一点不可达即整批拒绝（显式失败不跳点）
            tour_names, targets, err = self._cartesian_tour_targets(req)
            if err:
                resp.success = False
                resp.message = err
                return resp
            count = len(tour_names)
            self.get_logger().info(
                f"F173 cartesian tour: {count} xyz point(s), "
                f"orientation_mode={int(req.orientation_mode)}")
        elif req.fixed_sequence:
            tour_names = [str(n) for n in req.fixed_sequence]
            missing = [n for n in tour_names if n not in self._poses]
            if missing:
                resp.success = False
                resp.message = f"fixed_sequence unknown pose(s): {missing}"
                return resp
            targets = [self._pad_pose7(self._poses[n]) for n in tour_names]
            count = len(tour_names)
        else:
            count = int(req.count) if int(req.count) > 0 else int(
                self.get_parameter("random_tour_count").value)
            if count < 1:
                resp.success = False
                resp.message = f"invalid count {count}"
                return resp
            exclude = {
                str(n)
                for n in (self.get_parameter("random_tour_exclude_poses").value or [])
            }
            pool = [n for n in sorted(self._poses) if n not in exclude]
            if len(pool) < 2:
                resp.success = False
                resp.message = f"tour pool too small ({len(pool)}), need >= 2"
                return resp
            rng = random.Random(int(req.seed) if int(req.seed) != 0 else None)
            tour_names, targets, seq_failed = self._select_tour_targets(
                rng, count, pool, leg_retries)
            if seq_failed:
                self.get_logger().warn(
                    "tour sequence: no feasible poses after redraw "
                    "-- per-leg fallback")
                tour_names, targets = [], []

        # ── 2. 执行优先级：平滑链 → 序列链 → 逐腿链 ──
        if use_smooth and tour_names:
            out = self._tour_smooth_chain(tour_names, targets, smooth_mode)
            if out is not None:
                return out
            self.get_logger().warn(
                "tour smooth chain failed -- sequence/per-leg fallback")

        if use_sequence and tour_names:
            out = self._tour_sequence_chain(tour_names, targets)
            if out is not None:
                return out
            self.get_logger().warn("tour sequence failed -- per-leg fallback")

        return self._tour_perleg_chain(
            tour_names, targets, count, rng, pool, leg_retries)

    def _move_to_cb(
        self, req: MoveToJointPositions.Request, resp: MoveToJointPositions.Response
    ) -> MoveToJointPositions.Response:
        """F39: 通用平滑移动——当前位姿到任意目标位姿，duration_s 内多点插值."""
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

        # F67: 优先 move_group（L1–L6）；LL-077 L7 补下发
        if bool(self.get_parameter("goto_use_moveit").value):
            ok, duration, msg = self._moveit_move(q1[:6], JOINTS[:6], "move_to")
            if ok:
                # LL-077 / 6J：move_group 只规划 L1–L6，L7 需单独补下发；6J 无 L7
                if self._n_joints >= 7:
                    duration = max(duration, self._dispatch_l7_linear(q1[6]))
                self._schedule_back_to_ready(duration + 0.3)
                resp.success = True
                resp.message = msg
                return resp
            self.get_logger().warn(f"move_to: {msg} -- local linear fallback")

        q0 = list(self._positions)
        # F41/F88: 最短时长兜底（可配置，默认 3s）；两点轨迹由 JTC splines 插值
        duration = float(req.duration_s) if req.duration_s and req.duration_s > 0 else 1.0
        min_dur = float(self.get_parameter("move_to_min_duration_s").value)
        duration = min(max(duration, min_dur), 60.0)
        traj = self._two_point_trajectory(q0, q1, duration)
        duration = _tfs_seconds(traj.points[1].time_from_start)

        self._publish_mode("TRAJ_RUNNING")
        ok, dmsg = self._dispatch_trajectory(traj)
        if not ok:
            self._publish_mode("IDLE")
            resp.success = False
            resp.message = dmsg
            return resp
        self._set_state(STATE_TRAJ, f"move_to ({duration:.1f}s)")
        self._schedule_back_to_ready(duration + 0.3)

        resp.success = True
        resp.message = f"move_to (two-point fallback, {duration:.1f}s, 2 pts)"
        return resp

    def _save_named_pose_cb(
        self, req: SaveNamedPose.Request, resp: SaveNamedPose.Response
    ) -> SaveNamedPose.Response:
        """F39/F114: 保存命名点位（positions 留空 = 当前位姿）到包内 named_poses.yaml 并即时生效.

        F114: name 留空时自动命名 snap_YYYYMMDD_HHMMSS（手柄无法输入文本）；
        写入目标从 ~/.a3/poses.yaml 收敛到包内（F113 弃用用户层覆盖）。

        F131: 状态路由——WAYPOINT_TEACH 时本服务=打点（不写 named_poses.yaml）。
        F133: TEACH（短按 Share 连续示教）时允许随时保存当前位姿到 named_poses.yaml
        且不退出示教——拖到哪里按 L2 存哪里；与 WAYPOINT_TEACH 的 L2 打点互不相干。
        """
        if self._state == STATE_WAYPOINT_TEACH:
            return self._capture_waypoint(resp)
        in_teach = (self._state == STATE_TEACH)
        raw = (req.name or "").strip()
        name = (
            _sanitize_name(raw)
            if raw
            else datetime.now().strftime("snap_%Y%m%d_%H%M%S")
        )
        if req.positions:
            if len(req.positions) != self._n_joints:
                resp.success = False
                resp.message = f"need {self._n_joints} positions, got {len(req.positions)}"
                return resp
            q = [round(float(v), 4) for v in req.positions]
        else:
            if not self._have_js:
                resp.success = False
                resp.message = "no /joint_states yet"
                return resp
            q = [round(float(p), 4) for p in self._positions]

        try:
            path = self._named_poses_pkg_path()
            with open(path, "r", encoding="utf-8") as f:
                lines = f.readlines()
            pos_line = f"    positions: [{', '.join(f'{v:.4f}' for v in q)}]"
            lines = _upsert_pose_lines(lines, name, pos_line)
            with open(path, "w", encoding="utf-8") as f:
                f.writelines(lines)
        except Exception as exc:  # noqa: BLE001
            resp.success = False
            resp.message = f"save failed: {exc}"
            return resp

        self._poses[name] = q
        # F133: 示教中保存 → 白闪+弱震即时反馈，不退出示教（服务响应手柄侧不可见）
        if in_teach:
            self._emit_operator_event("named_pose_saved")
        resp.success = True
        suffix = " (示教中保存，继续拖动)" if in_teach else ""
        resp.message = (
            f"saved pose '{name}' (当前位姿 → a3_description/config/named_poses.yaml)"
            + suffix
        )
        resp.path = path
        self.get_logger().info(f"named pose saved: {name} -> {[round(v, 4) for v in q]}")
        return resp

    def _set_joint_positions_cb(
        self, req: SetJointPositions.Request, resp: SetJointPositions.Response
    ) -> SetJointPositions.Response:
        """Web 滑动条 jog 直驱：设 7 关节目标位置，短插值下发执行层（需求 F23 扩展）."""
        if not self._have_js:
            resp.success = False
            resp.message = "no /joint_states yet"
            return resp
        if self._state in (
            STATE_INIT, STATE_TEACH, STATE_WAYPOINT_TEACH, STATE_AI, STATE_FAULT, STATE_SERVO,
            STATE_SAFE_PARK, STATE_DISABLED, STATE_COOLING,
        ):
            resp.success = False
            resp.message = f"busy in state={self._state}"
            return resp
        if self._mode in BLOCKED_MODES:
            resp.success = False
            resp.message = f"mode={self._mode}"
            return resp
        if self._feedback_stale:
            resp.success = False
            resp.message = "feedback stale: protective freeze-hold active"
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
        # F88: 两点轨迹，JTC splines 控制器侧插值
        traj = self._two_point_trajectory(q0, target, duration)
        duration = _tfs_seconds(traj.points[1].time_from_start)

        self._publish_mode("TRAJ_RUNNING")
        ok, dmsg = self._dispatch_trajectory(traj)
        if not ok:
            self._publish_mode("IDLE")
            resp.success = False
            resp.message = dmsg
            return resp
        self._jogging = True
        self._set_state(STATE_TRAJ, "jog")
        self._schedule_back_to_ready(duration + 0.5)

        resp.success = True
        msg = f"jog {self._n_joints} joints ({duration:.2f}s)"
        if clamped:
            msg += f"; clamped {','.join(sorted(set(clamped)))}"
        resp.message = msg
        return resp

    def _teach_guard(self) -> Optional[str]:
        """F131: 进入任一示教模式的共同前置；None=通过，否则=拒绝原因."""
        if self._state not in (STATE_READY,):
            return f"require READY (now {self._state})"
        if self._mode in BLOCKED_MODES:
            return f"mode={self._mode}"
        return None

    def _enter_freedrive(self) -> Tuple[bool, str]:
        """F131: 进入自由拖动（标准栈原子 controller swap / 旧栈 zero_torque 服务）."""
        if self._using_controller_switch():
            return self._cm_freedrive_switch(True)
        return self._call_trigger(self._zt_start_cli, "zero_torque/start")

    def _exit_freedrive(self) -> Tuple[bool, str]:
        """F131: 退出自由拖动（恢复位置闭环）."""
        if self._using_controller_switch():
            return self._cm_freedrive_switch(False)
        return self._call_trigger(self._zt_stop_cli, "zero_torque/stop")

    def _emit_operator_event(self, event: str) -> None:
        """F131: 发操作员事件（ds4_feedback_node 白闪+弱震消费）."""
        msg = String()
        msg.data = event
        self._operator_event_pub.publish(msg)

    def _fk_pose(self, positions: List[float]) -> Optional[dict]:
        """
        F131: pinocchio FK → end_effector 相对 base_link 的位姿（YAML-ready dict）.

        模型不可用时返回 None（打点不阻断，但该路点无法走 LIN 段）。
        """
        try:
            if not self._pin_ready:
                self._ensure_pin_model()
            import numpy as np  # type: ignore
            import pinocchio as pin  # type: ignore

            model = self._pin_model
            q = np.zeros(model.nq)
            for pos, qi in zip(positions, self._pin_q_idx):
                q[qi] = pos
            data = model.createData()
            ee = str(self.get_parameter("waypoint_ee_link").value)
            fid = model.getFrameId(ee)
            pin.framesForwardKinematics(model, data, q)
            placement = data.oMf[fid]
            # eigenpy ≥3（pinocchio 4.0）coeffs 是方法而非属性，须调用
            coeffs = pin.Quaternion(placement.rotation).coeffs()  # [x, y, z, w]
            return {
                "position": {
                    "x": float(placement.translation[0]),
                    "y": float(placement.translation[1]),
                    "z": float(placement.translation[2]),
                },
                "orientation": {
                    "x": float(coeffs[0]),
                    "y": float(coeffs[1]),
                    "z": float(coeffs[2]),
                    "w": float(coeffs[3]),
                },
            }
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"F131 FK pose failed: {exc}", throttle_duration_sec=30.0)
            return None

    def _capture_waypoint(
        self, resp: SaveNamedPose.Response
    ) -> SaveNamedPose.Response:
        """F131: L2 打点——追加当前 7 关节位姿 + FK 位姿（去重 + 灯效反馈）."""
        if not self._have_js:
            resp.success = False
            resp.message = "no /joint_states yet"
            return resp
        q = [round(float(p), 4) for p in self._positions]
        min_d = float(self.get_parameter("waypoint_min_capture_delta_rad").value)
        if self._waypoints:
            last = self._waypoints[-1]["positions"]
            if max(abs(a - b) for a, b in zip(q, last)) < min_d:
                resp.success = True
                resp.message = (
                    f"waypoint unchanged (still {len(self._waypoints)}), skip")
                return resp
        pose = self._fk_pose(q)
        self._waypoints.append({"positions": q, "pose": pose})
        idx = len(self._waypoints)
        self._emit_operator_event("waypoint_captured")
        resp.success = True
        resp.message = f"waypoint {idx} captured" + (
            "" if pose else " (FK unavailable; LIN disabled for this task)")
        self.get_logger().info(f"waypoint {idx} captured: {q}")
        return resp

    def _start_teach_cb(
        self, req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        why = self._teach_guard()
        if why is not None:
            resp.success = False
            resp.message = why
            return resp

        ok, msg = self._enter_freedrive()
        if not ok:
            resp.success = False
            resp.message = msg
            return resp

        self._record = []
        self._record_start = time.monotonic()
        self._recording = True
        if self._using_controller_switch():
            # 回环 echo 置 _mode=ZERO_TORQUE，BLOCKED_MODES 门禁与 legacy 对齐
            self._publish_mode("ZERO_TORQUE")
        self._set_state(STATE_TEACH, "teaching (drag)")
        resp.success = True
        resp.message = "teach started"
        return resp

    def _start_waypoint_teach_cb(
        self, req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        """F131（Share 长按 1.5s）：路点示教——L2 打点，Options 结束保存."""
        why = self._teach_guard()
        if why is not None:
            resp.success = False
            resp.message = why
            return resp

        ok, msg = self._enter_freedrive()
        if not ok:
            resp.success = False
            resp.message = msg
            return resp

        self._waypoints = []
        if self._using_controller_switch():
            self._publish_mode("ZERO_TORQUE")
        self._set_state(STATE_WAYPOINT_TEACH, "waypoint teaching (L2=mark)")
        resp.success = True
        resp.message = "waypoint teach started; L2 to capture, Options to finish"
        return resp

    def _stop_teach_cb(
        self, req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        if self._state == STATE_WAYPOINT_TEACH:
            return self._stop_waypoint_teach(resp)
        if self._state != STATE_TEACH:
            resp.success = False
            resp.message = f"not teaching (state={self._state})"
            return resp
        self._recording = False
        if self._using_controller_switch():
            # 先恢复位置闭环：失败则保持 TEACH（重力补偿仍在），由操作员重试，
            # 不许报成功后臂还在自由态。
            ok, msg = self._exit_freedrive()
            if not ok:
                self._recording = True
                resp.success = False
                resp.message = (
                    f"free-drive exit failed: {msg}; arm still in TEACH, "
                    "retry stop_teach")
                return resp
            self._publish_mode("READY")
        else:
            ok, msg = self._exit_freedrive()
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

    def _dump_waypoints(self) -> dict:
        """F131: 路点任务序列化（含 FK 位姿，供 LIN 段使用）."""
        return {
            "kind": "waypoint",
            "joint_names": list(self._joint_names),
            "waypoints": [
                {
                    "positions": [float(v) for v in wp["positions"]],
                    "pose": wp.get("pose") or {},
                }
                for wp in self._waypoints
            ],
        }

    def _save_waypoints_to(self, path: str) -> bool:
        """把当前路点任务写到 path，返回是否成功（异常已记录，不抛）."""
        try:
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(self._dump_waypoints(), f)
            self.get_logger().info(f"waypoint task saved to {path}")
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"waypoint save failed to {path}: {exc}")
            return False

    def _stop_waypoint_teach(self, resp: Trigger.Response) -> Trigger.Response:
        """F131: Options 结束路点示教 → 退出 free-drive → 自动落盘."""
        if self._using_controller_switch():
            ok, msg = self._exit_freedrive()
            if not ok:
                resp.success = False
                resp.message = (
                    f"free-drive exit failed: {msg}; arm still in "
                    "WAYPOINT_TEACH, retry Options")
                return resp
            self._publish_mode("READY")
        else:
            ok, msg = self._exit_freedrive()
        n = len(self._waypoints)
        min_count = int(self.get_parameter("waypoint_min_count").value)
        auto_msgs: List[str] = []
        if n >= min_count:
            latest = os.path.join(self._wp_dir, "latest.yaml")
            if self._save_waypoints_to(latest):
                auto_msgs.append("auto-saved waypoints/latest.yaml")
                ts = time.strftime("%Y%m%d_%H%M%S")
                hist = os.path.join(self._wp_dir, f"teach_wp_{ts}.yaml")
                if self._save_waypoints_to(hist):
                    auto_msgs.append(f"backup teach_wp_{ts}.yaml")
            else:
                auto_msgs.append("auto-save FAILED (see log)")
        else:
            auto_msgs.append(
                f"auto-save skipped ({n} waypoints < {min_count}, keep previous latest)"
            )
        self._set_state(STATE_READY, f"waypoint teach stopped ({n} points)")
        resp.success = True
        resp.message = f"recorded {n} waypoints; " + ", ".join(auto_msgs) + (
            f" (zero_torque/stop: {msg})" if not ok else ""
        )
        return resp

    def _dump_recording(self) -> dict:
        """
        F54: 把当前内存录制(self._record: (t, pos) 列表)序列化为磁盘 YAML 数据.

        F57/LL-053: 保存时对 positions 做轻量平滑（teach_save_smooth_samples，默认 5 点
        中心滑动平均，复用 _smooth_points 凸组合不越包络）——latest.yaml 与
        teach_*.yaml 落盘即干净，任何消费方受益；时间轴不动。w<3 时原样保存。
        """
        rec = self._record
        w = int(self.get_parameter("teach_save_smooth_samples").value)
        if w >= 3 and len(rec) >= 4:
            tmp: List[JointTrajectoryPoint] = []
            for t, pos in rec:
                q = JointTrajectoryPoint()
                q.positions = [float(p) for p in pos]
                q.time_from_start = _duration(float(t))
                tmp.append(q)
            rec = [
                (
                    pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9,
                    pt.positions,
                )
                for pt in _smooth_points(tmp, w)
            ]
        data = {
            "joint_names": list(self._joint_names),
            "points": [
                {
                    "positions": [float(p) for p in pos],
                    "time_from_start_sec": float(t),
                }
                for t, pos in rec
            ],
        }
        return data

    def _save_recording_to(self, path: str) -> bool:
        """把当前录制写到 path，返回是否成功（异常已记录日志，不抛）."""
        try:
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(self._dump_recording(), f)
            self.get_logger().info(f"trajectory saved to {path}")
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"trajectory save failed to {path}: {exc}")
            return False

    def _writeback_retimed(
        self, path: str, traj: Optional[JointTrajectory], f137_eps: float = 0.0
    ) -> bool:
        """F133: 把 Ruckig 重定时后的轨迹烘焙回录制品 YAML（latest 槽优化沉淀）.

        几何路径不变、时间戳为重定时结果；写前把原文件备份为 *.preretime.yaml
        （只保最近一代）。失败仅 WARN，不影响本次回放。
        """
        if traj is None or not traj.points:
            return False
        try:
            if os.path.exists(path):
                shutil.copy2(path, path.replace(".yaml", ".preretime.yaml"))
            data = {
                # 已优化标记：下次回放据此跳过重复烘焙（几何已含一代重采样误差）
                "retimed": True,
                # F137 已去噪标记：下次回放据此跳过二次 B 样条去噪（几何已含去噪结果）
                "f137_smoothed": f137_eps > 0,
                "joint_names": list(traj.joint_names),
                "points": [
                    {
                        "positions": [float(v) for v in pt.positions],
                        "time_from_start_sec": round(
                            float(pt.time_from_start.sec)
                            + float(pt.time_from_start.nanosec) * 1e-9,
                            4,
                        ),
                    }
                    for pt in traj.points
                ],
            }
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f)
            self.get_logger().info(
                f"retime writeback: {path} ({len(data['points'])} pts, "
                "backup *.preretime.yaml)"
            )
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"retime writeback failed ({path}): {exc}")
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

    def _l7_recorded_traj(
        self, geo: List[List[float]], geo_times: List[float]
    ) -> JointTrajectory:
        """L7 原始录制时间戳轨迹（retime 排除 L7 后单独下发，不做 Ruckig）。"""
        li = self._joint_names.index("L7_joint")
        traj = JointTrajectory()
        traj.joint_names = [JOINTS[6]]
        for pos, t in zip(geo, geo_times):
            pt = JointTrajectoryPoint()
            pt.positions = [float(pos[li])]
            pt.time_from_start = _duration(t)
            traj.points.append(pt)
        return traj

    def _merge_retimed_with_l7(
        self, arm_traj: JointTrajectory, geo: List[List[float]], geo_times: List[float]
    ) -> JointTrajectory:
        """把 retime 后的 L1–L6 与原始录制的 L7 拼成 7 关节单时间轴轨迹。

        用于 writeback 落盘（文件格式要求单时间轴 7 关节）；下发仍走独立 l7_traj
        （保留原始录制时间戳）。L7 线性重采样到 retime 时间轴。
        """
        li = self._joint_names.index("L7_joint")
        l7_vals = [float(pos[li]) for pos in geo]
        out = JointTrajectory()
        out.joint_names = list(self._joint_names)
        for pt in arm_traj.points:
            t = _tfs_seconds(pt.time_from_start)
            row = [0.0] * len(self._joint_names)
            for k, name in enumerate(arm_traj.joint_names):
                if name in self._joint_names:
                    row[self._joint_names.index(name)] = float(pt.positions[k])
            row[li] = _interp_series(l7_vals, geo_times, t)
            np = JointTrajectoryPoint()
            np.positions = row
            np.time_from_start = pt.time_from_start
            out.points.append(np)
        return out

    def _call_retime(
        self,
        geo_points: List[List[float]],
        joint_names: List[str],
        backend: str,
        v_scaling: float,
        a_scaling: float,
        geo_times: Optional[List[float]] = None,
        target_duration: float = 0.0,
        velocity_limits: Optional[List[float]] = None,
        acceleration_limits: Optional[List[float]] = None,
        timeout_s: float = 5.0,
    ) -> Tuple[bool, Optional[JointTrajectory], str]:
        """
        F68: 同步（轮询）调 /a3/arm/retime_trajectory。geo_times 给出各点.

        time_from_start（录制时刻 + ramp 偏移），用于 Ruckig 播种。
        F167: velocity_limits/acceleration_limits 非空时按 group 顺序下发逐关节**绝对**
        上限（服务端忽略 scaling），用于 goto_via 复现自身 caps。
        """
        if not self._wait_service(self._retime_cli, 1.0):
            return False, None, "retime service unavailable"
        req = RetimeTrajectory.Request()
        req.trajectory.joint_names = list(joint_names)
        for i, pos in enumerate(geo_points):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in pos]
            if geo_times is not None:
                pt.time_from_start = _duration(geo_times[i])
            req.trajectory.points.append(pt)
        req.backend = backend
        req.velocity_scaling = v_scaling
        req.acceleration_scaling = a_scaling
        req.target_duration = target_duration
        if velocity_limits:
            req.velocity_limits = [float(x) for x in velocity_limits]
        if acceleration_limits:
            req.acceleration_limits = [float(x) for x in acceleration_limits]
        future = self._retime_cli.call_async(req)
        if not self._wait_future(future, float(timeout_s)):
            return False, None, "retime call timeout"
        result = future.result()
        if result.success:
            return True, result.trajectory, result.message
        return False, None, result.message

    @staticmethod
    def _traj_duration(traj: JointTrajectory) -> float:
        if not traj.points:
            return 0.0
        last = traj.points[-1].time_from_start
        return float(last.sec) + float(last.nanosec) * 1e-9

    def _prepare_playback(
        self, req: PlaybackTrajectory.Request, resp: PlaybackTrajectory.Response
    ) -> Tuple[Optional[dict], str, str, str, str, Optional[PlaybackTrajectory.Response]]:
        """Q3: 三类轨迹 kind 共用的回放前导——解析 type/strategy → 定 path/label →
        存在性 → _can_move → 发布 TRAJ_RUNNING → 加载 YAML → kind 双向门禁。

        返回 (data, label, kind, path, strategy, failed)：成功 data 非空、failed=None；
        失败 data=None、failed=已填好 success/message 的 resp。
        """
        is_wp = (req.type or "").strip().lower() == "waypoint"
        is_stream = (req.type or "").strip().lower() == "stream"
        is_cp = (req.type or "").strip().lower() == "cp"
        strategy = (req.strategy or "ptp").strip().lower()
        if is_wp and strategy not in ("ptp", "lin"):
            resp.success = False
            resp.message = f"unknown strategy '{strategy}' (ptp/lin)"
            return None, "", "", "", strategy, resp
        # F54: 空名 = 回放 latest 槽（分支在 sanitize 之前，_sanitize_name("")→"trajectory" 会撞纸面名）
        is_latest = not (req.name or "").strip()
        if (is_stream or is_cp) and is_latest:
            resp.success = False
            resp.message = "stream/cp requires explicit name"
            return None, "", "", "", strategy, resp
        prefix = "wp_" if is_wp else ("st_" if is_stream else ("cp_" if is_cp else ""))
        label = prefix + ("latest" if is_latest else _sanitize_name(req.name))
        if is_stream:
            path = self._stream_path(req.name)
        elif is_cp:
            path = self._cp_path(req.name)
        else:
            base_dir = self._wp_dir if is_wp else self._traj_dir
            path = _traj_path_for(base_dir, req.name)
        if not os.path.exists(path):
            if is_latest:
                resp.success = False
                resp.message = (
                    ("no latest waypoint task yet —— Share 长按进入路点示教 → "
                     "L2 打点 → Options 保存, ") if is_wp
                    else ("no latest trajectory yet —— 先 start_teach → 拖动 → "
                          "stop_teach 录制, ")
                    + "或显式指定 name")
            else:
                resp.success = False
                resp.message = f"trajectory not found: {path}"
            return None, "", "", "", strategy, resp
        can, why = self._can_move()
        if not can:
            resp.success = False
            resp.message = why
            return None, "", "", "", strategy, resp

        # F129: 通过门禁即发布 TRAJ_RUNNING + TRAJ 状态 → 灯带立即变色，不再等
        # YAML 解析（纯 Python loader 在满载节点内实测 3–4s）。cyan/purple 由
        # 回首点是否走 MoveIt 决定（灯效前缀 playback return）。
        use_moveit = bool(self.get_parameter("playback_return_use_moveit").value)
        self._publish_mode("TRAJ_RUNNING")
        self._set_state(
            STATE_TRAJ,
            (f"waypoint {strategy} {label}" if is_wp
             else (f"playback return {label}" if use_moveit else f"playback {label}")),
        )

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.load(f, Loader=_YamlSafeLoader)
        except Exception as exc:  # noqa: BLE001
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: load failed")
            resp.success = False
            resp.message = f"load failed: {exc}"
            return None, "", "", "", strategy, resp

        # F131: kind 双向门禁——杜绝连续轨迹被逐段规划、路点任务被短按原样回放
        file_kind = str(data.get("kind") or "continuous")
        if is_wp and file_kind != "waypoint":
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: kind gate")
            resp.success = False
            resp.message = (
                f"'{label}' is {file_kind}, not a waypoint task —— "
                "Square 短按回放连续轨迹；路点回放请先 Share 长按录制")
            return None, "", "", "", strategy, resp
        if is_stream and file_kind != "stream":
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: kind gate")
            resp.success = False
            resp.message = (
                f"'{label}' is {file_kind}, not a stream task —— "
                "轨迹流请用 type=stream 回放")
            return None, "", "", "", strategy, resp
        if is_cp and file_kind != "cp":
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: kind gate")
            resp.success = False
            resp.message = (
                f"'{label}' is {file_kind}, not a cp task —— "
                "连续笛卡尔路径请用 type=cp 回放")
            return None, "", "", "", strategy, resp
        if not is_wp and not is_stream and not is_cp and file_kind in ("waypoint", "stream", "cp"):
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: kind gate")
            resp.success = False
            resp.message = (
                f"'{label}' is a {file_kind} task —— 用 type={file_kind} 回放")
            return None, "", "", "", strategy, resp

        return data, label, file_kind, path, strategy, None

    def _playback_cb(
        self, req: PlaybackTrajectory.Request, resp: PlaybackTrajectory.Response
    ) -> PlaybackTrajectory.Response:
        data, label, kind, path, strategy, failed = self._prepare_playback(req, resp)
        if data is None:
            return failed
        if kind == "waypoint":
            return self._play_waypoints(data, label, strategy, resp)
        if kind == "stream":
            return self._play_stream(data, label, path, resp)
        if kind == "cp":
            return self._play_cp(data, label, resp)
        return self._play_continuous(data, label, path, resp)

    def _play_continuous(
        self, data: dict, label: str, path: str, resp: PlaybackTrajectory.Response
    ) -> PlaybackTrajectory.Response:
        """连续轨迹回放（F137 去噪 + F68 retime + JTC），从 _playback_cb 内联段抽出，逻辑不变。"""
        use_moveit = bool(self.get_parameter("playback_return_use_moveit").value)
        already_retimed = bool(data.get("retimed"))

        file_joint_names = list(data.get("joint_names") or self._joint_names)
        recorded_positions: List[List[float]] = []
        recorded_times: List[float] = []
        for p in data.get("points") or []:
            recorded_positions.append([float(v) for v in p["positions"]])
            recorded_times.append(float(p["time_from_start_sec"]))

        if not recorded_positions:
            self._publish_mode("IDLE")
            self._set_state(STATE_READY, f"playback {label}: empty trajectory")
            resp.success = False
            resp.message = "empty trajectory"
            return resp
        recorded_duration = max(recorded_times) if recorded_times else 0.0

        # 统一到标准关节序（L1..L7）；文件缺关节时补 0（retime 组要求 7 关节齐备）
        def _to_std(positions: List[float]) -> List[float]:
            if file_joint_names == self._joint_names:
                return list(positions)
            out = []
            for jn in self._joint_names:
                if jn in file_joint_names:
                    out.append(positions[file_joint_names.index(jn)])
                else:
                    out.append(0.0)
            return out

        recorded_std = [_to_std(p) for p in recorded_positions]

        # F129: 轨迹本体静态重力矩 gate（与点到点 _moveit_move 同款 F107）。
        # 静态重力矩随位形连续平滑变化，均匀抽样（≤96 点，含首末）足以捕获峰值，
        # 属工程近似（局限已记入 SAFETY.md）。
        n_body = len(recorded_std)
        gate_n = min(n_body, 96)
        gate_idxs = sorted({0, n_body - 1} | {
            int(round(i * (n_body - 1) / max(gate_n - 1, 1)))
            for i in range(gate_n)})
        for bi in gate_idxs:
            viol = self._static_torque_violations(recorded_std[bi])
            if viol:
                self._publish_mode("IDLE")
                self._set_state(STATE_READY, f"playback {label}: static gate")
                resp.success = False
                resp.message = (
                    f"static torque gate rejected at body point {bi}/{n_body}: "
                    + "; ".join(viol))
                return resp

        # F137: 几何去噪（B 样条平滑）——在 F68 重定时前把录制点几何光顺，去编码器
        # 量化锯齿。ε>0 启用；文件已带 f137_smoothed 标记（已烘焙过去噪结果）则跳过，
        # 避免对已去噪几何二次去噪。
        f137_smoothed = bool(data.get("f137_smoothed"))
        eps = float(self.get_parameter("playback_geometric_smoothing_eps").value)
        if eps > 0 and not f137_smoothed and len(recorded_std) >= 6:
            recorded_std = _geometric_smooth_positions(recorded_std, recorded_times, eps)

        # F38/F88: 回放前从当前位姿到首记录点建一条两点几何 ramp（时间由 retime
        # 重算，保几何），避免回放起始位 ≠ 记录起始位时突然跳变。
        ramp_s = float(self.get_parameter("playback_ramp_duration_s").value)
        # 当前位姿 ≈ 首记录点时跳过 ramp
        ramp_dist = (
            max(abs(a - b) for a, b in zip(self._positions, recorded_std[0]))
            if self._have_js else 0.0
        )
        use_ramp = ramp_s > 0.05 and self._have_js and ramp_dist > 0.02
        # F124: use_ramp 且 playback_return_use_moveit=true 时，回首点段改用 move_group
        # 轨迹规划（OMPL+TOTG，经 FJT→执行层），覆盖几何 ramp；灯带该段 cyan。MoveIt
        # 不可用/规划/执行失败 → WARN 并回落 join-ramp（与 goto 现行为一致）。
        first_q = recorded_std[0]
        moved = False
        r_dur = 0.0
        if use_ramp and use_moveit:
            ok, dur, msg = self._moveit_move(
                first_q[:6], JOINTS[:6], f"playback return {label}")
            if ok:
                moved = True
                # 覆盖 _moveit_move 的 "(planning)" 状态 → LED cyan 判定前缀
                self._set_state(STATE_TRAJ, f"playback return {label}")
                r_dur = dur
                if self._n_joints >= 7:
                    # LL-077: move_group 只规划 arm 组（L1–L6），首点夹爪位补下发
                    r_dur = max(dur, self._dispatch_l7_linear(first_q[6]))
            else:
                self.get_logger().warn(
                    f"playback return {label}: {msg} -- join-ramp fallback")
        ramp_in_geo = use_ramp and not moved
        geo: List[List[float]] = []
        geo_times: List[float] = []
        if ramp_in_geo:
            q0 = list(self._positions)
            q1 = recorded_std[0]
            geo = [q0, q1]
            geo_times = [0.0, ramp_s]
            t_base = recorded_times[0]
            geo.extend(recorded_std[1:])
            geo_times.extend(ramp_s + (recorded_times[i] - t_base)
                             for i in range(1, len(recorded_std)))
        else:
            geo = list(recorded_std)
            t_base = recorded_times[0]
            geo_times = [t - t_base for t in recorded_times]

        # F68: Ruckig（加加速度受限）/ TOTG 重新定时 —— 保几何路径，速度加速度由
        # 关节限值算出。失败（节点未起/求解失败）再退回旧的 smooth + time-warp 链。
        traj: Optional[JointTrajectory] = None
        l7_traj: Optional[JointTrajectory] = None
        retime_msg = ""
        if bool(self.get_parameter("playback_retime").value):
            backend = str(self.get_parameter("playback_retime_backend").value)
            min_dur = float(self.get_parameter("playback_retime_min_duration_s").value)
            v_scale = float(self.get_parameter("playback_velocity_scaling").value)
            a_scale = float(self.get_parameter("playback_acceleration_scaling").value)
            target = 0.0
            if bool(self.get_parameter("playback_match_recorded_duration").value):
                # Ruckig 用录制时刻播种（结果时长天然逼近录制）；TOTG 兜底时
                # target_duration 驱动服务端迭代缩放限值。
                # R5/F124: MoveIt 回首点已另耗 r_dur，此处排除 ramp_s 防重定时拖长
                target = max(recorded_duration, ramp_s if ramp_in_geo else 0.0, min_dur)
            # retime 只处理 L1–L6（夹爪无需 Ruckig）；L7 用原始录制时间戳单独下发。
            geo_arm = [p[:6] for p in geo]
            ok, rtraj, msg = self._call_retime(
                geo_arm, JOINTS[:6], backend, v_scale, a_scale,
                geo_times=geo_times, target_duration=target,
            )
            if ok:
                traj = rtraj
                # L7 单独下发/写回仅 7J 有（6J 变体无 L7）
                if self._n_joints >= 7:
                    l7_traj = self._l7_recorded_traj(geo, geo_times)
                retime_msg = msg
                # F133: 重定时成功 → 优化轨迹烘焙回 latest.yaml。仅 latest 槽
                # （空名或显式 'latest'）；ramp 前缀（若走几何 ramp）会污染录制，
                # 故仅当 ramp 由 MoveIt 完成（ramp_in_geo=False）或无 ramp 时写回。
                # 合并写回是 7 关节单时间轴格式，6J 跳过（不影响下发）。
                is_latest_path = os.path.basename(path) == "latest.yaml"
                if (self._n_joints >= 7 and is_latest_path and not ramp_in_geo
                        and not already_retimed
                        and bool(self.get_parameter("playback_retime_writeback").value)):
                    merged = self._merge_retimed_with_l7(rtraj, geo, geo_times)
                    if self._writeback_retimed(path, merged, eps):
                        retime_msg += f"; baked {len(merged.points)} pts"
                elif is_latest_path and already_retimed:
                    retime_msg += "; writeback skipped (already retimed)"
                elif is_latest_path and ramp_in_geo:
                    retime_msg += "; writeback skipped (ramp in geometry)"
            else:
                self.get_logger().warn(
                    f"playback retime failed ({msg}) -- legacy smooth/time-warp fallback"
                )

        if traj is None:
            # 旧链路（F54 默认关闭，仅 playback_retime=false 或服务失败时使用）
            traj = JointTrajectory()
            traj.joint_names = list(file_joint_names)
            for pos, t in zip(recorded_positions, recorded_times):
                pt = JointTrajectoryPoint()
                pt.positions = list(pos)
                pt.time_from_start = _duration(t)
                traj.points.append(pt)
            if ramp_in_geo:
                q1 = [float(v) for v in traj.points[0].positions]
                q0: List[float] = []
                for jn in traj.joint_names:
                    if jn in self._joint_names:
                        q0.append(float(self._positions[self._joint_names.index(jn)]))
                    elif len(q0) < len(q1):
                        q0.append(q1[len(q0)])
                    else:
                        q0.append(0.0)
                q0 = q0[: len(q1)] + q1[len(q0):]
                ramp_pts: List[JointTrajectoryPoint] = []
                p_r0 = JointTrajectoryPoint()
                p_r0.positions = list(q0)
                ramp_pts.append(p_r0)
                p_r1 = JointTrajectoryPoint()
                p_r1.positions = list(q1)
                p_r1.time_from_start = _duration(ramp_s)
                ramp_pts.append(p_r1)
                for pt in traj.points:
                    t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                    pt.time_from_start = _duration(t + ramp_s)
                traj.points = ramp_pts + traj.points

            smooth_s = int(self.get_parameter("playback_smooth_samples").value)
            if smooth_s >= 3 and len(traj.points) >= 4:
                traj.points = _smooth_points(traj.points, smooth_s)
            if bool(self.get_parameter("playback_time_warp").value):
                vmax = float(self.get_parameter("playback_warp_vmax_rad_s").value)
                dt_min = float(self.get_parameter("playback_warp_dt_min_s").value)
                amax = float(self.get_parameter("playback_warp_accel_max_rad_s2").value)
                traj.points = _time_warp_points(traj.points, vmax, dt_min, amax)

        duration = self._traj_duration(traj)
        self._publish_mode("TRAJ_RUNNING")
        ok, dmsg = self._dispatch_trajectory(traj, l7_traj=l7_traj)
        if not ok:
            self._publish_mode("IDLE")
            # R6/F124: phase R（MoveIt 回首点）已把 state 置于 TRAJ，此处失败必须
            # 显式复位 READY，否则 FSM 卡在 TRAJ/灯带滞留
            self._set_state(STATE_READY, f"playback {label}: {dmsg}")
            resp.success = False
            resp.message = dmsg
            return resp
        self._set_state(STATE_TRAJ, f"playback {label}")
        # F129: 轨迹本体时长入占空比账（回首段已在 _moveit_move 内入账），与点到点对齐
        self._duty_record(duration)
        self._schedule_back_to_ready(duration + 0.3)

        suffix = f"retime [{retime_msg}]" if retime_msg else "legacy chain"
        resp.success = True
        resp.message = f"playback {label} ({len(traj.points)} pts, {duration:.1f}s, {suffix})"
        if moved:
            resp.message += f"; return=MoveIt {r_dur:.1f}s"
        return resp

    def _enter_ai_cb(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._state in (STATE_TRAJ, STATE_SERVO, STATE_TEACH, STATE_WAYPOINT_TEACH, STATE_AI):
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
