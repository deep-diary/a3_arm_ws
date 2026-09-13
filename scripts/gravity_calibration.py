#!/usr/bin/env python3
"""F49 7J 重力标定：复刻官方 dynamics_calibration.py，适配本仓真机栈。

官方脚本逻辑（~EDULITE_A3/el_a3_ros/scripts/dynamics_calibration.py）：
  位姿网格（基位 [0, 0.785, -0.785, 0.5, 0.5, 0] 上 L2/L3 单关节扫 + L2xL3 网格 + L4xL5 网格，
  去重 + 碰撞过滤 ~67 点）→ 每点 FJT 移动 + 1.5s 稳定 + 40x20ms 采样均值 → JSONL 增量落盘
  → 12 参数（L2-L6 质量+质心主分量）L-BFGS-B 有界拟合 → 输出 inertia_params.yaml。

本仓适配（与官方差异，均已在代码注释标注）：
  * 7 关节：L7 全程 0（全开），轨迹 7 名（LL-015 单点轨迹坑 → 全部 2 点轨迹）；
  * /joint_states 真机是 BEST_EFFORT（LL-030），双订阅（BE+RELIABLE）兼容仿真；
  * 安全红线：所有移动 ≤ max_step(0.25) rad/步链式分段（3s/段，F41 兜底之上）；
    每点移动前用当前模型预测 τ_g，任一关节 |τ| > torque_skip(3.5) Nm 跳过（F42 RS00 限 5）；
    L3 温度 >75°C 自动回折叠 rest 位降温 <60°C 再续（F44 阈值 95，留裕量）；
  * 数据落盘 ~/.a3/calibration/calibration_data.jsonl（每点 fsync，断点续采）；
  * 拟合按杆名（frame.name → parentJoint）改写 inertia，与 gravity_torque_node 的
    joint_to_link 映射一致（官方按 inertias[2..6] 下标，两 URDF 体序相同所以等价）。

用法：
  python3 scripts/gravity_calibration.py --mode full          # 采集+拟合
  python3 scripts/gravity_calibration.py --optimize-only      # 只用已有数据拟合
  python3 scripts/gravity_calibration.py --dry-run            # 只打印位姿网格与分段数，不动臂
  python3 scripts/gravity_calibration.py --start 30           # 断点续采（0 基）
  python3 scripts/gravity_calibration.py --restart            # 清数据重来

前提：桥 + 编排层在跑、臂已 enable（READY）。中断后重跑自动续采。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import numpy as np

CAL_DATA_DIR = os.path.expanduser("~/.a3/calibration")
CAL_DATA_FILE = os.path.join(CAL_DATA_DIR, "calibration_data.jsonl")

JOINT_NAMES = [
    "L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint", "L7_joint",
]

# 与 gravity_torque_node._apply_calibrated_inertia 的 joint_to_link 完全一致：
# 各关节「下方」的杆（官方 inertias[2..6] 同下标）——τ_i 只依赖第 i 关节以下的杆
FIT_LINK_MAP = {
    "L2": "l2_l3_urdf_asm",
    "L3": "l3_lnik_urdf_asm",
    "L4": "l4_l5_urdf_asm",
    "L5": "part_9",
    "L6": "l5_l6_urdf_asm",
}

# 官方 full 模式 12 参数：L2(mass,com_x) L3(mass,com_x,com_y) L4(mass,com_x,com_y) L5(mass,com_z) L6(mass,com_z)
FIT_INIT = np.array([
    0.8348, 0.095,
    0.1976, -0.056, 0.049,
    0.4606, -0.024, 0.031,
    0.0180, 0.018,
    0.5313, 0.070,
])
FIT_BOUNDS = [
    (0.1, 2.0), (-0.2, 0.2),
    (0.05, 0.5), (-0.15, 0.0), (-0.1, 0.1),
    (0.1, 1.0), (-0.1, 0.0), (0.0, 0.1),
    (0.001, 0.1), (0.0, 0.05),
    (0.1, 1.0), (-0.15, 0.0),
]


@dataclass
class CalibrationConfig:
    urdf_path: str = ""
    output_path: str = ""
    home_position: List[float] = None  # 官方基位（7 关节，L7=0）
    samples_per_config: int = 40
    settle_time: float = 1.5
    sample_interval: float = 0.02
    segment_duration: float = 3.0
    max_step: float = 0.25
    torque_skip: float = 3.5
    rest_pose: List[float] = None  # 降温位（折叠 home）
    temp_guard: bool = True
    torque_filter: bool = True
    temp_pause_c: float = 75.0
    temp_resume_c: float = 60.0

    def __post_init__(self):
        if self.home_position is None:
            self.home_position = [0.0, 0.785, -0.785, 0.5, 0.5, 0.0, 0.0]
        if self.rest_pose is None:
            self.rest_pose = [0.0, 0.0, 0.0, 0.33, 0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# 位姿网格（官方逻辑原样，7 关节）
# ---------------------------------------------------------------------------

def generate_test_configurations(mode: str, home: List[float]) -> List[List[float]]:
    configs = []
    base = np.array(home)

    if mode == "quick":
        l2_range = np.linspace(0.4, 1.4, 6)
        l3_range = np.linspace(-1.2, -0.4, 5)
        l4_range = np.array([0.3, 0.7, 1.0])
        l5_range = np.array([0.3, 0.6])
        grid_step = 2
    elif mode == "high":
        l2_range = np.linspace(0.25, 1.55, 12)
        l3_range = np.linspace(-1.35, -0.25, 12)
        l4_range = np.linspace(0.25, 1.25, 7)
        l5_range = np.linspace(0.25, 1.05, 5)
        grid_step = 2
    elif mode == "ultra":
        l2_range = np.linspace(0.2, 1.6, 15)
        l3_range = np.linspace(-1.4, -0.2, 15)
        l4_range = np.linspace(0.2, 1.3, 8)
        l5_range = np.linspace(0.2, 1.1, 6)
        grid_step = 2
    else:  # full
        l2_range = np.linspace(0.3, 1.5, 10)
        l3_range = np.linspace(-1.3, -0.3, 10)
        l4_range = np.linspace(0.3, 1.2, 6)
        l5_range = np.linspace(0.3, 1.0, 5)
        grid_step = 2

    for l2 in l2_range:
        cfg = base.copy(); cfg[1] = l2
        configs.append(cfg.tolist())
    for l3 in l3_range:
        cfg = base.copy(); cfg[2] = l3
        configs.append(cfg.tolist())
    l2_grid = l2_range[::grid_step]
    l3_grid = l3_range[::grid_step]
    for l2 in l2_grid:
        for l3 in l3_grid:
            cfg = base.copy(); cfg[1] = l2; cfg[2] = l3
            configs.append(cfg.tolist())
    for l4 in l4_range:
        for l5 in l5_range:
            cfg = base.copy(); cfg[3] = l4; cfg[4] = l5
            configs.append(cfg.tolist())

    unique = []
    for cfg in configs:
        if not any(np.allclose(cfg, u, atol=0.05) for u in unique):
            unique.append(cfg)

    safe = [c for c in unique if not check_collision_with_base(c)]
    return safe


def check_collision_with_base(config: List[float]) -> bool:
    """官方碰撞判据（同臂同结构）。"""
    l2, l3 = config[1], config[2]
    if l2 >= 1.2 and l3 > -0.4:
        return True
    if l2 > 1.35 and l3 > -0.6:
        return True
    if l2 >= 1.2 and l3 >= -0.55 and (l2 + l3 * 0.5) > 0.9:
        return True
    return False


# ---------------------------------------------------------------------------
# JSONL 增量数据
# ---------------------------------------------------------------------------

def write_meta_line(path: str, mode: str, total_points: int, home: list):
    with open(path, "w") as f:
        f.write(json.dumps({
            "meta": True, "mode": mode, "total_points": total_points,
            "home": home, "start_time": datetime.now().isoformat(),
        }) + "\n")


def append_data_line(path: str, record: dict):
    with open(path, "a") as f:
        f.write(json.dumps(record) + "\n")
        f.flush()
        os.fsync(f.fileno())


def read_jsonl(path: str) -> Tuple[Optional[dict], List[dict]]:
    meta, records = None, []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if obj.get("meta"):
                meta = obj
            else:
                records.append(obj)
    return meta, records


def dedup_records(records: List[dict]) -> List[dict]:
    by_idx = {r["idx"]: r for r in records}
    return [by_idx[k] for k in sorted(by_idx.keys())]


# ---------------------------------------------------------------------------
# 拟合（模块级函数，可被冒烟脚本 import）
# ---------------------------------------------------------------------------

def _fit_jids(model):
    """返回 {L2..L6 → parentJoint} 的 jid（与 gravity_torque_node 同映射）。"""
    jids = {}
    for key, link_name in FIT_LINK_MAP.items():
        jid = None
        for frame in model.frames:
            if frame.name == link_name:
                jid = frame.parentJoint
                break
        jids[key] = jid
    return jids


def _param_to_inertia_overrides(model, params: np.ndarray, jids: dict) -> List[Tuple]:
    """12 参数 → 5 根杆的 (jid, mass, lever3)。未拟合分量保持 URDF 原值（官方同）。

    参数序（官方同）：L2(m,cx) L3(m,cx,cy) L4(m,cx,cy) L5(m,cz) L6(m,cz)
    """
    overrides = []
    for key, (jid, mass, fitted) in [
        ("L2", (jids["L2"], params[0], {"x": params[1]})),
        ("L3", (jids["L3"], params[2], {"x": params[3], "y": params[4]})),
        ("L4", (jids["L4"], params[5], {"x": params[6], "y": params[7]})),
        ("L5", (jids["L5"], params[8], {"z": params[9]})),
        ("L6", (jids["L6"], params[10], {"z": params[11]})),
    ]:
        lever = np.array(model.inertias[jid].lever, dtype=float).copy()
        if "x" in fitted:
            lever[0] = fitted["x"]
        if "y" in fitted:
            lever[1] = fitted["y"]
        if "z" in fitted:
            lever[2] = fitted["z"]
        overrides.append((jid, float(mass), lever))
    return overrides


def predict_gravity(model, data, q7: np.ndarray, params: Optional[np.ndarray] = None) -> np.ndarray:
    """rnea 静态重力：返回 7 维力矩。params 非空时原地改写惯性算完还原。"""
    import pinocchio as pin  # 延迟导入，供无 ROS 环境冒烟

    saved = None
    if params is not None:
        jids = _fit_jids(model)
        # 先读原值再原地改写（官方同款模式：mass/lever 字段直接赋值，gravity_torque_node 已验证可行）
        saved = [(jid, model.inertias[jid].mass, model.inertias[jid].lever.copy())
                 for jid in jids.values()]
        for jid, mass, lever in _param_to_inertia_overrides(model, params, jids):
            model.inertias[jid].mass = mass
            model.inertias[jid].lever = lever

    q = np.zeros(model.nq)
    q[: len(q7)] = q7
    try:
        tau = pin.rnea(model, data, q, np.zeros(model.nv), np.zeros(model.nv))
    finally:
        if saved is not None:
            for jid, mass, lever in saved:
                model.inertias[jid].mass = mass
                model.inertias[jid].lever = lever
    return np.asarray(tau)[:7]


def fit_inertia(model, records, fix_masses: bool = False) -> Tuple[dict, float, float]:
    """L-BFGS-B 拟合。records: [(pos7, eff7), ...]。返回 (results_dict, rmse, r_squared)。

    fix_masses=False：官方 12 参数（L2-L6 质量+质心主分量）。
    fix_masses=True：官方 8 参数 COM-only——质量钉在官方实测值表，只拟质心 + L5 质量。
    """
    from scipy.optimize import minimize

    data = model.createData()
    jids = _fit_jids(model)
    for key, jid in jids.items():
        if jid is None or jid <= 0 or jid >= len(model.inertias):
            raise RuntimeError(f"link for {key} not found in URDF (frame map mismatch)")
        print(f"fit link {key}: jid={jid} mass={model.inertias[jid].mass:.4f}")

    def objective(params):
        total = 0.0
        for positions, measured in records:
            predicted = predict_gravity(model, data, positions, params)
            for i in range(1, 6):  # L2..L6
                total += (measured[i] - predicted[i]) ** 2
        return total

    if fix_masses:
        fm = {"L2": 0.877, "L3": 0.251, "L4": 0.556, "L6": 0.668}  # 官方实测值表
        print(f"fix_masses: L2={fm['L2']} L3={fm['L3']} L4={fm['L4']} L6={fm['L6']}")
        initial = np.array([
            0.095,
            -0.056, 0.049,
            -0.024, 0.031,
            0.0180, 0.018,
            -0.070,
        ])
        bounds = [
            (-0.2, 0.2),
            (-0.15, 0.0), (-0.1, 0.1),
            (-0.1, 0.0), (0.0, 0.1),
            (0.001, 0.1), (0.0, 0.05),
            (-0.15, 0.0),
        ]

        def _expand(free: np.ndarray) -> np.ndarray:
            return np.array([
                fm["L2"], free[0],
                fm["L3"], free[1], free[2],
                fm["L4"], free[3], free[4],
                free[5], free[6],
                fm["L6"], free[7],
            ])

        result = minimize(lambda free: objective(_expand(free)), initial,
                          method="L-BFGS-B", bounds=bounds,
                          options={"maxiter": 500, "disp": False})
        opt = _expand(result.x)
        fixed_tags = {"L2", "L3", "L4", "L6"}
    else:
        result = minimize(objective, FIT_INIT, method="L-BFGS-B", bounds=FIT_BOUNDS,
                          options={"maxiter": 500, "disp": False})
        opt = result.x
        fixed_tags = set()

    final_error = result.fun
    n_samples = len(records) * 5
    rmse = float(np.sqrt(final_error / n_samples)) if n_samples else 0.0

    all_measured = []
    for _, efforts in records:
        all_measured.extend(efforts[1:6])
    mean_m = float(np.mean(all_measured)) if all_measured else 0.0
    ss_tot = sum((m - mean_m) ** 2 for m in all_measured)
    r_squared = 1 - final_error / ss_tot if ss_tot > 0 else 0.0

    # 未拟合分量取 URDF 原值（官方同：results 里也写死原值，但为换 URDF 安全从模型读）
    levers = {key: np.array(model.inertias[jid].lever, dtype=float)
              for key, jid in jids.items()}
    results = {
        "L2": {"mass": float(opt[0]), "com": [float(opt[1]), float(levers["L2"][1]), float(levers["L2"][2])]},
        "L3": {"mass": float(opt[2]), "com": [float(opt[3]), float(opt[4]), float(levers["L3"][2])]},
        "L4": {"mass": float(opt[5]), "com": [float(opt[6]), float(opt[7]), float(levers["L4"][2])]},
        "L5": {"mass": float(opt[8]), "com": [float(levers["L5"][0]), float(levers["L5"][1]), float(opt[9])]},
        "L6": {"mass": float(opt[10]), "com": [float(levers["L6"][0]), float(levers["L6"][1]), float(opt[11])]},
        "calibration_info": {
            "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "num_samples": len(records),
            "rmse": float(rmse),
            "r_squared": float(r_squared),
        },
    }
    for key, tag in [("L2", "L2" in fixed_tags), ("L3", "L3" in fixed_tags),
                     ("L4", "L4" in fixed_tags), ("L6", "L6" in fixed_tags)]:
        if tag:
            results[key]["_fixed_mass"] = True
    return results, rmse, r_squared


def save_results(results: dict, output_path: str) -> None:
    info = results["calibration_info"]
    content = (
        "# EL-A3 arm inertia parameters\n"
        "# For Pinocchio gravity compensation\n"
        "# Fitted by scripts/gravity_calibration.py (F49, 7J with gripper) — override URDF defaults\n"
        "#\n"
        f"# Calibration date: {info['date']}\n"
        f"# RMSE: {info['rmse']:.4f} Nm\n"
        f"# R^2: {info['r_squared']:.4f}\n"
        "\n"
        "use_calibrated_params: true\n"
        "\n"
        "inertia_params:\n"
        f"  L2:\n    mass: {results['L2']['mass']:.4f}\n    com: {results['L2']['com']}\n"
        f"\n  L3:\n    mass: {results['L3']['mass']:.4f}\n    com: {results['L3']['com']}\n"
        f"\n  L4:\n    mass: {results['L4']['mass']:.4f}\n    com: {results['L4']['com']}\n"
        f"\n  L5:\n    mass: {results['L5']['mass']:.4f}\n    com: {results['L5']['com']}\n"
        f"\n  L6:\n    mass: {results['L6']['mass']:.4f}\n    com: {results['L6']['com']}\n"
        "\ncalibration_info:\n"
        f"  date: \"{info['date']}\"\n"
        f"  num_samples: {info['num_samples']}\n"
        f"  rmse: {info['rmse']:.4f}\n"
        f"  r_squared: {info['r_squared']:.4f}\n"
    )
    if os.path.isfile(output_path):
        bak = f"{output_path}.bak-{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.rename(output_path, bak)
        print(f"backup old params → {bak}")
    with open(output_path, "w") as f:
        f.write(content)
    print(f"results saved → {output_path}")


# ---------------------------------------------------------------------------
# ROS 采集节点
# ---------------------------------------------------------------------------

class GravityCalibrator:
    """不继承 Node 的纯逻辑容器；ROS 部分由 run() 内建节点驱动。"""

    def __init__(self, config: CalibrationConfig, args):
        self.config = config
        self.args = args
        self.cur_pos = None  # 7
        self.cur_eff = None
        self.temps = None  # 7
        self.arm_state = None
        self.last_js_stamp = None
        self.node = None
        self.model = None
        self.data = None
        self.fjt = None
        self.records = []

    # ---- ROS 初始化 ----

    def init_ros(self) -> bool:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        from sensor_msgs.msg import JointState
        from a3_msgs.msg import ArmStatus

        rclpy.init()
        self._rclpy = rclpy
        self.node = Node("a3_gravity_calibration")
        self.clock = self.node.get_clock()

        def on_js(msg):
            q = {n: p for n, p in zip(msg.name, msg.position)}
            e = {n: p for n, p in zip(msg.name, msg.effort)}
            if all(n in q for n in JOINT_NAMES):
                self.cur_pos = np.array([q[n] for n in JOINT_NAMES])
                self.cur_eff = np.array([e[n] for n in JOINT_NAMES])
                self.last_js_stamp = msg.header.stamp

        # LL-030：真机 js 是 BEST_EFFORT，仿真 RELIABLE——双订阅兼容两端
        self.node.create_subscription(JointState, "/joint_states", on_js,
                                      QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(JointState, "/joint_states", on_js,
                                      QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        def on_status(msg: ArmStatus):
            self.arm_state = msg.state
            if len(msg.temperatures) == 7:
                self.temps = np.array(msg.temperatures)

        self.node.create_subscription(ArmStatus, "/a3/arm_status", on_status, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory")
        return True

    def init_pinocchio(self) -> bool:
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(self.config.urdf_path)
        self.data = self.model.createData()
        if self.model.nq != 7:
            print(f"WARN: URDF nq={self.model.nq} != 7 — 期望 7 关节模型")
        return True

    # ---- 基础工具 ----

    def spin_until(self, cond, timeout: float, interval: float = 0.05) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.02)
            if cond():
                return True
            time.sleep(interval)
        return False

    def wait_js(self, timeout: float = 10.0) -> bool:
        return self.spin_until(lambda: self.cur_pos is not None, timeout)

    def wait_ready(self, timeout: float = 10.0) -> bool:
        return self.spin_until(lambda: self.arm_state == "READY", timeout)

    def max_temp(self) -> float:
        return float(np.max(self.temps)) if self.temps is not None else 0.0

    # ---- 运动：链式分段 FJT ----

    def move_to_position(self, target: np.ndarray) -> bool:
        """2 点轨迹单段移动（当前 → target），等待动作收敛结果。"""
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        if not self.fjt.wait_for_server(timeout_sec=5.0):
            print("  FAIL: FJT action server unavailable")
            return False
        dur = self.config.segment_duration
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        p0 = JointTrajectoryPoint()
        p0.positions = [float(x) for x in self.cur_pos]
        p0.time_from_start = Duration(sec=0, nanosec=0)
        p1 = JointTrajectoryPoint()
        p1.positions = [float(x) for x in target]
        p1.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
        traj.points = [p0, p1]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        fut = self.fjt.send_goal_async(goal)
        if not self.spin_until(fut.done, 5.0):
            print("  FAIL: goal ack timeout")
            return False
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("  FAIL: goal rejected")
            return False
        res_fut = gh.get_result_async()
        ok = self.spin_until(res_fut.done, dur + 8.0)
        if not ok:
            print("  FAIL: move result timeout (arm may still be moving)")
            return False
        res = res_fut.result()
        if res.result.error_code != 0:
            print(f"  WARN: move error_code={res.result.error_code} ({res.result.error_string})", flush=True)
            # -5 timeout 可能是 FJT 收敛判定的偶发误报（回调组被饿时 _joint_pos 停在移动起点），
            # 臂（C++ 执行层独立插值）往往已到位：等 js 到位核实，到位视为成功继续采样
            for _ in range(40):  # ≤4 s
                self._rclpy.spin_once(self.node, timeout_sec=0.05)
                if (self.cur_pos is not None
                        and np.max(np.abs(self.cur_pos - np.array(target))) <= 0.05):
                    print("    但 js 已到位（误差 ≤0.05），视为成功", flush=True)
                    return True
                time.sleep(0.05)
            return False
        return True

    def move_chained(self, target: np.ndarray) -> bool:
        """≤ max_step rad/关节的链式分段移动；每段 2 点 FJT。"""
        steps = 0
        while True:
            delta = target - self.cur_pos
            maxd = float(np.max(np.abs(delta)))
            if maxd <= 0.02:
                return True
            if steps >= 40:
                print("  FAIL: chain too long (>40 segments), abort")
                return False
            scale = min(1.0, self.config.max_step / maxd)
            wp = self.cur_pos + delta * scale
            if not self.move_to_position(wp):
                return False
            steps += 1
            # 轨迹结束后等 js 跟上（收敛判定 0.05 rad 已由 FJT 保证）
            self.spin_until(lambda: self.cur_pos is not None, 1.0, interval=0.1)

    # ---- 温度守护 ----

    def cool_down_if_needed(self) -> bool:
        if not self.config.temp_guard:
            return True
        if self.max_temp() <= self.config.temp_pause_c:
            return True
        print(f"  [temp guard] max temp {self.max_temp():.1f}°C > "
              f"{self.config.temp_pause_c} — 回折叠位降温")
        if not self.move_chained(np.array(self.config.rest_pose)):
            return False
        print(f"  [temp guard] 等待降至 {self.config.temp_resume_c}°C ...")
        while True:
            self._rclpy.spin_once(self.node, timeout_sec=1.0)
            if self.arm_state not in ("READY", "TRAJ"):
                print(f"  [temp guard] 状态异常 {self.arm_state}，中止")
                return False
            if self.max_temp() <= self.config.temp_resume_c:
                print(f"  [temp guard] 已降至 {self.max_temp():.1f}°C，继续采集")
                return True
            print(f"  [temp guard] {self.max_temp():.1f}°C ...", flush=True)

    # ---- 采样 ----

    def collect_samples(self) -> Tuple[np.ndarray, np.ndarray]:
        positions, efforts = [], []
        for _ in range(self.config.samples_per_config):
            self._rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.cur_pos is not None:
                positions.append(self.cur_pos.copy())
                efforts.append(self.cur_eff.copy())
            time.sleep(self.config.sample_interval)
        return np.mean(positions, axis=0), np.mean(efforts, axis=0)

    # ---- 采集主循环 ----

    def run_collection(self) -> bool:
        mode = self.args.mode
        test_configs = generate_test_configurations(mode, self.config.home_position)

        # 锚点位姿：官方网格不含折叠位（L2/L3 近 0），但验收要 home/ready 的 |Δτ|≤0.2 Nm，
        # 把 rest（折叠 home）与 --anchors 指定位姿追加进网格做外推锚定（不进碰撞过滤，用户自定）
        anchors = [self.config.rest_pose]
        for raw in (self.args.anchors or []):
            try:
                anchors.append([float(x) for x in raw.split(",")])
            except ValueError:
                print(f"  WARN: 无法解析 anchor '{raw}'（需 7 个逗号分隔浮点），跳过")
        for a in anchors:
            if len(a) == 7 and not any(np.allclose(a, c, atol=0.05) for c in test_configs):
                test_configs.append(a)
        if len(anchors) > 1:
            print(f"锚点位姿追加：rest + {len(anchors) - 1} 个 --anchors")

        # 力矩预测过滤（当前模型，可能低估——F42/F44 仍是最后防线）
        skipped = 0
        if self.config.torque_filter:
            filtered = []
            for cfg in test_configs:
                q = np.zeros(self.model.nq)
                q[:7] = cfg
                tau = self.pin.computeGeneralizedGravity(self.model, self.data, q)
                if np.max(np.abs(tau[1:6])) > self.config.torque_skip:
                    print(f"  [skip] 预测 τ 超 {self.config.torque_skip} Nm: {cfg}")
                    skipped += 1
                    continue
                filtered.append(cfg)
            test_configs = filtered
            print(f"τ 预测过滤：{skipped} 点跳过，剩 {len(test_configs)} 点")

        if self.args.dry_run:
            total_segs = 0
            for i, cfg in enumerate(test_configs):
                n = int(np.ceil(np.max(np.abs(np.array(cfg) - np.array(
                    test_configs[i - 1] if i else self.config.home_position))) / self.config.max_step))
                total_segs += max(n, 1)
            print(f"--dry-run: {len(test_configs)} configs, 预估 {total_segs} 个分段 "
                  f"(×{self.config.segment_duration}s ≈ {total_segs * self.config.segment_duration / 60:.1f} min 纯移动)")
            return True

        total = len(test_configs)
        os.makedirs(os.path.dirname(os.path.abspath(self.args.data_file)) or ".", exist_ok=True)
        df = self.args.data_file

        if self.args.restart and os.path.exists(df):
            print("--restart: removing existing data file")
            os.unlink(df)

        if self.args.start is not None:
            completed = self.args.start
            if not os.path.exists(df):
                write_meta_line(df, mode, total, self.config.home_position)
            else:
                meta, _ = read_jsonl(df)
                if meta and meta.get("mode") != mode:
                    print(f'Mode mismatch: file has "{meta.get("mode")}", requested "{mode}". Use --restart.')
                    return False
        elif os.path.exists(df):
            meta, records = read_jsonl(df)
            if meta is None:
                print("Data file corrupt (no meta line). Use --restart.")
                return False
            if meta.get("mode") != mode:
                print(f'Mode mismatch: file has "{meta.get("mode")}", requested "{mode}". Use --restart.')
                return False
            completed = len(dedup_records(records))
            print(f"Resuming: {completed}/{total} points already collected")
        else:
            completed = 0
            write_meta_line(df, mode, total, self.config.home_position)
            print(f"Starting fresh collection: {total} points")

        if completed >= total:
            print("All points already collected, skipping to optimization")
            return True

        for i in range(completed, total):
            cfg = test_configs[i]
            print(f"[{i + 1}/{total}] {cfg[1]:.2f},{cfg[2]:.2f},{cfg[3]:.2f},{cfg[4]:.2f} "
                  f"(L3 {self.max_temp():.0f}°C)", flush=True)

            if not self.cool_down_if_needed():
                return False
            if self.arm_state not in ("READY", "TRAJ"):
                print(f"  状态异常 {self.arm_state} — 中止采集（臂可能被外部失能）")
                return False

            if not self.move_chained(np.array(cfg)):
                print("  移动失败，跳过该点")
                continue

            time.sleep(self.config.settle_time)
            pos, eff = self.collect_samples()
            if self.cur_pos is None:
                print("  js 断流，中止")
                return False

            append_data_line(df, {
                "idx": i,
                "config": cfg,
                "position": pos.tolist(),
                "effort": eff.tolist(),
                "temps": self.temps.tolist() if self.temps is not None else [],
                "timestamp": datetime.now().isoformat(),
            })
            print(f"  τ=[{eff[1]:.3f},{eff[2]:.3f},{eff[3]:.3f},{eff[4]:.3f},{eff[5]:.3f}] saved")

        print("返回折叠 rest 位 ...")
        self.move_chained(np.array(self.config.rest_pose))
        return True

    # ---- 拟合入口 ----

    def run_optimize(self) -> bool:
        df = self.args.data_file
        if not os.path.exists(df):
            print(f"No data file found at {df}")
            return False
        _, records = read_jsonl(df)
        records = dedup_records(records)
        if len(records) < 10:
            print(f"Too few data points ({len(records)}), need at least 10")
            return False
        self.records = [(np.array(r["position"]), np.array(r["effort"])) for r in records]
        print(f"Loaded {len(records)} data points")

        results, rmse, r_squared = fit_inertia(self.model, self.records,
                                               fix_masses=self.args.fix_masses)
        print(f"RMSE: {rmse:.4f} Nm  R^2: {r_squared:.4f}")
        for j in ["L2", "L3", "L4", "L5", "L6"]:
            p = results[j]
            print(f"  {j}: mass={p['mass']:.4f}, com={[round(x, 4) for x in p['com']]}")
        save_results(results, self.args.output)
        return True


def main():
    parser = argparse.ArgumentParser(description="F49 7J gravity calibration (官方复刻)")
    parser.add_argument("--mode", default="full", choices=["quick", "full", "high", "ultra"])
    parser.add_argument("--start", type=int, default=None, help="断点续采起始下标")
    parser.add_argument("--restart", action="store_true", help="清数据重来")
    parser.add_argument("--optimize-only", action="store_true", help="只拟合不采集")
    parser.add_argument("--fix-masses", action="store_true", help="固定质量只拟合质心")
    parser.add_argument("--dry-run", action="store_true", help="只打印网格不动臂")
    parser.add_argument("--samples", type=int, default=40)
    parser.add_argument("--data-file", default=CAL_DATA_FILE,
                        help=f"采集数据文件（默认 {CAL_DATA_FILE}；仿真冒烟请指向 /tmp）")
    parser.add_argument("--output", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "src", "a3_description", "config", "inertia_params.yaml"))
    parser.add_argument("--urdf", default=None)
    parser.add_argument("--max-step", type=float, default=0.25)
    parser.add_argument("--segment-s", type=float, default=3.0)
    parser.add_argument("--torque-skip", type=float, default=3.5)
    parser.add_argument("--anchors", action="append", default=None,
                        help="追加锚点位姿（可多次），7 个逗号分隔浮点，"
                             "如 0.07,1.08,-0.56,-0.46,0.08,-0.05,0（rest 折叠位默认追加）")
    parser.add_argument("--no-temp-guard", action="store_true")
    parser.add_argument("--no-torque-filter", action="store_true")
    args = parser.parse_args()

    if args.urdf is None:
        from ament_index_python.packages import get_package_share_directory
        args.urdf = os.path.join(
            get_package_share_directory("a3_description"), "urdf", "el_a3.urdf")
    print(f"urdf: {args.urdf}")

    config = CalibrationConfig(
        urdf_path=args.urdf,
        output_path=args.output,
        samples_per_config=args.samples,
        max_step=args.max_step,
        segment_duration=args.segment_s,
        torque_skip=args.torque_skip,
        temp_guard=not args.no_temp_guard,
        torque_filter=not args.no_torque_filter,
    )

    cal = GravityCalibrator(config, args)
    if not cal.init_pinocchio():
        print("Pinocchio init failed")
        return 1

    if args.optimize_only:
        ok = cal.run_optimize()
        return 0 if ok else 1

    if not cal.init_ros():
        print("ROS init failed")
        return 1

    try:
        if args.dry_run:
            cal.run_collection()
            return 0
        print("等待 /joint_states ...")
        if not cal.wait_js(10.0):
            print("FAIL: /joint_states 无数据（桥没起？）")
            return 1
        print(f"当前位姿 {[round(float(x), 3) for x in cal.cur_pos]}，等待 READY ...")
        if not cal.wait_ready(10.0):
            print(f"FAIL: 臂状态 {cal.arm_state} ≠ READY（先 enable）")
            return 1
        print("开始采集（Ctrl+C 中断可续采）")
        if not cal.run_collection():
            print("\n采集中止。已采数据保留，重跑续采。")
            return 1
        if not cal.run_optimize():
            return 1
        print("\n" + "=" * 60)
        print("  标定完成！重启重力节点（a3_gravity_torque）生效")
        print("=" * 60)
        return 0
    except KeyboardInterrupt:
        print(f"\nInterrupted. 数据已逐点保存在 {args.data_file}，重跑续采。")
        return 130
    finally:
        if cal.node is not None:
            cal.node.destroy_node()
            cal._rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
