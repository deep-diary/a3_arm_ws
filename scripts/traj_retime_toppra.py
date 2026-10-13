#!/usr/bin/env python3
"""F167.2 toppra + Pinocchio 离线时间参数化：速度/加速度 + 力矩约束 + 自定义凸约束。

给一条几何关节路径（waypoints 的 L1-L6），用 toppra（TOPP-RA）求时间最优重定时，
约束含：
  * JointVelocityConstraint / JointAccelerationConstraint（运动学）；
  * **JointTorqueConstraint**（Pinocchio `rnea` 逆动力学，含重力+科氏+惯性，标定惯量）；
  * **自定义凸约束** WeightedAccelConstraint（关节加速度线性组合 |W·q̈| ≤ c，示例：腕部三轴之和）。
输出 (t,q,v,a) 指标 + 写 baked 轨迹（可被编排层 `_play_baked` 直接回放下发）。

前置（先 source a3_shell_env.sh）：
  * toppra（本机 pip 装：`sudo pip3 install --no-deps --no-build-isolation toppra`，需先 Cython）
  * toppra 顶层 import matplotlib（本机 matplotlib/pyparsing 冲突，LL-152）→ 本脚本内置空壳 stub。
用法：
  python3 scripts/traj_retime_toppra.py --waypoints trajectories/waypoints/latest.yaml \
      --out trajectories/streams/toppra_5pt.baked.yaml --report docs/dev/F167_toppra.md
退出码：0=求解成功且力矩在限内，1=失败/越限。
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional, Tuple

import numpy as np
import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))


def _import_toppra():
    """toppra 顶层 import matplotlib（仅绘图）；本机 matplotlib 坏（LL-152）→ 空壳 stub。"""
    import types
    if "matplotlib" not in sys.modules:
        mpl = types.ModuleType("matplotlib")
        plt = types.ModuleType("matplotlib.pyplot")
        mpl.pyplot = plt
        sys.modules["matplotlib"] = mpl
        sys.modules["matplotlib.pyplot"] = plt
    import toppra as ta
    import toppra.constraint as constraint
    import toppra.algorithm as algo
    from toppra.constraint import LinearConstraint, DiscretizationType
    return ta, constraint, algo, LinearConstraint, DiscretizationType


class WeightedAccelConstraint:
    """工厂：|W·q̈| ≤ c 的自定义凸（多面体）约束。

    toppra 的 LinearConstraint 契约：compute_constraint_params(path, gridpoints) 返回
    (a, b, c, F, g, ubound, xbound)，其中 q̈ = q'(s)·u + q''(s)·x（u=路径加速度、x=路径速度²）。
    这里 a = q'·Wᵀ、b = q''·Wᵀ，F=[I;-I]、g=[hi;-lo]。（在函数内构造以继承 import 到的基类）
    """


def _env_caps(dof: int, vmax: float, amax: float) -> Tuple[np.ndarray, np.ndarray]:
    vlim = np.array([[-vmax, vmax]] * dof, dtype=float)
    alim = np.array([[-amax, amax]] * dof, dtype=float)
    return vlim, alim


def load_pin_model(with_calibrated: bool = True):
    """Pinocchio 模型 + L1-L6 关节索引；可选套用标定惯量（inertia_params.yaml）。"""
    import pinocchio as pin
    from ament_index_python.packages import get_package_share_directory
    urdf = os.path.join(get_package_share_directory("a3_description"), "urdf", "el_a3.urdf")
    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()
    if with_calibrated:
        cfg = os.path.join(get_package_share_directory("a3_description"), "config", "inertia_params.yaml")
        try:
            with open(cfg, "r", encoding="utf-8") as f:
                d = yaml.safe_load(f) or {}
            if d.get("use_calibrated_params") and d.get("inertia_params"):
                for name, p in d["inertia_params"].items():
                    jid = model.getJointId(name + "_joint")
                    if jid <= 0:
                        continue
                    old = model.inertias[jid]
                    model.inertias[jid] = pin.Inertia(
                        float(p["mass"]), np.array(p["com"], dtype=float), old.inertia)
        except Exception as exc:  # noqa: BLE001
            print(f"[warn] 标定惯量套用失败，用 URDF 默认: {exc}", file=sys.stderr)
    return pin, model, data


def main() -> int:
    ap = argparse.ArgumentParser(description="F167.2 toppra 力矩/凸约束离线重定时")
    ap.add_argument("--waypoints", default=os.path.join(WS, "trajectories", "waypoints", "latest.yaml"))
    ap.add_argument("--out", default=os.path.join(WS, "trajectories", "streams", "toppra_5pt.baked.yaml"))
    ap.add_argument("--report", default=os.path.join(WS, "docs", "dev", "F167_toppra.md"))
    ap.add_argument("--vmax", type=float, default=1.5, help="关节速度上限 rad/s（同 goto_via）")
    ap.add_argument("--amax", type=float, default=15.0, help="关节加速度上限 rad/s²（同 goto_via）")
    ap.add_argument("--tau-lim", default="7,7,7,3,3,3", help="L1-L6 力矩上限 Nm")
    ap.add_argument("--wrist-sum-frac", type=float, default=0.8,
                    help="自定义凸约束：腕部(L4-L6)加速度绝对值之和 ≤ frac·3·amax")
    ap.add_argument("--samples", type=int, default=240)
    args = ap.parse_args()

    ta, constraint, algo, LinearConstraint, DiscretizationType = _import_toppra()
    pin, model, data = load_pin_model(with_calibrated=True)

    JOINTS = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]
    q_idx = [int(model.idx_qs[model.getJointId(jn)]) for jn in JOINTS]
    v_idx = [int(model.idx_vs[model.getJointId(jn)]) for jn in JOINTS]

    def _expand(vec6: np.ndarray) -> np.ndarray:
        full = np.zeros(model.nv)
        for val, vi in zip(vec6, v_idx):
            full[vi] = float(val)
        return full

    def inv_dyn(q6, qd6, qdd6):
        q = np.zeros(model.nq)
        for val, qi in zip(q6, q_idx):
            q[qi] = float(val)
        tau = pin.rnea(model, data, q, _expand(qd6), _expand(qdd6))
        return np.array([tau[vi] for vi in v_idx], dtype=float)

    with open(args.waypoints, "r", encoding="utf-8") as f:
        wp = yaml.safe_load(f)
    wps = wp["waypoints"]
    qs = np.array([[float(v) for v in w["positions"][:6]] for w in wps], dtype=float)
    n = qs.shape[0]
    if n < 2:
        print("[fail] waypoints < 2", file=sys.stderr)
        return 1

    # 弦长参数化 → SplineInterpolator
    seg = np.linalg.norm(np.diff(qs, axis=0), axis=1)
    ss = np.concatenate([[0.0], np.cumsum(seg)])
    ss = ss / ss[-1]
    path = ta.SplineInterpolator(ss, qs)

    dof = 6
    vlim, alim = _env_caps(dof, args.vmax, args.amax)
    tau_lim = np.array([[-t, t] for t in [float(x) for x in args.tau_lim.split(",")]], dtype=float)
    fs_coef = np.zeros(dof)

    pc_vel = constraint.JointVelocityConstraint(vlim)
    pc_acc = constraint.JointAccelerationConstraint(alim, discretization_scheme=DiscretizationType.Interpolation)
    pc_tau = constraint.JointTorqueConstraint(inv_dyn, tau_lim, fs_coef,
                                              discretization_scheme=DiscretizationType.Collocation)

    # 自定义凸约束：|W·q̈| ≤ c，W=腕部三轴求和
    W = np.zeros((1, dof))
    W[0, 3:] = 1.0
    c_half = args.wrist_sum_frac * 3.0 * args.amax * 0.5  # 半带（上下对称）

    class _WeightedAccel(LinearConstraint):
        def __init__(self):
            super().__init__()
            self.dof = dof
            self.identical = True

        def compute_constraint_params(self, path, gridpoints, *a, **k):
            ps = np.asarray(path(gridpoints, order=1)).reshape((-1, dof)) @ W.T
            pss = np.asarray(path(gridpoints, order=2)).reshape((-1, dof)) @ W.T
            k_ = W.shape[0]
            F = np.vstack([np.eye(k_), -np.eye(k_)])
            g = np.concatenate([np.full(k_, c_half), np.full(k_, c_half)])
            return ps, pss, np.zeros_like(ps), F, g, None, None

    pc_custom = _WeightedAccel()

    algorithm = algo.TOPPRA([pc_vel, pc_acc, pc_tau, pc_custom], path,
                            parametrizer="ParametrizeConstAccel")
    traj = algorithm.compute_trajectory(0.0, 0.0)
    if traj is None:
        print("[fail] toppra 求解失败（约束不可行）", file=sys.stderr)
        return 1

    t0, t1 = float(traj.path_interval[0]), float(traj.path_interval[1])
    ts = np.linspace(t0, t1, args.samples)
    q_t = np.asarray(traj(ts))
    v_t = np.asarray(traj(ts, 1))
    a_t = np.asarray(traj(ts, 2))
    tau_t = np.array([inv_dyn(q_t[i], v_t[i], a_t[i]) for i in range(len(ts))])

    dur = float(ts[-1] - ts[0])
    a_rms = float(np.sqrt(np.mean(a_t ** 2)))
    j = np.gradient(a_t, ts, axis=0)
    j_rms = float(np.sqrt(np.mean(j ** 2)))
    tau_peak = np.max(np.abs(tau_t), axis=0)
    tau_max = tau_lim[:, 1]
    ok_tau = bool(np.all(tau_peak <= tau_max * 1.001))

    # 写 baked（编排层 _play_baked 直接下发）
    out_path = os.path.expanduser(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    baked = {"kind": "continuous", "retimed": True,
             "joint_names": JOINTS + ["L7_joint"],
             "points": [{"positions": ([float(x) for x in q_t[i]] + [0.0]),
                         "time_from_start_sec": float(ts[i] - ts[0])}
                        for i in range(len(ts))]}
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(baked, f, sort_keys=False)

    lines = ["# F167.2 toppra 离线重定时（力矩 + 自定义凸约束）", "",
             f"- 路径：`{os.path.basename(args.waypoints)}`（{n} 点，L1-L6）；轨迹 {len(ts)} 采样。",
             f"- 约束：速度 ≤ {args.vmax} rad/s、加速度 ≤ {args.amax} rad/s²、力矩 ≤ "
             f"{[float(x) for x in args.tau_lim.split(',')]} Nm、"
             f"自定义 |腕部(4-6)加速度和| ≤ {c_half:.1f} rad/s²。",
             f"- Pinocchio 逆动力学（rnea，标定惯量）复算力矩。", "",
             f"| 指标 | 值 |", "|---|---|",
             f"| 时长 dur | {dur:.2f} s |",
             f"| a_rms | {a_rms:.2f} rad/s² |",
             f"| j_rms | {j_rms:.1f} rad/s³ |",
             f"| 峰值 |τ| (L1-L6) | {np.round(tau_peak,2).tolist()} Nm |",
             f"| 力矩上限 | {tau_max.tolist()} Nm |",
             f"| 力矩在限内 | {'PASS ✓' if ok_tau else 'FAIL ✗'} |", ""]
    os.makedirs(os.path.dirname(args.report), exist_ok=True)
    with open(args.report, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print(f"dur={dur:.2f}s a_rms={a_rms:.2f} j_rms={j_rms:.1f}")
    print(f"tau_peak={np.round(tau_peak,2).tolist()}  limit={tau_max.tolist()}  "
          f"in-limit={'PASS' if ok_tau else 'FAIL'}")
    print(f"baked -> {out_path}\nreport -> {args.report}")
    return 0 if ok_tau else 1


if __name__ == "__main__":
    sys.exit(main())
