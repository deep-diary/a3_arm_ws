#!/usr/bin/env python3
"""F136 轨迹平滑度标定脚手架（先仿真/离线）。

用两条对照轨迹 + 统一平滑度指标 J + 有界参数扫描，解耦「回放不丝滑」的归因：
  1) 标准轨迹：两点间 quintic 规划（起止零速/零加速度，理论平滑基准）
  2) 录制轨迹：~/.a3/trajectories/latest.yaml（实机/仿真录制，真实输入）

J = 速度/加速度/jerk 的 RMS+峰值 加权（各归一化到基线）+ 几何偏离惩罚。
扫描轨迹生成层参数：velocity_scaling（时间缩放）、smooth window（中心滑动平均）。

纯 Python + PyYAML，无 ROS / 无 numpy 依赖；后续可加 --live 走 retime 服务在线扫描。

示例：
  python3 scripts/traj_smoothness_calibration.py \
      --q0 0,0,0,0,0,0,0 --q1 0.5,0.8,-1.2,0.4,-0.3,0.6,0.5 \
      --scan v_scaling --min 0.2 --max 1.5 --step 0.1
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

# J 权重：平滑度主要由 accel/jerk 主导，速度与几何为次约束（可改）。
DEFAULT_WEIGHTS: Dict[str, float] = {
    "v_rms": 0.05,
    "a_rms": 0.20,
    "j_rms": 0.35,
    "a_peak": 0.15,
    "j_peak": 0.20,
    "geom": 0.05,
}


def _quintic_s(tau: float) -> float:
    """0→0、1→1，起止一/二阶导数均为 0 的五次多项式位置剖线。"""
    return 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5


def build_standard_trajectory(
    q0: Sequence[float],
    q1: Sequence[float],
    duration: float = 4.0,
    samples: int = 200,
) -> Tuple[List[float], List[List[float]]]:
    """两点间 quintic 平滑轨迹：time_from_start 列表 + 每点位置列表。"""
    if samples < 2:
        samples = 2
    times = [duration * i / (samples - 1) for i in range(samples)]
    positions: List[List[float]] = []
    for t in times:
        s = _quintic_s(t / duration)
        positions.append([a + (b - a) * s for a, b in zip(q0, q1)])
    return times, positions


def load_recorded_trajectory(
    path: str,
) -> Tuple[List[str], List[float], List[List[float]]]:
    """读取录制 YAML（arm_controller `_dump_recording` / `_writeback_retimed` 格式）。"""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    joint_names = list(data.get("joint_names") or [])
    times: List[float] = []
    positions: List[List[float]] = []
    for p in data.get("points") or []:
        times.append(float(p["time_from_start_sec"]))
        positions.append([float(v) for v in p["positions"]])
    return joint_names, times, positions


def _derivatives(
    times: Sequence[float], col: Sequence[float]
) -> Tuple[List[float], List[float], List[float]]:
    """单关节标量序列的中心/单边有限差分：速度、加速度、jerk。"""
    n = len(col)
    v = [0.0] * n
    a = [0.0] * n
    j = [0.0] * n
    for i in range(n):
        if i == 0:
            dt = times[1] - times[0] if n > 1 else 1.0
            v[i] = (col[1] - col[0]) / dt if n > 1 else 0.0
        elif i == n - 1:
            dt = times[i] - times[i - 1]
            v[i] = (col[i] - col[i - 1]) / dt if dt > 0 else 0.0
        else:
            dt = times[i + 1] - times[i - 1]
            v[i] = (col[i + 1] - col[i - 1]) / dt if dt > 0 else 0.0
    for i in range(n):
        dt = (times[1] - times[0]) if i == 0 else (times[i] - times[i - 1])
        if i == 0:
            a[i] = (v[1] - v[0]) / dt if (n > 1 and dt > 0) else 0.0
        else:
            a[i] = (v[i] - v[i - 1]) / dt if dt > 0 else 0.0
    for i in range(n):
        dt = (times[1] - times[0]) if i == 0 else (times[i] - times[i - 1])
        if i == 0:
            j[i] = (a[1] - a[0]) / dt if (n > 1 and dt > 0) else 0.0
        else:
            j[i] = (a[i] - a[i - 1]) / dt if dt > 0 else 0.0
    return v, a, j


def _rms(vals: Sequence[float]) -> float:
    if not vals:
        return 0.0
    return math.sqrt(sum(x * x for x in vals) / len(vals))


def _peak(vals: Sequence[float]) -> float:
    return max((abs(x) for x in vals), default=0.0)


def compute_metrics(
    times: Sequence[float], positions: Sequence[Sequence[float]]
) -> Dict[str, float]:
    """逐关节有限差分求 vel/acc/jerk，聚合为跨关节 RMS 与峰值。"""
    n = len(positions)
    n_joints = len(positions[0]) if n else 0
    if n < 3 or n_joints == 0:
        return {
            "v_rms": 0.0, "a_rms": 0.0, "j_rms": 0.0,
            "a_peak": 0.0, "j_peak": 0.0,
            "n_points": n, "n_joints": n_joints,
        }
    v_rms = a_rms = j_rms = 0.0
    a_peak = j_peak = 0.0
    for jidx in range(n_joints):
        col = [p[jidx] for p in positions]
        v, a, j = _derivatives(times, col)
        v_rms += _rms(v)
        a_rms += _rms(a)
        j_rms += _rms(j)
        a_peak = max(a_peak, _peak(a))
        j_peak = max(j_peak, _peak(j))
    return {
        "v_rms": v_rms / n_joints,
        "a_rms": a_rms / n_joints,
        "j_rms": j_rms / n_joints,
        "a_peak": a_peak,
        "j_peak": j_peak,
        "n_points": n,
        "n_joints": n_joints,
    }


def apply_time_scale(
    times: Sequence[float], positions: Sequence[Sequence[float]], v_scale: float
) -> Tuple[List[float], List[List[float]]]:
    """匀速时间缩放：几何不变，t' = t / v_scale（v_scale>1 更快）。"""
    if v_scale <= 0:
        raise ValueError("v_scale must be > 0")
    return [t / v_scale for t in times], [list(p) for p in positions]


def smooth_positions(
    positions: Sequence[Sequence[float]], window: int
) -> List[List[float]]:
    """逐关节中心滑动平均（window<3 原样返回）。"""
    if window < 3:
        return [list(p) for p in positions]
    n = len(positions)
    n_joints = len(positions[0])
    half = window // 2
    out: List[List[float]] = []
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        cnt = hi - lo
        out.append(
            [sum(positions[k][j] for k in range(lo, hi)) / cnt for j in range(n_joints)]
        )
    return out


def geometry_deviation(
    ref: Sequence[Sequence[float]], new: Sequence[Sequence[float]]
) -> float:
    """逐点最大关节偏差的均值（rad）——惩罚平滑把路径改没。"""
    if len(ref) != len(new):
        return float("inf")
    total = 0.0
    for r, q in zip(ref, new):
        total += max(abs(q[k] - r[k]) for k in range(len(r)))
    return total / len(ref)


def score(
    metrics: Dict[str, float],
    ref_metrics: Dict[str, float],
    geom: float = 0.0,
    weights: Optional[Dict[str, float]] = None,
) -> Tuple[float, Dict[str, float]]:
    """相对基线归一化的复合代价 J（≤1 表示优于/等于基线，越小越平滑）。"""
    w = weights or DEFAULT_WEIGHTS
    terms: Dict[str, float] = {}
    total = 0.0
    for key in ("v_rms", "a_rms", "j_rms", "a_peak", "j_peak"):
        base = max(float(ref_metrics.get(key, 0.0)), 1e-9)
        terms[key] = metrics[key] / base
        total += w[key] * terms[key]
    terms["geom"] = geom
    total += w["geom"] * geom
    return total, terms


def _fmt_metrics(m: Dict[str, float]) -> str:
    return (
        f"v_rms={m['v_rms']:.4f} a_rms={m['a_rms']:.4f} j_rms={m['j_rms']:.4f} "
        f"a_peak={m['a_peak']:.4f} j_peak={m['j_peak']:.4f}"
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="F136 轨迹平滑度标定脚手架（离线）")
    ap.add_argument("--traj", default="~/.a3/trajectories/latest.yaml",
                    help="录制轨迹 YAML（默认 latest.yaml）")
    ap.add_argument("--q0", default="0,0,0,0,0,0,0", help="标准轨迹起点（7 关节）")
    ap.add_argument("--q1", default="0.5,0.8,-1.2,0.4,-0.3,0.6,0.5",
                    help="标准轨迹终点（7 关节）")
    ap.add_argument("--duration", type=float, default=4.0, help="标准轨迹时长 s")
    ap.add_argument("--samples", type=int, default=200, help="标准轨迹采样数")
    ap.add_argument("--scan", choices=["v_scaling", "smooth"], default="v_scaling",
                    help="扫描维度")
    ap.add_argument("--min", type=float, default=0.2, help="扫描下限")
    ap.add_argument("--max", type=float, default=1.5, help="扫描上限")
    ap.add_argument("--step", type=float, default=0.1, help="扫描步长")
    ap.add_argument("--out", default=None, help="可选 CSV 输出路径")
    args = ap.parse_args(argv)

    def parse_joints(s: str) -> List[float]:
        return [float(x) for x in s.split(",") if x.strip() != ""]

    q0 = parse_joints(args.q0)
    q1 = parse_joints(args.q1)
    if len(q0) != len(q1):
        print(f"q0/q1 关节数不一致: {len(q0)} vs {len(q1)}", file=sys.stderr)
        return 2

    # 1) 标准轨迹（理论平滑基准）
    std_times, std_pos = build_standard_trajectory(q0, q1, args.duration, args.samples)
    std_metrics = compute_metrics(std_times, std_pos)
    print("== 标准轨迹（quintic 两点，理论平滑基准）==")
    print(f"   {_fmt_metrics(std_metrics)}  (n={std_metrics['n_points']})")

    # 2) 录制轨迹（真实输入，可选）
    traj_path = os.path.expanduser(args.traj)
    rec_times: List[float] = []
    rec_pos: List[List[float]] = []
    if os.path.exists(traj_path):
        try:
            jn, rec_times, rec_pos = load_recorded_trajectory(traj_path)
            rec_metrics = compute_metrics(rec_times, rec_pos)
            print(f"== 录制轨迹 {traj_path} ({len(jn)} 关节) ==")
            print(f"   {_fmt_metrics(rec_metrics)}  (n={rec_metrics['n_points']})")
            scan_times, scan_pos, scan_base = rec_times, rec_pos, rec_metrics
        except Exception as exc:  # noqa: BLE001
            print(f"[warn] 录制轨迹解析失败，退回标准轨迹扫描: {exc}", file=sys.stderr)
            scan_times, scan_pos, scan_base = std_times, std_pos, std_metrics
    else:
        print(f"[warn] 未找到录制轨迹 {traj_path}，用标准轨迹扫描", file=sys.stderr)
        scan_times, scan_pos, scan_base = std_times, std_pos, std_metrics

    # 3) 参数扫描
    print(f"\n== 扫描 {args.scan} ∈ [{args.min},{args.max}] step={args.step} ==")
    rows: List[Tuple[float, Dict[str, float], float]] = []
    if args.scan == "v_scaling":
        v = args.min
        while v <= args.max + 1e-9:
            t2, p2 = apply_time_scale(scan_times, scan_pos, v)
            m = compute_metrics(t2, p2)
            j, _ = score(m, scan_base)
            rows.append((v, m, j))
            v += args.step
        print(f"  {'v_scale':>8} | {'v_rms':>8} {'a_rms':>8} {'j_rms':>8} "
              f"{'a_peak':>8} {'j_peak':>8} | {'J':>8}")
        for v, m, j in rows:
            print(f"  {v:>8.2f} | {m['v_rms']:>8.4f} {m['a_rms']:>8.4f} "
                  f"{m['j_rms']:>8.4f} {m['a_peak']:>8.4f} {m['j_peak']:>8.4f} "
                  f"| {j:>8.4f}")
        best = min(rows, key=lambda r: r[2])
        print(f"\n  best v_scale={best[0]:.2f}  J={best[2]:.4f}")
    else:  # smooth
        windows = [1, 3, 5, 7, 9, 11, 15, 21]
        print("== 扫描 smooth（固定窗口集 [1,3,5,7,9,11,15,21]）==")
        print(f"  {'window':>7} | {'v_rms':>8} {'a_rms':>8} {'j_rms':>8} "
              f"{'a_peak':>8} {'j_peak':>8} {'geom':>8} | {'J':>8}")
        for w in windows:
            p2 = smooth_positions(scan_pos, w)
            geom = geometry_deviation(scan_pos, p2)
            m = compute_metrics(scan_times, p2)
            j, _ = score(m, scan_base, geom=geom)
            rows.append((float(w), m, j))
            print(f"  {w:>7} | {m['v_rms']:>8.4f} {m['a_rms']:>8.4f} "
                  f"{m['j_rms']:>8.4f} {m['a_peak']:>8.4f} {m['j_peak']:>8.4f} "
                  f"{geom:>8.4f} | {j:>8.4f}")
        best = min(rows, key=lambda r: r[2])
        print(f"\n  best window={int(best[0])}  J={best[2]:.4f}")

    # 4) 可选 CSV
    if args.out:
        out_path = os.path.expanduser(args.out)
        with open(out_path, "w", encoding="utf-8") as f:
            f.write("param,v_rms,a_rms,j_rms,a_peak,j_peak,J\n")
            for param, m, j in rows:
                f.write(f"{param},{m['v_rms']:.6f},{m['a_rms']:.6f},"
                        f"{m['j_rms']:.6f},{m['a_peak']:.6f},{m['j_peak']:.6f},"
                        f"{j:.6f}\n")
        print(f"  CSV -> {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
