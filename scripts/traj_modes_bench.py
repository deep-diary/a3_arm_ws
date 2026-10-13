#!/usr/bin/env python3
"""多路点轨迹模式基准对比（离线：命令轨迹生成 + 平滑度指标 + SVG/MD 报告）。

给定 N 个命名位姿点（2~6），用 5 种轨迹模式（ptp/L1/L2/L3/L1.5）× 速度扫描，
离线生成 L2/L3/L1.5 的命令轨迹并算平滑度（j_rms/a_rms）+ 时长，输出 SVG + MD。
ptp/L1 依赖 move_group，命令轨迹在线生成（由 traj_modes_run.py 采集跟踪误差）。

用法（先 source a3_shell_env.sh）：
  python3 scripts/traj_modes_bench.py --config src/a3_arm_controller/config/traj_modes_bench.yaml
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, WS)
sys.path.insert(0, os.path.join(WS, "src", "a3_arm_controller", "a3_arm_controller"))
sys.path.insert(0, os.path.join(WS, "scripts"))

import yaml  # noqa: E402
from arm_controller import (  # noqa: E402
    _bell_timed_traj,
    _interpolate_via_spline,
    _jerk_limited_timed_traj,
    _trapezoidal_timed_traj,
)
from traj_smoothness_calibration import compute_metrics  # noqa: E402

COLORS = {
    "ptp": "#7f7f7f", "l1": "#e4572e", "l2": "#d62728",
    "l3": "#1e6bb8", "l1p5": "#2ca02c",
}
MODE_LABELS = {
    "ptp": "ptp 逐点 MoveJ", "l1": "L1 blend", "l2": "L2 梯形",
    "l3": "L3 钟形", "l1p5": "L1.5 保形",
}
OFFLINE_MODES = ("l2", "l3", "l1p5")


def load_named_pose_joints(path: str, name: str) -> Optional[List[float]]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    pose = (data.get("poses") or {}).get(name)
    if not pose:
        return None
    positions = [float(v) for v in pose["positions"]]
    return positions[:6]  # L1-L6


def linear_dense(via: List[List[float]], resample_n: int = 200) -> List[List[float]]:
    """稀疏 via 点 → 直线段连接 + 按段均匀重采样的稠密路径（保形，不过冲）。"""
    via = np.asarray(via, dtype=float)
    n, m = via.shape
    if n < 2:
        return [list(r) for r in via]
    per = max(2, resample_n // (n - 1))
    out: List[List[float]] = [list(via[0])]
    for i in range(n - 1):
        for k in range(1, per + 1):
            out.append(list(via[i] + (via[i + 1] - via[i]) * (k / per)))
    return out


def gen_offline(mode: str, via: List[List[float]], vmax: float, amax: float):
    """生成离线命令轨迹 (times, positions)；L2=线性+梯形，L3=样条+钟形，L1.5=线性+加巡减。"""
    if mode == "l2":
        dense = linear_dense(via)
        return _trapezoidal_timed_traj(dense, vmax, amax)
    if mode == "l3":
        dense = _interpolate_via_spline(via, 200)
        return _bell_timed_traj(dense, vmax, amax) if dense else None
    if mode == "l1p5":
        dense = linear_dense(via)
        return _jerk_limited_timed_traj(dense, vmax, amax)
    return None


def write_svg(rows, out_svg: str) -> None:
    """画 3 面板折线图（速度 vs 时长 / j_rms / a_rms），5 模式多线。"""
    modes = sorted({r["mode"] for r in rows if r.get("j_rms") is not None})
    speeds = sorted({r["speed"] for r in rows})
    panels = [("时长 (s)", "dur"), ("j_rms (rad/s³)", "j_rms"), ("a_rms (rad/s²)", "a_rms")]

    W, H = 340 * len(panels), 360
    ML, MR, MT, MB = 70, 20, 32, 42
    pw = (W - ML - MR - (len(panels) - 1) * 30) / len(panels)

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
             f'viewBox="0 0 {W} {H}" font-family="monospace">']
    parts.append(f'<rect width="{W}" height="{H}" fill="white"/>')

    for pi, (title, key) in enumerate(panels):
        left = ML + pi * (pw + 30)
        right = left + pw
        vals = [r[key] for r in rows if r.get(key) is not None]
        if not vals:
            continue
        ymin, ymax = min(vals), max(vals)
        if ymax - ymin < 1e-9:
            ymax = ymin + 1.0

        def sx(v):
            return left + (v - speeds[0]) / (speeds[-1] - speeds[0]) * (right - left)

        def sy(v):
            return MT + (H - MT - MB) - (v - ymin) / (ymax - ymin) * (H - MT - MB)

        parts.append(f'<text x="{left}" y="{MT-10}" font-size="13" fill="#333">{title}</text>')
        for gx in (speeds[0], speeds[len(speeds) // 2], speeds[-1]):
            parts.append(f'<line x1="{sx(gx):.1f}" y1="{MT}" x2="{sx(gx):.1f}" y2="{H-MB}" '
                         f'stroke="#eee"/>')
            parts.append(f'<text x="{sx(gx):.1f}" y="{H-MB+14}" font-size="10" fill="#666" '
                         f'text-anchor="middle">{gx:.1f}</text>')
        for mode in modes:
            pts = [(r["speed"], r[key]) for r in rows
                   if r["mode"] == mode and r.get(key) is not None]
            pts.sort()
            if not pts:
                continue
            poly = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in pts)
            parts.append(f'<polyline points="{poly}" fill="none" stroke="{COLORS[mode]}" '
                         f'stroke-width="2"/>')
            for x, y in pts:
                parts.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3" '
                             f'fill="{COLORS[mode]}"/>')
        parts.append(f'<text x="{right}" y="{MT+10}" font-size="9" fill="#999" text-anchor="end">'
                     f'{ymax:.3g}</text>')
        parts.append(f'<text x="{right}" y="{H-MB-6}" font-size="9" fill="#999" text-anchor="end">'
                     f'{ymin:.3g}</text>')

    lx = ML
    for mode in modes:
        parts.append(f'<line x1="{lx}" y1="{H-14}" x2="{lx+20}" y2="{H-14}" '
                     f'stroke="{COLORS[mode]}" stroke-width="2"/>')
        parts.append(f'<text x="{lx+24}" y="{H-9}" font-size="11">{MODE_LABELS[mode]}</text>')
        lx += 150
    parts.append("</svg>")
    with open(out_svg, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))


def main() -> None:
    ap = argparse.ArgumentParser(description="多路点轨迹模式基准（离线命令指标）")
    ap.add_argument("--config", required=True, help="traj_modes_bench.yaml 路径")
    ap.add_argument("--named-poses", default=None, help="named_poses.yaml（默认 a3_description 包内）")
    ap.add_argument("--out-dir", default=None, help="输出目录（默认 docs/dev）")
    args = ap.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    if args.named_poses:
        np_path = args.named_poses
    else:
        from ament_index_python.packages import get_package_share_directory
        np_path = os.path.join(
            get_package_share_directory("a3_description"), "config", "named_poses.yaml")

    points = [load_named_pose_joints(np_path, p) for p in cfg["points"]]
    if any(p is None for p in points):
        missing = [n for n, p in zip(cfg["points"], points) if p is None]
        print(f"命名位姿不存在: {missing}", file=sys.stderr)
        sys.exit(2)
    via = points
    vmax = float(cfg.get("vmax_rad_s", 1.5))
    amax = float(cfg.get("amax_rad_s2", 15.0))
    modes = cfg["modes"]
    speeds = [float(s) for s in cfg["speeds"]]

    rows: List[Dict] = []
    for mode in modes:
        for speed in speeds:
            v = vmax * speed
            a = amax * speed
            row = {"mode": mode, "speed": speed, "dur": None, "j_rms": None, "a_rms": None}
            if mode in OFFLINE_MODES:
                timed = gen_offline(mode, via, v, a)
                if timed is not None:
                    times, pos = timed
                    m = compute_metrics(times, pos)
                    row["dur"] = float(times[-1])
                    row["j_rms"] = m["j_rms"]
                    row["a_rms"] = m["a_rms"]
            rows.append(row)

    out_dir = args.out_dir or os.path.join(WS, "docs", "dev")
    os.makedirs(out_dir, exist_ok=True)

    svg = os.path.join(out_dir, "traj_modes_bench.svg")
    write_svg(rows, svg)

    lines = ["# 多路点轨迹模式基准对比（离线命令指标）", ""]
    lines.append(f"- 测试点：{', '.join(cfg['points'])}（{len(points)} 点，关节角）")
    lines.append(f"- 模式：{', '.join(MODE_LABELS[m] for m in modes)}")
    lines.append(f"- vmax={vmax} rad/s, amax={amax} rad/s²；speed 缩放 v/a")
    lines.append("")
    lines.append("![基准图表](traj_modes_bench.svg)")
    lines.append("")
    lines.append("| mode | speed | 时长 s | j_rms rad/s³ | a_rms rad/s² |")
    lines.append("|---|---|---|---|---|")
    for row in rows:
        dur = f"{row['dur']:.2f}" if row["dur"] is not None else "-"
        j = f"{row['j_rms']:.4f}" if row["j_rms"] is not None else "-"
        a = f"{row['a_rms']:.4f}" if row["a_rms"] is not None else "-"
        lines.append(f"| {MODE_LABELS[row['mode']]} | {row['speed']:.1f} | {dur} | {j} | {a} |")
    lines.append("")
    lines.append("> ptp/L1 依赖 move_group，命令指标需在线采集（traj_modes_run.py）。")
    lines.append("> 平滑度排序预期：L3 钟形 < L1.5 保形 < L2 梯形（梯形加速度方波，冲击最大）。")
    lines.append("> 统一标准 = 时长 + j_rms/a_rms（命令平滑度）+ 在线 peak_err/rms_err（跟踪）。")
    lines.append("")

    md = os.path.join(out_dir, "traj_modes_bench.md")
    with open(md, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"报告 -> {md}")
    print(f"图表 -> {svg}")


if __name__ == "__main__":
    main()
