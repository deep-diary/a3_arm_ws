#!/usr/bin/env python3
"""回放轨迹报告生成器（F137 真机对比，离线 MVP）。

输入一条录制轨迹（teach_*.yaml）+ 一次实际执行录制（f137_record.py 的 CSV），
生成逐关节三列位置轨迹叠加图 + 指标表：
  - raw      原始录制位置（teach yaml）
  - smoothed F137 去噪位置（逐关节五次 B 样条，同 ε）
  - actual   实际执行位置（/joint_states 录制，剔除 return 回首段后对齐）

输出到 report/<basename>/：overlay.svg + report.md（git 忽略，不入库）。

示例：
  python3 scripts/make_report.py \
      --traj ~/.a3/trajectories/teach_20261004_184429.yaml \
      --exec /tmp/ab_smooth2.csv --eps 0.01 --skip-start 6
"""

from __future__ import annotations

import argparse
import datetime
import os
import sys
from typing import Dict, List, Tuple

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_smooth import fit_smoothing_splines  # noqa: E402
from traj_smoothness_calibration import compute_metrics, load_recorded_trajectory  # noqa: E402

JOINT_NAMES = ("L1", "L2", "L3", "L4", "L5", "L6", "L7")
COLORS = {"raw": "#e4572e", "smoothed": "#1e6bb8", "actual": "#2ca02c"}


def load_exec_csv(path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """读 f137_record.py 的 CSV：返回 (times, positions, velocities)。"""
    import csv
    rows = list(csv.reader(open(path)))
    hdr = rows[0]
    d = np.array([[float(x) for x in r] for r in rows[1:]])
    t = d[:, 0]
    nq = (len(hdr) - 1) // 2
    pos = d[:, 1:1 + nq]
    vel = d[:, 1 + nq:1 + 2 * nq]
    return t, pos, vel


def align_exec(
    t_exec: np.ndarray, pos_exec: np.ndarray, vel_exec: np.ndarray,
    q0: np.ndarray, skip_start: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """剔除 return 回首段，把轨迹本体对齐到 raw t=0，返回 (t, pos, vel)。"""
    mask = t_exec >= skip_start
    if mask.sum() < 3:
        return t_exec - t_exec[0], pos_exec, vel_exec
    d = np.linalg.norm(pos_exec[mask] - q0, axis=1)
    i0 = int(np.argmin(d)) + int(np.argmax(mask))
    return t_exec[i0:] - t_exec[i0], pos_exec[i0:], vel_exec[i0:]


def _fmt(v: float) -> str:
    if v >= 1e4:
        return f"{v:.3g}"
    if v >= 100:
        return f"{v:.1f}"
    return f"{v:.4f}"


def _downsample(x: np.ndarray, n: int) -> np.ndarray:
    if len(x) <= n:
        return x
    idx = np.linspace(0, len(x) - 1, n).astype(int)
    return x[idx]


def write_overlay_svg(
    joint_names: List[str],
    t_raw: np.ndarray, raw: np.ndarray,
    t_smooth: np.ndarray, smooth: np.ndarray,
    t_act: np.ndarray, act: np.ndarray,
    out_svg: str,
) -> None:
    """逐关节三列位置轨迹叠加 SVG（7 面板）。"""
    W, H = 900, 60 + 7 * 110 + 40
    ML, MR, MT, MB = 80, 20, 30, 40
    panel_h = 110

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
             f'viewBox="0 0 {W} {H}" font-family="monospace">']
    parts.append(f'<rect width="{W}" height="{H}" fill="white"/>')

    tmax = max(t_raw[-1], t_smooth[-1], t_act[-1] if len(t_act) else 0.0)

    for j in range(raw.shape[1]):
        top = MT + j * panel_h
        ymin = float(np.min([raw[:, j].min(), smooth[:, j].min(), act[:, j].min()]))
        ymax = float(np.max([raw[:, j].max(), smooth[:, j].max(), act[:, j].max()]))
        span = (ymax - ymin) or 1e-6

        def sx(v):
            return ML + v / tmax * (W - ML - MR)

        def sy(v):
            return top + panel_h - (v - ymin) / span * (panel_h - 14)

        # 关节名 + 范围
        parts.append(f'<text x="{ML-4}" y="{top+12}" font-size="12" fill="#333" '
                     f'text-anchor="end">{joint_names[j]}</text>')
        parts.append(f'<text x="{ML+4}" y="{top+12}" font-size="10" fill="#999">'
                     f'[{ymin:.3f}..{ymax:.3f}]</text>')

        # 三条曲线（raw 降采样）
        for key, (tx, vx, w, op) in {
            "raw": (t_raw, raw[:, j], 1, 0.5),
            "smoothed": (t_smooth, smooth[:, j], 2, 1.0),
            "actual": (t_act, act[:, j], 1.5, 1.0),
        }.items():
            ti = _downsample(tx, 500)
            vi = _downsample(vx, 500)
            pts = " ".join(f"{sx(float(ti[k])):.1f},{sy(float(vi[k])):.1f}"
                           for k in range(len(ti)))
            parts.append(f'<polyline points="{pts}" fill="none" '
                         f'stroke="{COLORS[key]}" stroke-width="{w}" opacity="{op}"/>')

    # 图例 + 横轴
    lx = ML
    for key, label in (("raw", "raw 原始录制"), ("smoothed", "smoothed F137 去噪"),
                       ("actual", "actual 实际执行")):
        parts.append(f'<line x1="{lx}" y1="{H-16}" x2="{lx+24}" y2="{H-16}" '
                     f'stroke="{COLORS[key]}" stroke-width="2"/>')
        parts.append(f'<text x="{lx+30}" y="{H-11}" font-size="12">{label}</text>')
        lx += 180
    parts.append(f'<text x="{W-MR}" y="{H-16}" font-size="11" fill="#666" '
                 f'text-anchor="end">time (s), 0..{tmax:.1f}</text>')
    parts.append("</svg>")
    with open(out_svg, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))


def velocity_metrics(times: np.ndarray, vel: np.ndarray) -> Dict[str, float]:
    """用 velocity 字段求 v/a（只差一次分，避开位置二次差分噪声）。"""
    a = np.gradient(vel, times, axis=0)
    j = np.gradient(a, times, axis=0)
    return {
        "v_rms": float(np.sqrt((vel ** 2).mean())),
        "a_rms": float(np.sqrt((a ** 2).mean())),
        "a_peak": float(np.abs(a).max()),
        "j_rms": float(np.sqrt((j ** 2).mean())),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="回放轨迹报告生成器（F137 真机对比）")
    ap.add_argument("--traj", required=True, help="录制轨迹 yaml")
    ap.add_argument("--exec", required=True, help="实际执行 CSV（f137_record.py 输出）")
    ap.add_argument("--eps", type=float, default=0.01, help="F137 去噪 ε")
    ap.add_argument("--skip-start", type=float, default=6.0,
                    help="执行录制剔除 return 回首段的最少秒数")
    ap.add_argument("--out-dir", default=None,
                    help="报告输出目录（默认 report/<basename>）")
    args = ap.parse_args(argv)

    # 1) 原始录制
    joint_names, t_rec, positions = load_recorded_trajectory(os.path.expanduser(args.traj))
    raw = np.asarray(positions, dtype=float)
    t_rec = np.asarray(t_rec, dtype=float)

    # 2) F137 去噪（同一时间轴）
    splines, u_all = fit_smoothing_splines(raw, t_rec, args.eps)
    smooth = np.column_stack([spl(u_all) for spl in splines])
    t_smooth = t_rec

    # 3) 实际执行（剔除 return 后对齐）
    t_exec, pos_exec, vel_exec = load_exec_csv(os.path.expanduser(args.exec))
    t_act, act, vel_act = align_exec(t_exec, pos_exec, vel_exec, raw[0], args.skip_start)

    # 指标
    m_raw = compute_metrics(t_rec.tolist(), [list(p) for p in raw])
    m_smooth = compute_metrics(t_smooth.tolist(), [list(p) for p in smooth])
    m_act = velocity_metrics(t_act, vel_act)

    # 输出目录
    base = os.path.splitext(os.path.basename(args.traj))[0]
    out_dir = args.out_dir or os.path.join("report", base)
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    svg = os.path.join(out_dir, "overlay.svg")
    write_overlay_svg(joint_names, t_rec, raw, t_smooth, smooth, t_act, act, svg)

    # Markdown 报告
    lines: List[str] = []
    lines.append(f"# 回放轨迹报告 —— {base}")
    lines.append("")
    lines.append(f"- 生成：{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"- 录制文件：`{os.path.basename(args.traj)}`（{len(t_rec)} 点，{t_rec[-1]-t_rec[0]:.2f}s）")
    lines.append(f"- F137 ε={args.eps:.4f} rad；执行剔除 return 后对齐")
    lines.append("")
    lines.append("![逐关节三列轨迹](overlay.svg)")
    lines.append("")
    lines.append("## 指标对比")
    lines.append("")
    lines.append("| 方法 | v_rms | a_rms | a_peak | j_rms |")
    lines.append("|---|---|---|---|---|")
    for label, m in (("raw", m_raw), ("smoothed", m_smooth), ("actual", m_act)):
        lines.append(f"| {label} | {_fmt(m['v_rms'])} | {_fmt(m['a_rms'])} "
                     f"| {_fmt(m['a_peak'])} | {_fmt(m['j_rms'])} |")
    lines.append("")
    lines.append("> raw/smoothed 用位置差分；actual 用 velocity 字段（只差一次分，更干净）。")
    lines.append("> a_peak 是平滑度的主指标（去噪后峰值加速度应下降）；j_rms 受 TOTG 重定时主导，区分度低。")

    md = os.path.join(out_dir, "report.md")
    with open(md, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print(f"报告 -> {out_dir}/")
    print(f"  overlay.svg : 逐关节三列位置轨迹叠加")
    print(f"  report.md   : 指标表 + 图")
    print(f"\n指标：raw a_peak={m_raw['a_peak']:.1f}  smoothed a_peak={m_smooth['a_peak']:.1f}  "
          f"actual a_peak={m_act['a_peak']:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
