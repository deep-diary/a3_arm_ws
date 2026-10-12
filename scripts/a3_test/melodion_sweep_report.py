#!/usr/bin/env python3
"""模式×速度扫描轨迹的报告生成器（离线，复用编排层同源 IK + 样条 + 重定时）。

输入 sweep.yaml（melodion_score2stream --sweep 生成），逐段算命令轨迹平滑度
（j_rms/a_rms）+ 时长，按 mode(ptp/lin)×speed 分组，出 Markdown + 手写 SVG 图表。

用法（先 source a3_shell_env.sh）：
  python3 scripts/a3_test/melodion_sweep_report.py --stream sweep [--out-dir docs/dev]
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import List, Tuple

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, WS)
sys.path.insert(0, os.path.join(WS, "src", "a3_arm_controller", "a3_arm_controller"))
sys.path.insert(0, os.path.join(WS, "scripts"))

import yaml  # noqa: E402
from arm_controller import (  # noqa: E402
    _finite_diff_metrics,
    _interpolate_via_spline,
    _jerk_limited_timed_traj,
)
from melodion_bake import fkR, ik, load_pin, resolve_point  # noqa: E402

VMAX = 1.5
AMAX = 15.0
COLORS = {"ptp": "#1e6bb8", "lin": "#e4572e"}


def fk_xyz(pin, model, q_idx, fid, q6):
    q = np.zeros(model.nq)
    for val, qi in zip(q6, q_idx):
        q[qi] = float(val)
    d = model.createData()
    pin.forwardKinematics(model, d, q)
    pin.updateFramePlacement(model, d, fid)
    return [float(v) for v in d.oMf[fid].translation]


def seg_metrics(pin, model, q_idx, fid, cur_q, seg):
    target = seg.get("target") or {}
    via = seg.get("via") or []
    speed = float(seg.get("speed") or 1.0)
    mode = str(seg.get("mode") or "ptp").strip().lower()
    v = VMAX * speed
    a = AMAX * speed

    if mode == "lin":
        # lin 段用笛卡尔 via 样条近似（pilz LIN 实际规划无法离线复现，仅趋势对比）
        def xyz(point):
            pose = (point or {}).get("pose")
            if pose and "position" in pose:
                p = pose["position"]
                return [float(p["x"]), float(p["y"]), float(p["z"])]
            return None
        seq = [fk_xyz(pin, model, q_idx, fid, cur_q)]
        for vp in via:
            x = xyz(vp)
            if x:
                seq.append(x)
        t = xyz(target)
        if t:
            seq.append(t)
        dense = _interpolate_via_spline(seq, 200)
        timed = _jerk_limited_timed_traj(dense, v, a) if dense else None
        qt = resolve_point(pin, model, q_idx, fid, target)
        next_q = qt or cur_q
    else:
        qt = resolve_point(pin, model, q_idx, fid, target)
        if qt is None:
            return None, None
        next_q = qt
        refR = fkR(pin, model, q_idx, fid, qt)
        via_q = [cur_q]
        for vp in via:
            qv = resolve_point(pin, model, q_idx, fid, vp, refR)
            if qv is None:
                qv = resolve_point(pin, model, q_idx, fid, vp)
            if qv is None:
                return None, None
            via_q.append(qv)
        via_q.append(qt)
        dense = _interpolate_via_spline(via_q, 200)
        timed = _jerk_limited_timed_traj(dense, v, a) if dense else None

    if timed is None:
        return None, next_q
    times, pos = timed
    m = _finite_diff_metrics(times, pos)
    return {
        "dur": float(times[-1]),
        "j_rms": m[1] if m else float("nan"),
        "a_rms": m[0] if m else float("nan"),
    }, next_q


def write_svg(rows: List[Tuple[str, float, float, float, float]], out_svg: str) -> None:
    """画 3 面板折线图（速度 vs 时长 / j_rms / a_rms），ptp/lin 两线。"""
    modes = sorted({r[0] for r in rows})
    speeds = sorted({r[1] for r in rows})
    panels = [("时长 (s)", 2, "dur"), ("j_rms (rad/s³)", 3, "j_rms"), ("a_rms (rad/s²)", 4, "a_rms")]

    W, H = 320 * len(panels), 340
    ML, MR, MT, MB = 60, 20, 30, 40
    pw = (W - ML - MR - (len(panels) - 1) * 30) / len(panels)

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
             f'viewBox="0 0 {W} {H}" font-family="monospace">']
    parts.append(f'<rect width="{W}" height="{H}" fill="white"/>')

    for pi, (title, key, _k) in enumerate(panels):
        left = ML + pi * (pw + 30)
        right = left + pw
        vals = [r[key] for r in rows if r[key] == r[key]]  # 排除 nan
        ymin, ymax = (min(vals), max(vals)) if vals else (0.0, 1.0)
        if ymax - ymin < 1e-9:
            ymax = ymin + 1.0

        def sx(v):
            return left + (v - speeds[0]) / (speeds[-1] - speeds[0]) * (right - left)

        def sy(v):
            return MT + (H - MT - MB) - (v - ymin) / (ymax - ymin) * (H - MT - MB)

        parts.append(f'<text x="{left}" y="{MT-10}" font-size="13" fill="#333">{title}</text>')
        for gx in (speeds[0], speeds[len(speeds) // 2], speeds[-1]):
            parts.append(f'<line x1="{sx(gx):.1f}" y1="{MT}" x2="{sx(gx):.1f}" y2="{H-MB}" '
                         f'stroke="#eee" stroke-width="1"/>')
            parts.append(f'<text x="{sx(gx):.1f}" y="{H-MB+14}" font-size="10" fill="#666" '
                         f'text-anchor="middle">{gx:.1f}</text>')
        for mode in modes:
            pts = [(r[1], r[key]) for r in rows if r[0] == mode and r[key] == r[key]]
            if not pts:
                continue
            pts.sort()
            poly = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in pts)
            parts.append(f'<polyline points="{poly}" fill="none" stroke="{COLORS[mode]}" '
                         f'stroke-width="2"/>')
            for x, y in pts:
                parts.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3" fill="{COLORS[mode]}"/>')
        # y 轴范围标注
        parts.append(f'<text x="{right}" y="{MT+10}" font-size="9" fill="#999" text-anchor="end">'
                     f'{ymax:.3g}</text>')
        parts.append(f'<text x="{right}" y="{H-MB-6}" font-size="9" fill="#999" text-anchor="end">'
                     f'{ymin:.3g}</text>')

    # 图例
    lx = ML
    for mode in modes:
        parts.append(f'<line x1="{lx}" y1="{H-14}" x2="{lx+20}" y2="{H-14}" '
                     f'stroke="{COLORS[mode]}" stroke-width="2"/>')
        parts.append(f'<text x="{lx+24}" y="{H-9}" font-size="12">{mode}</text>')
        lx += 70
    parts.append(f'<text x="{ML+200}" y="{H-9}" font-size="11" fill="#666">x 轴 = speed（0.1~1.0）</text>')
    parts.append("</svg>")
    with open(out_svg, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))


def main() -> None:
    ap = argparse.ArgumentParser(description="模式×速度扫描轨迹报告")
    ap.add_argument("--stream", default="sweep", help="stream 名（config/streams/<name>.yaml）")
    ap.add_argument("--out-dir", default=None, help="输出目录（默认 docs/dev）")
    args = ap.parse_args()

    cfg_dir = os.path.join(WS, "src", "app_melodion", "config")
    stream_path = os.path.join(cfg_dir, "streams", args.stream + ".yaml")
    with open(stream_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    pin, model, q_idx, fid = load_pin()
    segments = data.get("segments") or []
    cur_q = resolve_point(pin, model, q_idx, fid, segments[0].get("target") or {})
    if cur_q is None:
        print("首段 IK 失败", file=sys.stderr)
        sys.exit(2)

    rows: List[Tuple[str, float, float, float, float]] = []
    for seg in segments[1:]:
        m, nq = seg_metrics(pin, model, q_idx, fid, cur_q, seg)
        cur_q = nq if nq is not None else cur_q
        if m is None:
            continue
        rows.append((str(seg.get("mode") or "ptp"), float(seg.get("speed") or 1.0),
                     m["dur"], m["j_rms"], m["a_rms"]))

    out_dir = args.out_dir or os.path.join(WS, "docs", "dev")
    os.makedirs(out_dir, exist_ok=True)

    svg = os.path.join(out_dir, "sweep_report.svg")
    write_svg(rows, svg)

    modes = sorted({r[0] for r in rows})
    speeds = sorted({r[1] for r in rows})
    lines = ["# 模式×速度扫描轨迹报告（sweep）", ""]
    lines.append(f"- 轨迹：{args.stream}（F3↔C6 一来一回，ptp/lin × speed 0.1~1.0）")
    lines.append(f"- 命令轨迹平滑度（vmax={VMAX} rad/s, amax={AMAX} rad/s²；lin 用笛卡尔 via 样条近似）")
    lines.append("")
    lines.append("![扫描图表](sweep_report.svg)")
    lines.append("")
    lines.append("| mode | speed | 时长 s | j_rms rad/s³ | a_rms rad/s² |")
    lines.append("|---|---|---|---|---|")
    for mode in modes:
        for speed in speeds:
            r = [x for x in rows if x[0] == mode and abs(x[1] - speed) < 1e-9]
            if not r:
                continue
            _, _, dur, j, a = r[0]
            lines.append(f"| {mode} | {speed:.1f} | {dur:.2f} | {j:.3f} | {a:.4f} |")
    lines.append("")
    lines.append("> j_rms/a_rms 越小越平滑；时长反映速度缩放对运动时间的影响。")
    lines.append("> 稳定性判定需结合仿真/真机的跟踪误差（见 melodion_sweep_run.py）。")
    lines.append("")

    md = os.path.join(out_dir, "sweep_report.md")
    with open(md, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"报告 -> {md}")
    print(f"图表 -> {svg}")


if __name__ == "__main__":
    main()
