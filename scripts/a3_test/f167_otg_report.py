#!/usr/bin/env python3
"""F167.3 OTG 演示报告：读 otg_demo 的 CSV → 指标 + SVG + MD（numpy-only，手写 SVG）。

场景（otg_demo 产出）：
  * otg_sync.csv     : 多轴 Time 同步（全轴同时到达）
  * otg_async.csv    : 同步关闭（各轴独立，"无同步"对照）
  * otg_retarget.csv : 运动中 t≈0.5s 切换目标（不停车）
  * otg_brake.csv    : 运动中触发急停（jerk-limited 制动）
用法：
  python3 scripts/a3_test/f167_otg_report.py --dir /tmp --out-dir docs/dev
退出码：0=PASS（同步到达差≈0、改目标速度连续、制动到零速），1=FAIL。
"""
from __future__ import annotations

import argparse
import os
from typing import Dict, Tuple

import numpy as np

DOF = 6
SYNC_TGT = np.array([1.0, 0.5, -0.8, 0.3, -0.2, 0.6])
EPS = 1e-3


def load(path: str) -> Dict[str, np.ndarray]:
    a = np.loadtxt(path, delimiter=",", comments="#")
    return {"t": a[:, 0], "q": a[:, 1:7], "v": a[:, 7:13], "a": a[:, 13:19]}


def arrival_time(t, q, v, tgt) -> float:
    """各关节首次「到位（|q-tgt|<EPS 且 |v|<EPS）」的最晚时刻（全轴到齐时间）。"""
    last = 0.0
    for i in range(DOF):
        m = (np.abs(q[:, i] - tgt[i]) < EPS) & (np.abs(v[:, i]) < EPS)
        idx = np.where(m)[0]
        if len(idx):
            last = max(last, float(t[idx[0]]))
    return last


def peaks(d: Dict[str, np.ndarray]) -> Tuple[float, float, float]:
    j = np.gradient(d["a"], d["t"], axis=0)
    return (float(np.max(np.abs(d["v"]))), float(np.max(np.abs(d["a"]))),
            float(np.max(np.abs(j))))


def svg_ts(series: Dict[str, Tuple[np.ndarray, np.ndarray]], ylab, title, out_svg,
           colors=None, height=360) -> None:
    W, H = 620, int(height)
    L, T, R, B = 64, 26, W - 16, H - 40
    allt = np.concatenate([t for t, _ in series.values()])
    ally = np.concatenate([y for _, y in series.values()])
    xlo, xhi = 0.0, float(allt.max())
    ylo, yhi = 0.0, float(ally.max() * 1.1) if ally.max() > 0 else 1.0

    def sx(x): return L + (x - xlo) / (xhi - xlo or 1) * (R - L)
    def sy(y): return B - (y - ylo) / (yhi - ylo or 1) * (B - T)

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
             f'viewBox="0 0 {W} {H}" font-family="monospace">'
             f'<rect width="{W}" height="{H}" fill="white"/>']
    parts.append(f'<rect x="{L}" y="{T}" width="{R-L}" height="{B-T}" fill="none" stroke="#bbb"/>')
    parts.append(f'<text x="{(L+R)/2:.0f}" y="{T-8}" font-size="13" text-anchor="middle" '
                 f'fill="#222">{title}</text>')
    parts.append(f'<text x="{(L+R)/2:.0f}" y="{B+28}" font-size="11" text-anchor="middle" '
                 f'fill="#555">时间 t (s)</text>')
    parts.append(f'<text x="{L-8}" y="{(T+B)/2:.0f}" font-size="11" fill="#555" '
                 f'transform="rotate(-90 {L-8} {(T+B)/2:.0f})" text-anchor="middle">{ylab}</text>')
    for f in (0.0, 0.25, 0.5, 0.75, 1.0):
        gx, gy = L + f * (R - L), B - f * (B - T)
        parts.append(f'<line x1="{gx:.1f}" y1="{T}" x2="{gx:.1f}" y2="{B}" stroke="#eee"/>')
        parts.append(f'<line x1="{L}" y1="{gy:.1f}" x2="{R}" y2="{gy:.1f}" stroke="#eee"/>')
        parts.append(f'<text x="{gx:.1f}" y="{B+14}" font-size="9" fill="#888" '
                     f'text-anchor="middle">{xlo+f*(xhi-xlo):.2f}</text>')
        parts.append(f'<text x="{L-4}" y="{gy+3:.1f}" font-size="9" fill="#888" '
                     f'text-anchor="end">{ylo+f*(yhi-ylo):.2f}</text>')
    cmap = colors or {}
    for i, (name, (t, y)) in enumerate(series.items()):
        col = cmap.get(name, ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf"][i % 6])
        poly = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(t, y))
        parts.append(f'<polyline points="{poly}" fill="none" stroke="{col}" stroke-width="1.8"/>')
        parts.append(f'<line x1="{L+i*90}" y1="{H-12}" x2="{L+i*90+16}" y2="{H-12}" '
                     f'stroke="{col}" stroke-width="3"/>')
        parts.append(f'<text x="{L+i*90+20}" y="{H-9}" font-size="9" fill="#333">{name}</text>')
    parts.append("</svg>")
    with open(out_svg, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="/tmp")
    ap.add_argument("--out-dir", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "docs", "dev"))
    args = ap.parse_args()

    s = load(os.path.join(args.dir, "otg_sync.csv"))
    asy = load(os.path.join(args.dir, "otg_async.csv"))
    rt = load(os.path.join(args.dir, "otg_retarget.csv"))
    bk = load(os.path.join(args.dir, "otg_brake.csv"))

    sync_spread = arrival_time(s["t"], s["q"], s["v"], SYNC_TGT)  # 全轴到齐
    # 各轴到位时刻的离散度
    def spread(d):
        ts = []
        for i in range(DOF):
            m = (np.abs(d["q"][:, i] - SYNC_TGT[i]) < EPS) & (np.abs(d["v"][:, i]) < EPS)
            idx = np.where(m)[0]
            if len(idx):
                ts.append(float(d["t"][idx[0]]))
        return (max(ts) - min(ts)) if len(ts) == DOF else float("nan")
    s_sp = spread(s)
    a_sp = spread(asy)

    # retarget：切换点(t≈0.5)速度连续 & 不停车
    idx = int(np.argmin(np.abs(rt["t"] - 0.5)))
    dv = float(np.max(np.abs(rt["v"][idx + 1] - rt["v"][idx])))
    v_at_switch = float(np.max(np.abs(rt["v"][idx])))

    # brake：制动时长/距离（触发 t≈0.3）
    b0 = int(np.argmin(np.abs(bk["t"] - 0.3)))
    stop = None
    for k in range(b0, len(bk["t"])):
        if np.max(np.abs(bk["v"][k])) < EPS:
            stop = k
            break
    brake_time = float(bk["t"][stop] - bk["t"][b0]) if stop else float("nan")
    brake_dist = float(np.max(np.abs(bk["q"][stop] - bk["q"][b0]))) if stop else float("nan")
    brake_time_total = float(bk["t"][-1] - bk["t"][0])

    p_s = peaks(s); p_asy = peaks(asy); p_rt = peaks(rt); p_bk = peaks(bk)

    out = args.out_dir
    os.makedirs(out, exist_ok=True)
    svg_ts({f"q{i}": (s["t"], s["q"][:, i]) for i in range(DOF)},
           "关节位置 (rad)", "多轴同步：6 轴同时到达", os.path.join(out, "f167_otg_sync.svg"))
    svg_ts({"retarget |v|max": (rt["t"], np.max(np.abs(rt["v"]), axis=1)),
            "brake |v|max": (bk["t"], np.max(np.abs(bk["v"]), axis=1))},
           "峰值速度 |v| (rad/s)", "改目标 / 急停：速度剖面",
           os.path.join(out, "f167_otg_speed.svg"),
           colors={"retarget |v|max": "#1f77b4", "brake |v|max": "#d62728"})

    ok = (abs(s_sp) < 0.05) and (a_sp > s_sp + 0.05) and (dv < 0.05) and (v_at_switch > 0.05) \
        and (not np.isnan(brake_time))

    lines = ["# F167.3 实时 OTG 演示（Ruckig，仿真-only 数值验证）", "",
             "- 生成：`a3_trajectory_processing/otg_demo`（复用栈内 ruckig）；指标/SVG 由本脚本离线算。",
             "- 不接生产 SERVO/teleop 50Hz 链路，纯数值演示。", "",
             f"| 场景 | 指标 | 值 |", "|---|---|---|",
             f"| 多轴同步(Time) | 各轴到位时刻离散度 | {s_sp*1000:.1f} ms |",
             f"| 无同步(None) | 各轴到位时刻离散度 | {a_sp*1000:.1f} ms |",
             f"| 同步轨迹 | 峰值 v / a / jerk | {p_s[0]:.2f} / {p_s[1]:.2f} / {p_s[2]:.1f} |",
             f"| 改目标 | 切换点速度跳变 max|Δv| | {dv:.4f} rad/s |",
             f"| 改目标 | 切换点速度(不停车) | {v_at_switch:.3f} rad/s |",
             f"| 急停 | 制动时长 | {brake_time*1000:.1f} ms |",
             f"| 急停 | 制动距离(最大单关节) | {brake_dist*1000:.1f} mrad |",
             f"| 急停 | 峰值 jerk | {p_bk[2]:.1f} rad/s³ |", "",
             "## 多轴同步（6 轴同时到达）", "", "![sync](f167_otg_sync.svg)", "",
             "## 改目标 / 急停速度剖面", "", "![speed](f167_otg_speed.svg)", "",
             f"- 判定：同步离散≈0 且 < 无同步；改目标速度连续且不停车；急停到零速 → "
             f"{'PASS ✓' if ok else 'FAIL ✗'}"]
    with open(os.path.join(out, "F167_otg.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"sync_spread={s_sp*1000:.1f}ms async_spread={a_sp*1000:.1f}ms "
          f"retarget_dv={dv:.4f} v@switch={v_at_switch:.3f} "
          f"brake={brake_time*1000:.1f}ms/{brake_dist*1000:.1f}mrad")
    print(f"report -> {os.path.join(out, 'F167_otg.md')}")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
