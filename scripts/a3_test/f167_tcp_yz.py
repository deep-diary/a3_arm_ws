#!/usr/bin/env python3
"""F167 附图：4 个自研(软件)模式的 TCP 空间轨迹（y-z 平面，x 不变）+ 性能指标。

因 numpy/matplotlib 环境隔离（LL-152），分两步运行：
  # 1) FK + 指标（ROS 环境，需 pinocchio）
  source scripts/a3_shell_env.sh
  python3 scripts/a3_test/f167_tcp_yz.py dump --run /tmp/traj_modes_tcp.json --out /tmp/f167_tcp_yz.json
  # 2) 出图（默认环境，**不要** source a3_shell_env.sh）
  python3 scripts/a3_test/f167_tcp_yz.py plot --in /tmp/f167_tcp_yz.json --out-dir docs/dev
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from typing import Dict, List, Tuple

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(WS, "scripts", "a3_test"))
SOFT = ["jnt_g64_quintic", "jnt_g64_trapezoid", "jnt_spline_bell", "jnt_spline_approx"]
COLORS = {"jnt_g64_quintic": "#2ca02c", "jnt_g64_trapezoid": "#d62728",
          "jnt_spline_bell": "#9467bd", "jnt_spline_approx": "#e377c2"}
LABELS = {"jnt_g64_quintic": "jnt_g64_quintic（G64+quintic）",
          "jnt_g64_trapezoid": "jnt_g64_trapezoid（G64+梯形）",
          "jnt_spline_bell": "jnt_spline_bell（样条+钟形）",
          "jnt_spline_approx": "jnt_spline_approx（逼近样条）"}


def _joint_metrics(samples: List[List[float]], n_total: int) -> Tuple[float, float, float]:
    """模式段关节空间 (dur, a_rms, j_rms)。"""
    import numpy as np
    rows = [s for s in samples if int(s[8]) < n_total] or samples
    rows.sort(key=lambda r: r[0])
    t = np.array([r[0] for r in rows], dtype=float); t -= t[0]
    q = np.array([[r[1 + i] for i in range(6)] for r in rows], dtype=float)
    keep = np.concatenate([[True], np.diff(t) > 1e-6]); t, q = t[keep], q[keep]
    if len(t) > 3:
        dq = np.max(np.abs(np.diff(q, axis=0)), axis=1)
        if np.any(dq > 1e-5):
            last = int(np.max(np.where(dq > 1e-5)[0])) + 1
            t, q = t[:last + 1], q[:last + 1]
    if len(t) < 5:
        return (float(t[-1]) if len(t) else 0.0), 0.0, 0.0
    tu = np.arange(t[0], t[-1], 0.02)
    qu = np.column_stack([np.interp(tu, t, q[:, k]) for k in range(6)])
    v = np.gradient(qu, tu, axis=0); a = np.gradient(v, tu, axis=0); j = np.gradient(a, tu, axis=0)
    return float(tu[-1]), float(np.sqrt(np.mean(a ** 2))), float(np.sqrt(np.mean(j ** 2)))


def do_dump(args: argparse.Namespace) -> int:
    import numpy as np
    from traj_modes_tcp_report import load_pin, process_mode
    import yaml

    with open(args.run, "r", encoding="utf-8") as f:
        data = json.load(f)
    with open(args.waypoints, "r", encoding="utf-8") as f:
        wp = yaml.safe_load(f)
    targets = [[w["pose"]["position"]["y"], w["pose"]["position"]["z"]] for w in wp["waypoints"]]
    P = np.array(targets, dtype=float)

    def dev_mm(yz: np.ndarray) -> float:
        def seg(pt, a, b):
            ab = b - a
            tt = np.clip(np.dot(pt - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1)
            return np.linalg.norm(pt - (a + tt * ab))
        return max((min(seg(p, P[i], P[i + 1]) for i in range(len(P) - 1)) for p in yz), default=0.0) * 1000.0

    pin, model, q_idx, fid = load_pin()
    out: Dict = {"targets": targets, "modes": {}}
    print(f"{'模式':22s} {'时长s':>6s} {'TCP峰速':>8s} {'TCP峰加速':>9s} {'路径长m':>8s} {'距直线mm':>8s} "
          f"{'a_rms':>7s} {'j_rms':>7s}")
    for m in SOFT:
        rec = data["modes"].get(m)
        if not rec or not rec.get("samples"):
            continue
        nt = int(rec.get("n_total", 2))
        t, xyz, speed, accel = process_mode(pin, model, q_idx, fid, rec["samples"], nt)
        yz = [[float(p[1]), float(p[2])] for p in xyz]
        dur, a_rms, j_rms = _joint_metrics(rec["samples"], nt)
        plen = float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))) if len(xyz) > 1 else 0.0
        mtr = {"dur": dur, "vmax": float(speed.max()) if len(speed) else 0.0,
               "amax": float(accel.max()) if len(accel) else 0.0,
               "path_len": plen, "dev_mm": dev_mm(np.array(yz)), "a_rms": a_rms, "j_rms": j_rms}
        out["modes"][m] = {"yz": yz, "metrics": mtr}
        print(f"{m:22s} {mtr['dur']:6.2f} {mtr['vmax']:8.3f} {mtr['amax']:9.3f} {plen:8.4f} "
              f"{mtr['dev_mm']:8.1f} {a_rms:7.2f} {j_rms:7.1f}")
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f)
    print(f"-> {args.out}")
    return 0


def do_plot(args: argparse.Namespace) -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager as fm
    for p in glob.glob("/usr/share/fonts/**/NotoSansCJK*", recursive=True):
        try:
            fm.fontManager.addfont(p)
        except Exception:
            pass
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "Noto Sans CJK JP", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    with open(args.inp, "r", encoding="utf-8") as f:
        d = json.load(f)

    ys = [p[0] for r in d["modes"].values() for p in r["yz"]]
    zs = [p[1] for r in d["modes"].values() for p in r["yz"]]
    ymin, ymax, zmin, zmax = min(ys), max(ys), min(zs), max(zs)

    fig, ax = plt.subplots(figsize=(10.5, 7.0))
    for m, rec in d["modes"].items():
        yz = rec["yz"]
        ax.plot([p[0] for p in yz], [p[1] for p in yz], color=COLORS[m], lw=1.9,
                label=f"{LABELS[m]}  {rec['metrics']['dur']:.1f}s")
    tg = d["targets"]
    ax.plot([p[0] for p in tg], [p[1] for p in tg], "o", color="#333", ms=6, zorder=5)
    for i, p in enumerate(tg):
        ax.annotate(f"P{i}", (p[0], p[1]), textcoords="offset points", xytext=(6, -13),
                    fontsize=10, color="#111")
    ax.set_xlabel("TCP y (m)", fontsize=12)
    ax.set_ylabel("TCP z (m)", fontsize=12)
    ax.set_title("自研(软件)模式 TCP 空间轨迹对比（y-z 平面，x≈const；口风琴 5 点，仿真实采）",
                 fontsize=13, pad=12)
    ax.grid(True, alpha=0.3)
    # 独立坐标（纵向放大 z）以便看清 4 条曲线的空间分布；标注真实量程。
    ax.text(0.015, 0.97,
            f"y ∈ [{ymin:.3f}, {ymax:.3f}] m（跨度 {(ymax-ymin)*1000:.0f} mm）\n"
            f"z ∈ [{zmin:.4f}, {zmax:.4f}] m（跨度仅 {(zmax-zmin)*1000:.1f} mm，纵向已放大）",
            transform=ax.transAxes, va="top", ha="left", fontsize=10, color="#444",
            bbox=dict(boxstyle="round", fc="white", ec="#ccc", alpha=0.85))
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2, fontsize=11, frameon=True)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    os.makedirs(args.out_dir, exist_ok=True)
    png = os.path.join(args.out_dir, "F167_softmodes_tcp_yz.png")
    svg = os.path.join(args.out_dir, "F167_softmodes_tcp_yz.svg")
    fig.savefig(png, dpi=140)
    fig.savefig(svg)
    print(f"png -> {png}\nsvg -> {svg}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("dump")
    d.add_argument("--run", default="/tmp/traj_modes_tcp.json")
    d.add_argument("--waypoints", default=os.path.join(WS, "trajectories", "waypoints", "latest.yaml"))
    d.add_argument("--out", default="/tmp/f167_tcp_yz.json")
    d.set_defaults(fn=do_dump)
    p = sub.add_parser("plot")
    p.add_argument("--in", dest="inp", default="/tmp/f167_tcp_yz.json")
    p.add_argument("--out-dir", default=os.path.join(WS, "docs", "dev"))
    p.set_defaults(fn=do_plot)
    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
