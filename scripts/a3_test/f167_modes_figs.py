#!/usr/bin/env python3
"""F167 实验：9 模式在「首末精确、中间不要求精确」路点上执行 →
  * yz : TCP 空间轨迹（y-z 平面，x≈const）——「位置-位置」
  * pv : 主导关节 位置-速度 相图——「位置-速度」
  * pa : 主导关节 位置-加速度 相图——「位置-加速度」

因 numpy/matplotlib 环境隔离（LL-152），分两步：
  # 1) FK + 关节序列（ROS 环境，需 pinocchio）
  source scripts/a3_shell_env.sh
  python3 scripts/a3_test/f167_modes_figs.py dump --run /tmp/traj_modes_tcp_3pt.json \
      --waypoints trajectories/waypoints/three_pts.yaml --out /tmp/f167_modes3.json
  # 2) 出图（默认环境，**不要** source a3_shell_env.sh）
  python3 scripts/a3_test/f167_modes_figs.py plot --in /tmp/f167_modes3.json --out-dir docs/dev --tag modes3
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
MODES = ["ptp", "ptp_blend", "lin_exact", "lin_blend", "jnt_g64_quintic",
         "jnt_g64_trapezoid", "jnt_spline_bell", "circ", "jnt_spline_approx"]
COLORS = {"ptp": "#7f7f7f", "ptp_blend": "#8c564b", "lin_exact": "#1f77b4",
          "lin_blend": "#ff7f0e", "circ": "#17becf", "jnt_g64_quintic": "#2ca02c",
          "jnt_g64_trapezoid": "#d62728", "jnt_spline_bell": "#9467bd",
          "jnt_spline_approx": "#e377c2"}
JN = ["L1", "L2", "L3", "L4", "L5", "L6"]


def _extract(samples: List[List[float]], n_total: int) -> Tuple:
    """模式段（掉末尾回起点段 + 去尾静止）→ 均匀 0.02s 重采样 (t,q6,v6,a6)。"""
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
    tu = np.arange(0.0, max(t[-1], 0.04), 0.02)
    qu = np.column_stack([np.interp(tu, t, q[:, k]) for k in range(6)])
    v = np.gradient(qu, tu, axis=0); a = np.gradient(v, tu, axis=0)
    return tu, qu, v, a


def do_dump(args: argparse.Namespace) -> int:
    import numpy as np
    import yaml
    from traj_modes_tcp_report import load_pin, process_mode

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
    ser: Dict[str, Tuple] = {}
    for m in MODES:
        rec = data["modes"].get(m)
        if rec and rec.get("samples"):
            ser[m] = _extract(rec["samples"], int(rec.get("n_total", 2)))
    rng = np.zeros(6)
    for (tu, qu, _, _) in ser.values():
        rng = np.maximum(rng, qu.max(axis=0) - qu.min(axis=0))
    j = int(np.argmax(rng))

    out: Dict = {"joint_index": j, "n_points": len(wp["waypoints"]), "targets": targets,
                 "ref_q": [float(v) for v in wp["waypoints"][0]["positions"][:6]],
                 "ref_yz": [float(targets[0][0]), float(targets[0][1])],
                 "modes": {}, "phase": {}}
    print(f"{'模式':18s} {'时长s':>6s} {'TCP峰速':>7s} {'TCP峰加速':>8s} {'路径长m':>8s} {'距折线mm':>8s} "
          f"{'j_rms':>6s}")
    for m, (tu, qu, v, a) in ser.items():
        rec = data["modes"][m]
        t, xyz, speed, accel = process_mode(pin, model, q_idx, fid, rec["samples"], int(rec.get("n_total", 2)))
        yz = [[float(p[1]), float(p[2])] for p in xyz]
        plen = float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))) if len(xyz) > 1 else 0.0
        jr = float(np.sqrt(np.mean(np.gradient(a, tu, axis=0) ** 2)))
        out["modes"][m] = {"yz": yz, "metrics": {
            "dur": float(tu[-1]), "vmax": float(speed.max()) if len(speed) else 0.0,
            "amax": float(accel.max()) if len(accel) else 0.0, "path_len": plen,
            "dev_mm": dev_mm(np.array(yz)), "j_rms": jr}}
        out["phase"][m] = {"q": [float(x) for x in qu[:, j]], "v": [float(x) for x in v[:, j]],
                           "a": [float(x) for x in a[:, j]]}
        mm = out["modes"][m]["metrics"]
        print(f"{m:18s} {mm['dur']:6.2f} {mm['vmax']:7.3f} {mm['amax']:8.3f} {plen:8.4f} "
              f"{mm['dev_mm']:8.2f} {jr:6.2f}")
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f)
    print(f"dominant joint = {JN[j]}；-> {args.out}")
    return 0


def _mpl_setup():
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
    return plt


def do_plot(args: argparse.Namespace) -> int:
    plt = _mpl_setup()
    with open(args.inp, "r", encoding="utf-8") as f:
        d = json.load(f)
    j = d["joint_index"]; jn = JN[j]
    tg = d["targets"]
    al = bool(getattr(args, "align", False))
    tail = "（起点已对齐到 P0）" if al else ""
    order = [m for m in MODES if m in d["modes"]] or list(d["modes"])
    os.makedirs(args.out_dir, exist_ok=True)

    def legend(ax, **kw):
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=3, fontsize=9, frameon=True, **kw)

    # 图1：位置-位置（TCP y-z）
    fig, ax = plt.subplots(figsize=(10.5, 7.2))
    ry, rz = d.get("ref_yz", [tg[0][0], tg[0][1]])
    ys = [p[0] for m in order for p in d["modes"][m]["yz"]]
    zs = [p[1] for m in order for p in d["modes"][m]["yz"]]
    for m in order:
        yz = d["modes"][m]["yz"]
        dy = (ry - yz[0][0]) if al else 0.0
        dz = (rz - yz[0][1]) if al else 0.0
        ax.plot([p[0] + dy for p in yz], [p[1] + dz for p in yz], color=COLORS.get(m, "#333"),
                lw=1.7, label=f"{m}  {d['modes'][m]['metrics']['dur']:.1f}s")
    ax.plot([p[0] for p in tg], [p[1] for p in tg], "o", color="#111", ms=7, zorder=6)
    for i, p in enumerate(tg):
        ax.annotate(f"P{i}", (p[0], p[1]), textcoords="offset points", xytext=(7, -13), fontsize=10)
    ax.set_xlabel("TCP y (m)", fontsize=12); ax.set_ylabel("TCP z (m)", fontsize=12)
    ax.set_title(f"位置-位置：9 模式 TCP 空间轨迹（y-z，x≈const；3 点）{tail}",
                 fontsize=13, pad=12)
    ax.grid(True, alpha=0.3)
    ax.text(0.015, 0.97, f"y∈[{min(ys):.3f},{max(ys):.3f}] m，z∈[{min(zs):.4f},{max(zs):.4f}] m"
            f"（z 跨度 {(max(zs)-min(zs))*1000:.1f} mm，纵向已放大）",
            transform=ax.transAxes, va="top", fontsize=9, color="#444",
            bbox=dict(boxstyle="round", fc="white", ec="#ccc", alpha=0.85))
    legend(ax)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(os.path.join(args.out_dir, f"F167_{args.tag}_yy.png"), dpi=140)
    fig.savefig(os.path.join(args.out_dir, f"F167_{args.tag}_yy.svg")); plt.close(fig)

    # 图2/3：位置-速度、位置-加速度（主导关节相图）
    for key, ylab, fname in [("v", "速度 (rad/s)", f"F167_{args.tag}_pv.png"),
                             ("a", "加速度 (rad/s²)", f"F167_{args.tag}_pa.png")]:
        fig, ax = plt.subplots(figsize=(10.5, 7.2))
        for m in order:
            ph = d["phase"][m]
            dq = (d.get("ref_q", [0.0] * 6)[j] - ph["q"][0]) if al else 0.0
            ax.plot([q + dq for q in ph["q"]], ph[key], color=COLORS.get(m, "#333"), lw=1.7,
                    label=f"{m}  {d['modes'][m]['metrics']['dur']:.1f}s")
        ax.set_xlabel(f"关节 {jn} 位置 (rad)", fontsize=12)
        ax.set_ylabel(f"关节 {jn} {ylab}", fontsize=12)
        ax.set_title(f"位置-{'速度' if key=='v' else '加速度'}：9 模式关节 {jn} 相图（3 点）{tail}",
                     fontsize=13, pad=12)
        ax.grid(True, alpha=0.3); ax.axhline(0.0, color="#999", lw=0.8)
        legend(ax)
        fig.tight_layout(rect=(0, 0.05, 1, 1))
        fig.savefig(os.path.join(args.out_dir, fname), dpi=140)
        fig.savefig(os.path.join(args.out_dir, fname[:-4] + ".svg")); plt.close(fig)
    print(f"figs -> {args.out_dir}/F167_{args.tag}_{{yy,pv,pa}}.png|svg")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("dump")
    d.add_argument("--run", default="/tmp/traj_modes_tcp_3pt.json")
    d.add_argument("--waypoints", default=os.path.join(WS, "trajectories", "waypoints", "three_pts.yaml"))
    d.add_argument("--out", default="/tmp/f167_modes3.json")
    d.set_defaults(fn=do_dump)
    p = sub.add_parser("plot")
    p.add_argument("--in", dest="inp", default="/tmp/f167_modes3.json")
    p.add_argument("--out-dir", default=os.path.join(WS, "docs", "dev"))
    p.add_argument("--tag", default="modes3")
    p.add_argument("--align", action="store_true",
                   help="把各模式起点平移到 P0（消除仿真落点不一致，便于比较路径形状）")
    p.set_defaults(fn=do_plot)
    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
