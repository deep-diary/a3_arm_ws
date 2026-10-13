#!/usr/bin/env python3
"""F167 附图：4 个自研(软件)模式的关节 位置/速度/加速度 对比（matplotlib 出图）。

输入 traj_modes_tcp_capture.py 的 JSON；取 jnt_g64_quintic / jnt_g64_trapezoid /
jnt_spline_bell / jnt_spline_approx 的模式段实测关节序列，选运动幅度最大的关节，
出 3 联子图（位置 rad / 速度 rad/s / 加速度 rad/s²）→ PNG + SVG（全尺寸，不截断）。

用法：
  python3 scripts/a3_test/f167_softmodes_curves.py --run /tmp/traj_modes_tcp.json \
      --out-dir docs/dev

注意：本脚本只用 numpy + matplotlib，**请勿 source scripts/a3_shell_env.sh**——
它会强制系统 numpy 1.21 与旧 pyparsing，反而让 pip 版 matplotlib 导入失败（LL-152）。
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from typing import Dict, List

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SOFT = ["jnt_g64_quintic", "jnt_g64_trapezoid", "jnt_spline_bell", "jnt_spline_approx"]
COLORS = {"jnt_g64_quintic": "#2ca02c", "jnt_g64_trapezoid": "#d62728",
          "jnt_spline_bell": "#9467bd", "jnt_spline_approx": "#e377c2"}
SHORT = {"jnt_g64_quintic": "jnt_g64_quintic（G64+quintic）",
         "jnt_g64_trapezoid": "jnt_g64_trapezoid（G64+梯形）",
         "jnt_spline_bell": "jnt_spline_bell（样条+钟形）",
         "jnt_spline_approx": "jnt_spline_approx（逼近样条）"}
JN = ["L1", "L2", "L3", "L4", "L5", "L6"]


def _setup_font() -> None:
    """注册中文字体（Noto Sans CJK SC），避免方块。"""
    from matplotlib import font_manager as fm
    for p in glob.glob("/usr/share/fonts/**/NotoSansCJK*", recursive=True):
        try:
            fm.fontManager.addfont(p)
        except Exception:
            pass
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "Noto Sans CJK JP", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def extract(samples: List[List[float]], n_total: int):
    """模式段（掉末尾回起点段 + 去尾静止）→ 均匀 0.02s 重采样 (t,q6,v6,a6)。"""
    rows = [s for s in samples if int(s[8]) < n_total] or samples
    rows.sort(key=lambda r: r[0])
    t = np.array([r[0] for r in rows], dtype=float); t -= t[0]
    q = np.array([[r[1 + i] for i in range(6)] for r in rows], dtype=float)
    keep = np.concatenate([[True], np.diff(t) > 1e-6])
    t, q = t[keep], q[keep]
    if len(t) > 3:
        dq = np.max(np.abs(np.diff(q, axis=0)), axis=1)
        if np.any(dq > 1e-5):
            last = int(np.max(np.where(dq > 1e-5)[0])) + 1
            t, q = t[:last + 1], q[:last + 1]
    tu = np.arange(0.0, t[-1], 0.02)
    qu = np.column_stack([np.interp(tu, t, q[:, k]) for k in range(6)])
    v = np.gradient(qu, tu, axis=0)
    a = np.gradient(v, tu, axis=0)
    return tu, qu, v, a


def main() -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="/tmp/traj_modes_tcp.json")
    ap.add_argument("--out-dir", default=os.path.join(WS, "docs", "dev"))
    args = ap.parse_args()
    _setup_font()

    with open(args.run, "r", encoding="utf-8") as f:
        data = json.load(f)
    got: Dict[str, tuple] = {}
    for m in SOFT:
        rec = data["modes"].get(m)
        if rec and rec.get("samples"):
            got[m] = extract(rec["samples"], int(rec.get("n_total", 2)))
    if not got:
        print("[fail] 无软件模式数据", file=sys.stderr)
        return 1

    rng = np.zeros(6)
    for (tu, qu, _, _) in got.values():
        rng = np.maximum(rng, qu.max(axis=0) - qu.min(axis=0))
    j = int(np.argmax(rng))

    fig, axes = plt.subplots(3, 1, figsize=(10.5, 11.0), sharex=True)
    specs = [("位置 (rad)", 0), ("速度 (rad/s)", 1), ("加速度 (rad/s²)", 2)]
    for ax, (ylab, idx) in zip(axes, specs):
        for m, (tu, qu, v, a) in got.items():
            y = (qu, v, a)[idx][:, j]
            ax.plot(tu, y, color=COLORS[m], lw=1.8, label=SHORT[m])
        ax.set_ylabel(ylab, fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.axhline(0.0, color="#999", lw=0.8)
        # 标注该子图峰值（仅供读数）
        ymax = max(np.max(np.abs((qu, v, a)[idx][:, j])) for (tu, qu, v, a) in got.values())
        ax.set_title(f"峰值 |{'q' if idx==0 else ('v' if idx==1 else 'a')}| ≈ {ymax:.3f}",
                     fontsize=9, loc="right", color="#666")
    axes[0].set_title(
        f"自研(软件)模式对比：关节 {JN[j]}（幅度 {rng[j]:.2f} rad）位置/速度/加速度"
        f"（口风琴 5 点，仿真实采）", fontsize=14, pad=12)
    axes[-1].set_xlabel("时间 t (s)", fontsize=12)
    axes[-1].legend(loc="upper center", bbox_to_anchor=(0.5, -0.18),
                    ncol=2, fontsize=11, frameon=True)
    fig.tight_layout(rect=(0, 0.05, 1, 1))

    os.makedirs(args.out_dir, exist_ok=True)
    png = os.path.join(args.out_dir, "F167_softmodes_curves.png")
    svg = os.path.join(args.out_dir, "F167_softmodes_curves.svg")
    fig.savefig(png, dpi=140)
    fig.savefig(svg)
    print(f"关节 {JN[j]}（幅度 {rng[j]:.2f} rad）")
    print(f"png -> {png}\nsvg -> {svg}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
