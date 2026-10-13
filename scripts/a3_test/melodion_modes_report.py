#!/usr/bin/env python3
"""口风琴 32 键 9 模式对比：分析 + 报告 + 图。

两步（numpy/matplotlib 环境隔离，LL-152）：
  # 1) FK + 指标（ROS 环境，需 pinocchio）
  source scripts/a3_shell_env.sh
  python3 scripts/a3_test/melodion_modes_report.py dump --run /tmp/melodion_modes.json --out /tmp/melodion_modes_m.json
  # 2) 出图（默认环境，**不要** source a3_shell_env.sh）
  python3 scripts/a3_test/melodion_modes_report.py plot --in /tmp/melodion_modes_m.json --out-dir docs/dev
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
sys.path.insert(0, os.path.join(WS, "scripts", "a3_test"))
MODES = ["ptp", "ptp_blend", "lin_exact", "lin_blend", "jnt_g64_quintic",
         "jnt_g64_trapezoid", "jnt_spline_bell", "circ", "jnt_spline_approx"]
JN = ["L1", "L2", "L3", "L4", "L5", "L6"]


def _groups(samples: List[List[float]]):
    """按段号把样本分组，返回 [(seg, rows...)]（时间序）。"""
    rows = sorted(samples, key=lambda r: r[0])
    out, cur, cur_seg = [], [], None
    for r in rows:
        s = int(r[8])
        if cur_seg is None or s == cur_seg:
            cur.append(r); cur_seg = s
        else:
            out.append((cur_seg, cur)); cur, cur_seg = [r], s
    if cur:
        out.append((cur_seg, cur))
    return out


def do_dump(args: argparse.Namespace) -> int:
    import yaml
    from traj_modes_tcp_report import load_pin, fk_tcp
    pin, model, q_idx, fid = load_pin()
    with open(args.run, "r", encoding="utf-8") as f:
        data = json.load(f)
    # 32 个键（下压点）的 yz：取基准流前 32 段 target（relative 先展开为绝对）
    with open(args.base, "r", encoding="utf-8") as f:
        base = yaml.safe_load(f)
    if str(base.get("frame", "absolute")).lower() == "relative":
        a = (base.get("anchor") or {}).get("position") or {}
        ax, ay, az = float(a["x"]), float(a["y"]), float(a["z"])
        for s in base["segments"]:
            for h in [s.get("target", {})] + (s.get("via") or []):
                p = (h.get("pose") or {}).get("position")
                if p:
                    p["x"] += ax; p["y"] += ay; p["z"] += az
    keys = []
    for s in base["segments"][:32]:
        p = s["target"]["pose"]["position"]
        keys.append([float(p["y"]), float(p["z"])])
    keys = np.array(keys)
    print(f"键位: {len(keys)} 个（y {keys[:,0].min():.3f}~{keys[:,0].max():.3f} m）")

    res: Dict[str, dict] = {}
    print(f"{'模式':18s} {'ok':>3s} {'段数':>4s} {'总时长s':>7s} {'运动s':>6s} "
          f"{'够键μmm':>8s} {'够键max':>8s} {'到键':>6s} {'按深μ':>7s} {'最欠按':>7s} "
          f"{'j_rms':>6s} {'TCP峰速':>7s} {'TCP峰加速':>8s}")
    for m in MODES:
        rec = data["modes"].get(m)
        if not rec:
            continue
        ok = bool(rec.get("ok"))
        if not rec.get("samples"):
            res[m] = {"ok": ok, "n_seg": 0}
            print(f"{m:18s} {str(ok):>3s}    0   (无样本)")
            continue
        gs = _groups(rec["samples"])
        t = np.array([r[0] for r in rec["samples"]], float); t -= t[0]
        tcp = np.array([fk_tcp(pin, model, q_idx, fid, r[1:7]) for r in rec["samples"]])
        # 够键：每个键 → 轨迹到该键的最小距离(mm)
        hits = [float(np.min(np.linalg.norm(tcp[:, 1:3] - k, axis=1))) * 1000.0 for k in keys]
        # 按深：键 y±3mm 窗口内，轨迹最低 z 相对键面 z 的差（>0 = 按到/过压；<0 = 欠按）
        press = []
        for k in keys:
            msk = np.abs(tcp[:, 1] - k[0]) < 0.003
            press.append((k[1] - float(np.min(tcp[msk, 2]))) * 1000.0 if msk.any() else -999.0)
        press = np.array(press)
        vp = press > -900
        seg_durs = [rows[-1][0] - rows[0][0] for _, rows in gs]
        q = np.array([[r[1 + k] for k in range(6)] for r in rec["samples"]])
        tu = np.arange(0.0, max(t[-1], 0.04), 0.02)
        qu = np.column_stack([np.interp(tu, t, q[:, k]) for k in range(6)])
        a = np.gradient(np.gradient(qu, tu, axis=0), tu, axis=0)
        j = np.gradient(a, tu, axis=0)
        xu = np.column_stack([np.interp(tu, t, tcp[:, k]) for k in range(3)])
        v = np.gradient(xu, tu, axis=0)
        res[m] = {
            "ok": ok, "n_seg": len(gs), "n_expected": rec.get("n_total"),
            "total_s": float(t[-1]), "motion_s": float(np.sum(seg_durs)),
            "hit_mean_mm": float(np.mean(hits)), "hit_max_mm": float(np.max(hits)),
            "press_mean_mm": float(np.mean(press[vp])) if vp.any() else None,
            "press_worst_mm": float(np.min(press[vp])) if vp.any() else None,
            "n_keys_reached": int(np.sum(press[vp] > -1.0)),
            "j_rms": float(np.sqrt(np.mean(j ** 2))),
            "tcp_vmax": float(np.max(np.linalg.norm(v, axis=1))),
            "tcp_amax": float(np.max(np.linalg.norm(np.gradient(v, tu, axis=0), axis=1))),
        }
        rr = res[m]
        g = lambda k, p=2: (f"{rr[k]:.{p}f}" if isinstance(rr.get(k), (int, float)) else "-")
        print(f"{m:18s} {str(ok):>3s} {rr['n_seg']:>4d} {rr['total_s']:7.1f} {rr['motion_s']:6.1f} "
              f"{g('hit_mean_mm'):>8s} {g('hit_max_mm'):>8s} {rr['n_keys_reached']:>3d}/32 "
              f"{g('press_mean_mm'):>7s} {g('press_worst_mm'):>7s} {rr['j_rms']:6.2f} "
              f"{rr['tcp_vmax']:7.3f} {rr['tcp_amax']:8.3f}")

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"speed": data.get("speed"), "modes": res}, f, indent=1)
    print(f"-> {args.out}")

    md = [f"# 口风琴 32 键 · 9 模式对比（speed={data.get('speed')}，仿真实采）", "",
          "轨迹：`scale.yaml`（32 键 F3→C6 逐键；每链 3 点 = 键→抬升→键）。"
          "`ptp` 展开为无 via 逐腿段；`circ`/`lin_*` 用键位 position-only IK 的真实姿态。", "",
          "「够键」= 轨迹到该键下压点的最小 TCP 距离；「按深」= 键 y±3mm 内轨迹最低 z 相对键面"
          "（>0 过压、≈0 到位、<0 欠按）；「到键」= 按深 > -1mm 的键数。", "",
          "| 模式 | ok | 段数 | 总时长(s) | 运动时长(s) | 够键均值(mm) | 够键最大(mm) | 到键 | 平均按深(mm) | 最欠按(mm) | j_rms | TCP峰速(m/s) | TCP峰加速(m/s²) |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for m in MODES:
        rr = res.get(m)
        if not rr:
            continue
        f2 = lambda k, p=2: (f"{rr[k]:.{p}f}" if isinstance(rr.get(k), (int, float)) else "-")
        md.append(f"| `{m}` | {'✅' if rr.get('ok') else '❌'} | {rr.get('n_seg','-')} | {f2('total_s',1)} | "
                  f"{f2('motion_s',1)} | {f2('hit_mean_mm')} | {f2('hit_max_mm')} | {rr.get('n_keys_reached','-')}/32 | "
                  f"{f2('press_mean_mm')} | {f2('press_worst_mm')} | {f2('j_rms')} | "
                  f"{f2('tcp_vmax',3)} | {f2('tcp_amax',3)} |")
    os.makedirs(os.path.join(WS, "docs", "dev"), exist_ok=True)
    md_path = os.path.join(WS, "docs", "dev", f"MELODION_MODES_{args.tag}.md")
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    print(f"md -> {md_path}")
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
    ms = d["modes"]
    okm = [m for m in MODES if ms.get(m) and ms[m].get("ok") and ms[m].get("total_s")]
    fail = [m for m in MODES if ms.get(m) and not ms[m].get("ok")]

    fig, axes = plt.subplots(1, 2, figsize=(14.5, 6.0))
    x = np.arange(len(okm))
    cols = ["#2ca02c" if "g64" in m else "#d62728" if "trapezoid" in m else
            "#9467bd" if "bell" in m else "#e377c2" if "approx" in m else
            "#1f77b4" if m.startswith("lin") else "#17becf" if m == "circ" else "#7f7f7f"
            for m in okm]
    axes[0].bar(x, [ms[m]["total_s"] for m in okm], color=cols)
    axes[0].set_xticks(x); axes[0].set_xticklabels(okm, rotation=25, ha="right", fontsize=9)
    axes[0].set_ylabel("总时长 (s)", fontsize=12)
    axes[0].set_title(f"口风琴 32 键 · 总时长（speed={d.get('speed')}）", fontsize=13)
    axes[0].grid(True, axis="y", alpha=0.3)
    for i, m in enumerate(okm):
        axes[0].text(i, ms[m]["total_s"], f"{ms[m]['total_s']:.0f}", ha="center", va="bottom", fontsize=8)

    axes[1].bar(x, [ms[m].get("n_keys_reached") or 0 for m in okm], color=cols)
    axes[1].set_xticks(x); axes[1].set_xticklabels(okm, rotation=25, ha="right", fontsize=9)
    axes[1].set_ylabel("按到键数 / 32", fontsize=12)
    axes[1].set_ylim(0, 33)
    axes[1].set_title("按键命中（按深 > -1mm 的键数）", fontsize=13)
    axes[1].grid(True, axis="y", alpha=0.3)
    for i, m in enumerate(okm):
        axes[1].text(i, ms[m].get("n_keys_reached") or 0, f"{ms[m].get('n_keys_reached') or 0}",
                     ha="center", va="bottom", fontsize=8)
    if fail:
        fig.text(0.5, 0.01, f"未成功执行：{', '.join(fail)}", ha="center", fontsize=10, color="#b00")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    os.makedirs(args.out_dir, exist_ok=True)
    png = os.path.join(args.out_dir, f"MELODION_MODES_{args.tag}.png")
    svg = os.path.join(args.out_dir, f"MELODION_MODES_{args.tag}.svg")
    fig.savefig(png, dpi=140); fig.savefig(svg)
    print(f"png -> {png}\nsvg -> {svg}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("dump")
    d.add_argument("--run", default="/tmp/melodion_modes.json")
    d.add_argument("--base", default=os.path.join(WS, "src", "app_melodion", "config", "streams", "inspection.yaml"))
    d.add_argument("--tag", default="s04")
    d.add_argument("--out", default="/tmp/melodion_modes_m.json")
    d.set_defaults(fn=do_dump)
    p = sub.add_parser("plot")
    p.add_argument("--in", dest="inp", default="/tmp/melodion_modes_m.json")
    p.add_argument("--tag", default="s04")
    p.add_argument("--out-dir", default=os.path.join(WS, "docs", "dev"))
    p.set_defaults(fn=do_plot)
    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
