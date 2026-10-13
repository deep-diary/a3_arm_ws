#!/usr/bin/env python3
"""F167.1 A/B：软件模式时间律 software（手写剖面）vs totg（/a3/arm/retime_trajectory）。

对 4 个软件模式（jnt_g64_quintic / jnt_g64_trapezoid / jnt_spline_bell /
jnt_spline_approx）比较两次在线采集（`goto_via_time_backend`=software / totg）：
几何路径应一致（同路径，仅时间律不同），时间律指标（时长 / j_rms / a_rms）允许差异。

前置：仿真栈已起、已 enable；两次采集用 traj_modes_tcp_capture.py 分别产出 JSON。
  1) goto_via_time_backend=software（默认）→ capture → software.json
  2) 改 yaml 为 totg、重启仿真 → capture → totg.json
用法（先 source a3_shell_env.sh，export ROS_DOMAIN_ID=99 仅采集需要）：
  python3 scripts/a3_test/f167_retime_ab.py \
      --software /tmp/f167_ab_software.json --totg /tmp/f167_ab_totg.json \
      --out-md docs/dev/F167_retime_ab.md
退出码：0=PASS（几何一致、两轮均真实执行），1=FAIL。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "scripts", "a3_test"))

from traj_modes_tcp_report import load_pin, fk_tcp, process_mode  # noqa: E402

SOFT_MODES = ["jnt_g64_quintic", "jnt_g64_trapezoid", "jnt_spline_bell", "jnt_spline_approx"]
GEO_TOL = 0.05  # 几何路径长相对差容差（5%）


def joint_metrics(samples: List[List[float]], n_total: int) -> Tuple[float, float, float]:
    """模式段的关节空间 (dur, a_rms, j_rms)：剔除末尾回起点段 + 尾部静止样本。"""
    rows = [s for s in samples if int(s[8]) < n_total]
    if len(rows) < 5:
        rows = samples
    rows.sort(key=lambda r: r[0])
    t = np.array([r[0] for r in rows], dtype=float)
    t = t - t[0]
    q = np.array([[r[1 + i] for i in range(6)] for r in rows], dtype=float)
    if len(t) > 3:
        keep = np.concatenate([[True], np.diff(t) > 1e-6])
        t, q = t[keep], q[keep]
        dq = np.max(np.abs(np.diff(q, axis=0)), axis=1)
        if np.any(dq > 1e-5):
            last = int(np.max(np.where(dq > 1e-5)[0])) + 1
            t, q = t[:last + 1], q[:last + 1]
    if len(t) < 5:
        return (float(t[-1]) if len(t) else 0.0), 0.0, 0.0
    tu = np.arange(t[0], t[-1], 0.02)
    qu = np.column_stack([np.interp(tu, t, q[:, k]) for k in range(6)])
    v = np.gradient(qu, tu, axis=0)
    a = np.gradient(v, tu, axis=0)
    j = np.gradient(a, tu, axis=0)
    return float(tu[-1]), float(np.sqrt(np.mean(a ** 2))), float(np.sqrt(np.mean(j ** 2)))


def collect(run_path: str, pin, model, q_idx, fid) -> Dict[str, Dict]:
    with open(run_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    out: Dict[str, Dict] = {}
    for mode in SOFT_MODES:
        rec = data["modes"].get(mode)
        if not rec or not rec.get("samples"):
            out[mode] = {"ok": False, "n": 0}
            continue
        n_total = int(rec.get("n_total", 2))
        dur, a_rms, j_rms = joint_metrics(rec["samples"], n_total)
        t, xyz, speed, accel = process_mode(pin, model, q_idx, fid, rec["samples"], n_total)
        path_len = float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))) if len(xyz) > 1 else 0.0
        out[mode] = {"ok": bool(rec.get("ok")), "n": len(rec["samples"]),
                     "dur": dur, "j_rms": j_rms, "a_rms": a_rms,
                     "vmax": float(speed.max()) if len(speed) else 0.0,
                     "path_len": path_len}
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="F167.1 software vs totg 时间律 A/B")
    ap.add_argument("--software", required=True)
    ap.add_argument("--totg", required=True)
    ap.add_argument("--out-md", default=os.path.join(WS, "docs", "dev", "F167_retime_ab.md"))
    args = ap.parse_args()

    pin, model, q_idx, fid = load_pin()
    sw = collect(args.software, pin, model, q_idx, fid)
    tg = collect(args.totg, pin, model, q_idx, fid)

    lines = ["# F167.1 A/B：软件模式时间律 software vs totg（仿真实采）", "",
             "同一组软件模式、同一 5 点路径，仅 `goto_via_time_backend` 不同；",
             "几何路径（路径长）应一致，时间律（时长 / j_rms）允许差异。", "",
             "| 模式 | backend | 时长(s) | j_rms(rad/s³) | a_rms(rad/s²) | TCP路径长(m) | 几何相对差 |",
             "|---|---|---|---|---|---|---|"]
    ok_all = True
    for mode in SOFT_MODES:
        s, g = sw[mode], tg[mode]
        if not s.get("ok") or not g.get("ok"):
            ok_all = False
            lines.append(f"| {mode} | - | ⚠ 缺数据 (sw ok={s.get('ok')}, totg ok={g.get('ok')}) | "
                         "| | | |")
            continue
        geo_rel = abs(g["path_len"] - s["path_len"]) / max(s["path_len"], 1e-9)
        geo_ok = geo_rel <= GEO_TOL
        ok_all = ok_all and geo_ok
        lines.append(f"| {mode} | software | {s['dur']:.2f} | {s['j_rms']:.1f} | "
                     f"{s['a_rms']:.1f} | {s['path_len']:.4f} | - |")
        lines.append(f"| {mode} | totg | {g['dur']:.2f} | {g['j_rms']:.1f} | "
                     f"{g['a_rms']:.1f} | {g['path_len']:.4f} | "
                     f"{geo_rel*100:.2f}% {'✓' if geo_ok else '✗'} |")
    lines += ["", f"- 几何一致判定（路径长相对差 ≤ {GEO_TOL*100:.0f}%）："
                  f"{'PASS ✓' if ok_all else 'FAIL ✗'}"]
    os.makedirs(os.path.dirname(args.out_md), exist_ok=True)
    with open(args.out_md, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"\n-> {args.out_md}")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
