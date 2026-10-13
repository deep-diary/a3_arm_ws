#!/usr/bin/env python3
"""多路点轨迹模式基准报告（合并离线命令指标 + 在线跟踪误差）。

读 traj_modes_run.py 的 JSON（在线跟踪）+ 重算离线命令指标，出 MD + SVG 汇总。

用法（先 source a3_shell_env.sh）：
  python3 scripts/a3_test/traj_modes_report.py \
      --config src/a3_arm_controller/config/traj_modes_bench.yaml \
      --run /tmp/traj_modes_run.json [--out-dir docs/dev]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Optional

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, WS)
sys.path.insert(0, os.path.join(WS, "src", "a3_arm_controller", "a3_arm_controller"))
sys.path.insert(0, os.path.join(WS, "scripts"))

import yaml  # noqa: E402
from traj_smoothness_calibration import compute_metrics  # noqa: E402
from traj_modes_bench import (  # noqa: E402
    COLORS, MODE_LABELS, gen_offline, load_named_pose_joints,
)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--run", default=None, help="traj_modes_run.py 的 JSON（可选）")
    ap.add_argument("--named-poses", default=None)
    ap.add_argument("--out-dir", default=None)
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
    vmax = float(cfg.get("vmax_rad_s", 1.5))
    amax = float(cfg.get("amax_rad_s2", 15.0))
    modes = cfg["modes"]
    speeds = [float(s) for s in cfg["speeds"]]

    run_map: Dict = {}
    if args.run and os.path.exists(args.run):
        with open(args.run, "r", encoding="utf-8") as f:
            for r in json.load(f):
                run_map[(r["mode"], r["speed"])] = r

    rows: List[Dict] = []
    for mode in modes:
        for speed in speeds:
            v, a = vmax * speed, amax * speed
            row = {"mode": mode, "speed": speed, "dur": None, "j_rms": None,
                   "a_rms": None, "peak_err": None, "rms_err": None}
            if mode in ("l2", "l3", "l1p5"):
                timed = gen_offline(mode, points, v, a)
                if timed is not None:
                    times, pos = timed
                    m = compute_metrics(times, pos)
                    row["dur"] = float(times[-1])
                    row["j_rms"] = m["j_rms"]
                    row["a_rms"] = m["a_rms"]
            if (mode, speed) in run_map:
                r = run_map[(mode, speed)]
                if r.get("peak_err") is not None:
                    row["peak_err"] = r["peak_err"]
                    row["rms_err"] = r["rms_err"]
            rows.append(row)

    out_dir = args.out_dir or os.path.join(WS, "docs", "dev")
    os.makedirs(out_dir, exist_ok=True)

    lines = ["# 多路点轨迹模式基准报告", ""]
    lines.append(f"- 测试点：{', '.join(cfg['points'])}（{len(points)} 点）")
    lines.append(f"- 模式：{', '.join(MODE_LABELS[m] for m in modes)} × speed {speeds[0]}~{speeds[-1]}")
    lines.append("")
    lines.append("| mode | speed | 时长 s | j_rms | a_rms | peak_err | rms_err |")
    lines.append("|---|---|---|---|---|---|---|")
    for row in rows:
        f = lambda x: f"{x:.4f}" if x is not None else "-"
        lines.append(f"| {MODE_LABELS[row['mode']]} | {row['speed']:.1f} | "
                     f"{f(row['dur'])} | {f(row['j_rms'])} | {f(row['a_rms'])} | "
                     f"{f(row['peak_err'])} | {f(row['rms_err'])} |")
    lines.append("")
    lines.append("> 统一标准：时长（越快越好）+ j_rms/a_rms（越平滑越好，L3 最优）+ "
                 "peak_err/rms_err（跟踪越稳越好）。")
    lines.append("> ptp/L1 的离线 j_rms/a_rms 依赖 move_group（在线采）；l1 需 stream mode=lin。")
    lines.append("")

    md = os.path.join(out_dir, "traj_modes_report.md")
    with open(md, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"报告 -> {md}")


if __name__ == "__main__":
    main()
