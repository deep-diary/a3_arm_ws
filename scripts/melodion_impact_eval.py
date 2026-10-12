#!/usr/bin/env python3
"""口风琴跨键轨迹「末端笛卡尔冲击 + 时长」评估（approach 10mm vs 20mm + 速度档）。

笛卡尔空间评估（不经关节 IK，避免 position-only IK 姿态自由导致不可比）：
把 down→via→down 三点用过点样条插值成 C2 路径，再 quintic 时间重参数化，算末端
加速度/加加速度 RMS（冲击指标）与时长。

用法（先 source a3_shell_env.sh）：
  python3 scripts/melodion_impact_eval.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src", "a3_arm_controller", "a3_arm_controller"))

from arm_controller import (  # noqa: E402
    _finite_diff_metrics,
    _interpolate_via_spline,
    _jerk_limited_timed_traj,
)

# 键面高度（base.z=0.26）、press=5mm、approach 10/20mm
Z_BASE = 0.26
Z_DOWN = Z_BASE - 0.005
# 相邻键（scale 全扫真实间距）：白→黑 6.4mm（F3→F#3）、黑→白 12.5mm（F#3→G3）、白→白 18.9mm
PAIRS = [
    ("F3→F#3 (6.4mm)", -0.1700, -0.1636),
    ("F#3→G3 (12.5mm)", -0.1636, -0.1511),
    ("F3→G3 (18.9mm)", -0.1700, -0.1511),
]


def main() -> None:
    print("末端笛卡尔跨键冲击/时长评估（x 恒 -0.20，z: down=0.255 → via → down）")
    print(f"{'跨键':>18} | {'approach':>8} | {'vmax m/s':>8} | {'时长s':>7} | {'a_rms m/s²':>10} | {'j_rms m/s³':>10}")
    for name, y0, y1 in PAIRS:
        p_from = [-0.20, y0, Z_DOWN]
        p_to = [-0.20, y1, Z_DOWN]
        for appr_mm, via_z in [(10, Z_BASE + 0.010), (20, Z_BASE + 0.020)]:
            via = [-0.20, (y0 + y1) / 2.0, via_z]
            dense = _interpolate_via_spline([p_from, via, p_to], 200)
            if dense is None:
                continue
            for vmax in (0.05, 0.1, 0.2):
                amax = vmax * 10.0
                timed = _jerk_limited_timed_traj(dense, vmax, amax)
                if timed is None:
                    continue
                times, pos = timed
                m = _finite_diff_metrics(times, pos)
                a_rms, j_rms = (m[0], m[1]) if m else (float("nan"), float("nan"))
                print(f"{name:>18} | {appr_mm:>6}mm | {vmax:>8} | {times[-1]:>7.2f} | {a_rms:>10.4f} | {j_rms:>10.4f}")


if __name__ == "__main__":
    main()
