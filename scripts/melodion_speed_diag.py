#!/usr/bin/env python3
"""口风琴跨键 via 轨迹的关节空间 nominal 时长（T）诊断：approach / vmax / amax 扫参。

目的：定位「补时长后 total=232.8s 偏慢」的瓶颈，给出 ~1s/键（出厂检测）与
~0.3s/键（弹琴 demo）分别需要的速度/加速度上限，以及 approach 8/10/20mm 对 T 的影响。

用法（先 source a3_shell_env.sh）：
  python3 scripts/melodion_speed_diag.py
"""
from __future__ import annotations

import os
import sys
from typing import List, Optional

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src", "app_melodion"))
sys.path.insert(0, os.path.join(WS, "src", "a3_arm_controller", "a3_arm_controller"))

from app_melodion.key_config import MelodionConfig  # noqa: E402
from arm_controller import _interpolate_via_spline, _jerk_limited_timed_traj  # noqa: E402

JOINTS = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]
SEED = [0.0, 1.05, -1.575, 0.0, 0.0, 0.0]


def load_pin():
    import pinocchio as pin
    from ament_index_python.packages import get_package_share_directory
    urdf = os.path.join(get_package_share_directory("a3_description"), "urdf", "el_a3.urdf")
    model = pin.buildModelFromUrdf(urdf)
    q_idx = [int(model.idx_qs[model.getJointId(jn)]) for jn in JOINTS]
    fid = model.getFrameId("end_effector")
    return pin, model, q_idx, fid


def ik(pin, model, q_idx, fid, xyz) -> Optional[List[float]]:
    from scipy.optimize import least_squares
    lo = [float(model.lowerPositionLimit[qi]) + 0.05 for qi in q_idx]
    hi = [float(model.upperPositionLimit[qi]) - 0.05 for qi in q_idx]
    x0 = [min(max(SEED[i], lo[i]), hi[i]) for i in range(6)]
    tgt = np.array(xyz, dtype=float)

    def residual(q6):
        q = np.zeros(model.nq)
        for v, qi in zip(q6, q_idx):
            q[qi] = float(v)
        d = model.createData()
        pin.forwardKinematics(model, d, q)
        pin.updateFramePlacement(model, d, fid)
        return d.oMf[fid].translation - tgt

    r = least_squares(residual, x0, bounds=(lo, hi),
                      xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=400)
    return [float(v) for v in r.x] if float(np.linalg.norm(r.fun)) < 1e-3 else None


def main() -> None:
    cfg_dir = os.path.join(WS, "src", "app_melodion", "config")
    cfg = MelodionConfig(os.path.join(cfg_dir, "melodion_keys.json"),
                         os.path.join(cfg_dir, "playing_geometry.yaml"))
    pin, model, q_idx, fid = load_pin()

    f3 = cfg.keys[0]
    fs3 = cfg.keys[1]  # F#3（最短跨键 6.4mm）
    g3 = cfg.keys[2]   # G3（12.5mm）

    def joint_of(key, state):
        p = cfg.key_pose(key, state)["position"]
        return ik(pin, model, q_idx, fid, [p["x"], p["y"], p["z"]])

    base_z = float(cfg.geometry["base_pose"]["position"]["z"])
    z_down = base_z - 0.005

    for pair_name, k_from, k_to in [("F3→F#3 6.4mm", f3, fs3), ("F#3→G3 12.5mm", fs3, g3)]:
        q_a = joint_of(k_from, "down")
        q_b = joint_of(k_to, "down")
        if q_a is None or q_b is None:
            print(f"{pair_name}: IK 失败"); continue
        p_from = cfg.key_pose(k_from, "down")["position"]
        p_to = cfg.key_pose(k_to, "down")["position"]
        mid_xy = [(float(p_from["x"]) + float(p_to["x"])) / 2.0,
                  (float(p_from["y"]) + float(p_to["y"])) / 2.0]
        print("=" * 72)
        print(f"{pair_name}：joint-space via nominal 时长 T（s）")
        print(f"  {'approach':>8} | {'vmax':>5} {'amax':>5} | {'T(s)':>7}")
        for appr_mm, appr_z in [(8, base_z + 0.008), (10, base_z + 0.010), (20, base_z + 0.020)]:
            q_via = ik(pin, model, q_idx, fid, [mid_xy[0], mid_xy[1], appr_z])
            if q_via is None:
                print(f"  {appr_mm:>6}mm via IK 失败"); continue
            dense = _interpolate_via_spline([q_a, q_via, q_b], 200)
            if dense is None:
                continue
            for vmax, amax in [(0.3, 1.0), (0.6, 2.0), (1.2, 4.0), (2.4, 8.0), (4.0, 12.0)]:
                timed = _jerk_limited_timed_traj(dense, vmax, amax)
                T = timed[0][-1] if timed else float("nan")
                print(f"  {appr_mm:>6}mm | {vmax:>5} {amax:>5} | {T:>7.2f}")


if __name__ == "__main__":
    main()
