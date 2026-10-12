#!/usr/bin/env python3
"""一次性工具：把口风琴 3 个关键对位点（F3 up / F3 down / C6 up）解算成关节位姿。

复用 arm_controller._solve_ik_position 的 Pinocchio position-only IK（姿态自由）逻辑，
输出 3 段 YAML 片段，用于追加到 src/a3_description/config/named_poses.yaml。

用法（需先 source a3_shell_env.sh 使 app_melodion/a3_description 在 ament 环境里）：
  python3 scripts/melodion_dpad_ik.py
"""

from __future__ import annotations

import os
from typing import List, Optional

import numpy as np

from app_melodion.key_config import MelodionConfig

JOINTS = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]
SEED = [0.0, 1.05, -1.575, 0.0, 0.0, 0.0]  # ready 关节值


def _load_pinocchio():
    import pinocchio as pin
    from ament_index_python.packages import get_package_share_directory

    urdf = os.path.join(
        get_package_share_directory("a3_description"), "urdf", "el_a3.urdf")
    model = pin.buildModelFromUrdf(urdf)
    q_idx = [int(model.idx_qs[model.getJointId(jn)]) for jn in JOINTS]
    fid = model.getFrameId("end_effector")
    if fid >= model.nframes:
        for cand in ("end_effector", "L6_link", "tool0", "link6"):
            f = model.getFrameId(cand)
            if f < model.nframes:
                fid = f
                break
    return pin, model, q_idx, fid


def solve_position_only(pin, model, q_idx, fid, target) -> Optional[List[float]]:
    from scipy.optimize import least_squares

    lo = [float(model.lowerPositionLimit[qi]) + 0.05 for qi in q_idx]
    hi = [float(model.upperPositionLimit[qi]) - 0.05 for qi in q_idx]
    x0 = [min(max(SEED[i], lo[i]), hi[i]) for i in range(6)]

    def residual(q6):
        q = np.zeros(model.nq)
        for v, qi in zip(q6, q_idx):
            q[qi] = float(v)
        data = model.createData()
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, fid)
        return data.oMf[fid].translation - target

    res = least_squares(
        residual, x0, bounds=(lo, hi),
        xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=400,
    )
    if float(np.linalg.norm(res.fun)) < 1e-3:
        return [float(v) for v in res.x]
    return None


def main() -> None:
    # 直接读源码树 config（install 树 site-packages 的 _default_path 会错位）
    ws = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg_dir = os.path.join(ws, "src", "app_melodion", "config")
    cfg = MelodionConfig(
        os.path.join(cfg_dir, "melodion_keys.json"),
        os.path.join(cfg_dir, "playing_geometry.yaml"),
    )
    f3 = cfg.keys[0]
    c6 = cfg.keys[-1]
    targets = {
        "melodion_F3_up": cfg.key_pose(f3, "up")["position"],
        "melodion_F3_down": cfg.key_pose(f3, "down")["position"],
        "melodion_C6_up": cfg.key_pose(c6, "up")["position"],
    }

    pin, model, q_idx, fid = _load_pinocchio()
    print("# 口风琴关键对位点（playing_geometry press=5/approach=10/retreat=20）")
    for name, pos in targets.items():
        tgt = np.array([float(pos["x"]), float(pos["y"]), float(pos["z"])])
        q = solve_position_only(pin, model, q_idx, fid, tgt)
        if q is None:
            print(f"# {name}: IK FAILED at {pos}")
            continue
        vals = ", ".join(f"{v:.4f}" for v in q) + ", 0.0000"
        print(f"  {name}:")
        print(f"    positions: [{vals}]")


if __name__ == "__main__":
    main()
