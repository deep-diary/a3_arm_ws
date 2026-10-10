#!/usr/bin/env python3
"""Probe Pinocchio IK reachability for all 32 keys (up/down). Used to calibrate base_pose."""

from __future__ import annotations

from app_melodion.ik import PinocchioIK
from app_melodion.key_config import MelodionConfig


def main() -> None:
    config = MelodionConfig()
    ik = PinocchioIK()
    pos_only = bool(config.geometry.get("ik_position_only", True))
    seed = None
    ok_all = True
    for key in config.keys:
        for state in ("up", "down"):
            pose = config.key_pose(key, state)
            ok, msg, q = ik.solve(pose, seed=seed, position_only=pos_only)
            if ok:
                seed = q
                status = "OK"
            else:
                ok_all = False
                status = f"FAIL {msg}"
            print(f"{key.index:2d} {key.name:4s} {state:4s} x={key.x_mm:6.1f}mm -> {status}")
    print(f"span first->last = {config.key_span_mm():.1f} mm")
    print("ALL PASS" if ok_all else "SOME FAILED")


if __name__ == "__main__":
    main()
