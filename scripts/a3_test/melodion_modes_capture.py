#!/usr/bin/env python3
"""口风琴 32 键弹琴轨迹：9 模式对比采集（仿真）。

轨迹来源：src/app_melodion/config/streams/scale.yaml（32 键 F3→C6 逐键，33 段；
每段链 = [上一键(当前位) → 抬升 via → 下一键] = **3 点**）。
对 9 种模式各生成一条 stream（所有段 mode 覆盖、speed 统一），逐模式 playback，
订阅 /joint_states（实测关节）+ /a3/arm_status（段号）→ JSON 供报告脚本。

模式差异说明：
  * `ptp`：把 3 点链展开为 2 个「无 via」段（真·逐腿逐点停，含在抬升点停车）；
  * 其余模式保持 1 段/链（含 1 个 via）。
  * `jnt_spline_approx` 在 3 点链下会退回插值样条（`_approx_spline` 需 ≥4 点）——已如实记录。

用法：
  source scripts/a3_shell_env.sh && export ROS_DOMAIN_ID=99
  python3 scripts/a3_test/melodion_modes_capture.py --speed 0.4 --out /tmp/melodion_modes.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Tuple

import numpy as np
import rclpy
import yaml

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(WS, "scripts", "a3_test"))
sys.path.insert(0, os.path.join(WS, "scripts"))
from traj_modes_tcp_capture import Capture, MODES, STREAM_DIR  # noqa: E402

BASE = os.path.join(WS, "src", "app_melodion", "config", "streams", "inspection.yaml")
JOINTS6 = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]
SEED = [0.0, 1.05, -1.575, 0.0, 0.0, 0.0]  # ready 关节值


def resolve_relative(base: dict) -> int:
    """frame=relative 时把 target/via 的 pose.position 加上 anchor 展开为绝对（就地修改）。"""
    if str(base.get("frame", "absolute")).lower() != "relative":
        return 0
    a = (base.get("anchor") or {}).get("position") or {}
    ax, ay, az = float(a["x"]), float(a["y"]), float(a["z"])
    n = 0

    def shift(holder: dict):
        nonlocal n
        p = (holder.get("pose") or {}).get("position")
        if not p:
            return
        p["x"] += ax; p["y"] += ay; p["z"] += az
        n += 1

    for s in base["segments"]:
        shift(s.get("target", {}))
        for v in (s.get("via") or []):
            shift(v)
    base.pop("frame", None)
    base.pop("anchor", None)
    return n


def _pinocchio():
    import pinocchio as pin
    from ament_index_python.packages import get_package_share_directory
    urdf = os.path.join(get_package_share_directory("a3_description"), "urdf", "el_a3.urdf")
    model = pin.buildModelFromUrdf(urdf)
    q_idx = [int(model.idx_qs[model.getJointId(jn)]) for jn in JOINTS6]
    fid = model.getFrameId("end_effector")
    return pin, model, q_idx, fid


def _ik_orientation(pin, model, q_idx, fid, xyz) -> Dict[str, float] | None:
    """position-only IK（同 melodion_dpad_ik），再 FK 取真实姿态四元数（供 LIN/CIRC 用）。"""
    from scipy.optimize import least_squares
    lo = [float(model.lowerPositionLimit[qi]) + 0.05 for qi in q_idx]
    hi = [float(model.upperPositionLimit[qi]) - 0.05 for qi in q_idx]
    x0 = [min(max(SEED[i], lo[i]), hi[i]) for i in range(6)]

    def res(q6):
        q = np.zeros(model.nq)
        for v, qi in zip(q6, q_idx):
            q[qi] = float(v)
        d = model.createData()
        pin.forwardKinematics(model, d, q)
        pin.updateFramePlacement(model, d, fid)
        return d.oMf[fid].translation - xyz

    r = least_squares(res, x0, bounds=(lo, hi), xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=400)
    if float(np.linalg.norm(r.fun)) >= 1e-3:
        return None
    q = np.zeros(model.nq)
    for v, qi in zip(r.x, q_idx):
        q[qi] = float(v)
    d = model.createData()
    pin.forwardKinematics(model, d, q)
    pin.updateFramePlacement(model, d, fid)
    quat = pin.Quaternion(d.oMf[fid].rotation)
    return {"x": float(quat.x), "y": float(quat.y), "z": float(quat.z), "w": float(quat.w)}


def fix_orientations(base_segs: List[dict]) -> int:
    """把每个点的占位姿态(0,0,0,1) 换成该点 position-only IK 的真实姿态（就地修改）。"""
    pin, model, q_idx, fid = _pinocchio()
    cache: Dict[Tuple, Dict[str, float]] = {}
    n = 0

    def fix(pose_holder: dict):
        nonlocal n
        pose = pose_holder.get("pose")
        if not pose or "position" not in pose:
            return
        p = pose["position"]
        key = (round(float(p["x"]), 5), round(float(p["y"]), 5), round(float(p["z"]), 5))
        if key not in cache:
            o = _ik_orientation(pin, model, q_idx, fid, np.array(key))
            cache[key] = o if o else {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        pose["orientation"] = dict(cache[key])
        n += 1

    for s in base_segs:
        fix(s.get("target", {}))
        for v in (s.get("via") or []):
            fix(v)
    return n


def _yz(seg_or_point: dict) -> List[float]:
    p = seg_or_point.get("target", seg_or_point).get("pose", {}).get("position")
    return [float(p["y"]), float(p["z"])] if p else [0.0, 0.0]


def build_mode_stream(base_segs: List[dict], mode: str, speed: float) -> Tuple[List[dict], List[List[float]]]:
    """返回 (段列表, 每段的期望目标 yz)。"""
    out: List[dict] = []
    exp: List[List[float]] = []
    for s in base_segs:
        tgt = s["target"]
        via = s.get("via") or []
        sleep = s.get("sleep")
        if mode == "ptp" and via:
            # 3 点链 → 2 个无 via 段（真逐腿停）
            out.append({"mode": "ptp", "speed": speed, "target": via[0]})
            exp.append(_yz(via[0]))
            seg2 = {"mode": "ptp", "speed": speed, "target": tgt}
            if sleep:
                seg2["sleep"] = sleep
            out.append(seg2)
            exp.append(_yz(tgt))
        else:
            seg = {"mode": mode, "speed": speed, "target": tgt}
            if via:
                seg["via"] = via
            if sleep:
                seg["sleep"] = sleep
            out.append(seg)
            exp.append(_yz(tgt))
    return out, exp


def write_streams(base_segs: List[dict], jnames: List[str], speed: float):
    os.makedirs(STREAM_DIR, exist_ok=True)
    meta: Dict[str, Tuple[str, int, List[List[float]]]] = {}
    for mode in MODES:
        segs, exp = build_mode_stream(base_segs, mode, speed)
        name = f"melodion_cmp_{mode}"
        with open(os.path.join(STREAM_DIR, name + ".yaml"), "w", encoding="utf-8") as f:
            yaml.safe_dump({"kind": "stream", "joint_names": list(jnames), "segments": segs},
                           f, sort_keys=False, allow_unicode=True)
        meta[mode] = (name, len(segs), exp)
    # prep：跑到第 1 键下压点（独立 playback，不计采样）
    prep = [{"mode": "ptp", "speed": speed, "target": base_segs[0]["target"], "sleep": 0.5}]
    with open(os.path.join(STREAM_DIR, "melodion_cmp_prep.yaml"), "w", encoding="utf-8") as f:
        yaml.safe_dump({"kind": "stream", "joint_names": list(jnames), "segments": prep},
                       f, sort_keys=False, allow_unicode=True)
    return meta


def main() -> None:
    ap = argparse.ArgumentParser(description="口风琴 32 键 9 模式采集")
    ap.add_argument("--base", default=BASE)
    ap.add_argument("--speed", type=float, default=0.4)
    ap.add_argument("--out", default="/tmp/melodion_modes.json")
    ap.add_argument("--modes", nargs="+", default=None, help=f"模式子集（默认全部：{' '.join(MODES)}）")
    ap.add_argument("--no-fix-orientation", action="store_true",
                    help="不改姿态（口风琴键位 orientation 是占位 (0,0,0,1)，LIN/CIRC 会失败）")
    args = ap.parse_args()
    modes = args.modes or MODES

    with open(args.base, "r", encoding="utf-8") as f:
        base = yaml.safe_load(f)
    base_segs = base["segments"]
    jnames = base.get("joint_names") or []
    nrel = resolve_relative(base)
    ys = [float(s["target"]["pose"]["position"]["y"]) for s in base_segs[:32]]
    print(f"基准轨迹: {os.path.basename(args.base)}  {len(base_segs)} 段（32 键，每链 3 点）；跑 {len(modes)} 模式")
    if nrel:
        print(f"相对坐标展开: {nrel} 个点 + anchor → 绝对（y {min(ys):.3f}~{max(ys):.3f} m）")
    if not args.no_fix_orientation:
        n = fix_orientations(base_segs)
        print(f"姿态修正: {n} 个点（占位四元数 → position-only IK 的真实姿态），供 LIN/CIRC 使用")
    meta = write_streams(base_segs, jnames, args.speed)

    rclpy.init()
    node = Capture()
    rclpy.spin_once(node, timeout_sec=0.5)

    out: Dict = {"speed": args.speed, "base": os.path.basename(args.base), "modes": {}}
    if os.path.exists(args.out):  # 累加（分批跑时保留已有结果）
        try:
            with open(args.out, "r", encoding="utf-8") as f:
                prev = json.load(f)
            out["modes"] = prev.get("modes", {}) or {}
        except Exception:
            pass
    for mode in modes:
        name, n_total, exp = meta[mode]
        node.run_prep("melodion_cmp_prep")  # 先停到第 1 键
        r = node.run_mode(name, n_total)
        r["expected"] = exp
        out["modes"][mode] = r
        print(f"[{mode:18s}] ok={r['ok']} n={r['n']:4d} segs={n_total:3d} wall={r['wall']:6.1f}s  {r['msg']}")

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f)
    print(f"采集 -> {args.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
