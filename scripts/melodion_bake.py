#!/usr/bin/env python3
"""把 stream 轨迹预烘焙成关节轨迹缓存（<name>.baked.yaml）。

复用编排层同源函数（_interpolate_via_spline / _jerk_limited_timed_traj）+ 姿态保持 IK，
把 stream 的逐段 via 轨迹拼接成一条连续关节轨迹（含段间 sleep 时间轴）。
playback 时检测到 <name>.baked.yaml 存在则直接下发，跳过实时 IK+重定时（提速关键）。

用法（先 source a3_shell_env.sh）：
  python3 scripts/melodion_bake.py --stream scale [--speed 1.0] [--vmax 1.5] [--amax 15.0]
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional, Tuple

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src", "app_melodion"))
sys.path.insert(0, os.path.join(WS, "src", "a3_arm_controller", "a3_arm_controller"))

import yaml  # noqa: E402
from app_melodion.key_config import MelodionConfig  # noqa: E402
from arm_controller import _interpolate_via_spline, _jerk_limited_timed_traj  # noqa: E402

JOINTS = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]
DEFAULT_JOINT_NAMES = [*JOINTS, "L7_joint"]
SEED = [0.0, 1.05, -1.575, 0.0, 0.0, 0.0]


def load_pin():
    import pinocchio as pin
    from ament_index_python.packages import get_package_share_directory
    urdf = os.path.join(get_package_share_directory("a3_description"), "urdf", "el_a3.urdf")
    model = pin.buildModelFromUrdf(urdf)
    q_idx = [int(model.idx_qs[model.getJointId(jn)]) for jn in JOINTS]
    fid = model.getFrameId("end_effector")
    return pin, model, q_idx, fid


def ik(pin, model, q_idx, fid, xyz, refR=None) -> Optional[List[float]]:
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
        te = d.oMf[fid].translation - tgt
        if refR is None:
            return te
        return np.concatenate([te, pin.log3(refR.T @ d.oMf[fid].rotation)])

    res = least_squares(residual, x0, bounds=(lo, hi),
                        xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=120)
    return [float(v) for v in res.x] if float(np.linalg.norm(res.fun)) < 1e-3 else None


def fkR(pin, model, q_idx, fid, q6):
    q = np.zeros(model.nq)
    for v, qi in zip(q6, q_idx):
        q[qi] = float(v)
    d = model.createData()
    pin.forwardKinematics(model, d, q)
    pin.updateFramePlacement(model, d, fid)
    return np.array(d.oMf[fid].rotation, dtype=float)


def resolve_point(pin, model, q_idx, fid, point, refR=None) -> Optional[List[float]]:
    """把 target/via point 解析成 L1-L6 关节值（positions 直接；pose 走 IK）。"""
    positions = point.get("positions")
    if positions:
        q = [float(v) for v in positions]
        if len(q) < 6:
            q = q + [0.0] * (6 - len(q))
        return q[:6]
    pose = point.get("pose")
    if pose and "position" in pose:
        p = pose["position"]
        return ik(pin, model, q_idx, fid, [float(p["x"]), float(p["y"]), float(p["z"])], refR)
    return None


def two_point_quintic(q0, q1, vmax, amax, resample=200):
    """两点直线 + quintic 时间重定时（无 via 的单段）。"""
    dense = np.linspace(np.array(q0, float), np.array(q1, float), resample).tolist()
    return _jerk_limited_timed_traj(dense, vmax, amax)


def bake_stream(stream_path: str, out_path: str, vmax: float, amax: float) -> Tuple[int, float]:
    with open(stream_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    # 相对展开（与编排层 _expand_relative 同语义）
    if str(data.get("frame") or "absolute").strip().lower() == "relative":
        anchor = (data.get("anchor") or {}).get("position") or {}
        ax, ay, az = float(anchor["x"]), float(anchor["y"]), float(anchor["z"])
        def shift(pt):
            pose = (pt or {}).get("pose")
            if pose and "position" in pose:
                p = pose["position"]
                p["x"] = float(p.get("x", 0.0)) + ax
                p["y"] = float(p.get("y", 0.0)) + ay
                p["z"] = float(p.get("z", 0.0)) + az
        for seg in (data.get("segments") or []):
            shift(seg.get("target"))
            for v in (seg.get("via") or []):
                shift(v)

    pin, model, q_idx, fid = load_pin()
    segments = data.get("segments") or []
    if not segments:
        raise ValueError("empty stream")

    # 起点 = 首段 target（首键 down）
    q0 = resolve_point(pin, model, q_idx, fid, segments[0].get("target") or {})
    if q0 is None:
        raise ValueError("first segment target IK failed")
    cur_q = q0

    all_times: List[float] = []
    all_pos: List[List[float]] = []
    t_off = 0.0
    n_pts = 0
    for i, seg in enumerate(segments):
        target = seg.get("target") or {}
        via = seg.get("via") or []
        speed = float(seg.get("speed") or 1.0)
        sleep_s = float(seg.get("sleep") or 0.0)
        v = vmax * speed
        a = amax * speed

        qt = resolve_point(pin, model, q_idx, fid, target)
        if qt is None:
            raise ValueError(f"segment {i + 1} target IK failed")

        if via:
            refR = fkR(pin, model, q_idx, fid, qt)
            via_q = [cur_q]
            for vp in via:
                qv = resolve_point(pin, model, q_idx, fid, vp, refR)
                if qv is None:
                    qv = resolve_point(pin, model, q_idx, fid, vp)  # 回落 position-only
                if qv is None:
                    raise ValueError(f"segment {i + 1} via IK failed")
                via_q.append(qv)
            via_q.append(qt)
            dense = _interpolate_via_spline(via_q, 200)
            timed = _jerk_limited_timed_traj(dense, v, a)
        else:
            # 单段：cur -> qt。位移≈0（起点=首键 down）时跳过运动，只累加 sleep。
            if float(np.linalg.norm(np.array(cur_q) - np.array(qt))) < 1e-9:
                t_off += sleep_s
                cur_q = qt
                continue
            timed = two_point_quintic(cur_q, qt, v, a)

        if timed is None:
            raise ValueError(f"segment {i + 1} timing failed")
        times, pos = timed
        for t, p in zip(times, pos):
            all_times.append(t_off + float(t))
            all_pos.append([float(x) for x in p] + [0.0])  # L7 = 0
        n_pts += len(times)
        t_off += float(times[-1]) + sleep_s
        cur_q = qt

    baked = {
        "kind": "continuous",
        "retimed": True,  # F133 标志：跳过 F68 重烘焙
        "joint_names": list(DEFAULT_JOINT_NAMES),
        "points": [
            {"positions": all_pos[k], "time_from_start_sec": float(all_times[k])}
            for k in range(len(all_times))
        ],
    }
    out_path = os.path.expanduser(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(baked, f, sort_keys=False)
    return n_pts, t_off


def main() -> None:
    ap = argparse.ArgumentParser(description="stream 轨迹预烘焙（<name>.baked.yaml 关节轨迹缓存）")
    ap.add_argument("--stream", required=True, help="stream 名（app_melodion/config/streams/<name>.yaml）")
    ap.add_argument("--vmax", type=float, default=1.5, help="goto_via 峰值速度 rad/s（默认 1.5）")
    ap.add_argument("--amax", type=float, default=15.0, help="goto_via 峰值加速度 rad/s²（默认 15.0）")
    ap.add_argument("--out", default=None, help="输出路径（默认 <stream>.baked.yaml 同目录）")
    args = ap.parse_args()

    # 用 install 树 share 目录（与编排层 _stream_path 同源，baked 检测同目录命中）
    from ament_index_python.packages import get_package_share_directory
    share_streams = os.path.join(
        get_package_share_directory("app_melodion"), "config", "streams")
    stream_path = os.path.join(share_streams, args.stream + ".yaml")
    if not os.path.exists(stream_path):
        print(f"stream 不存在: {stream_path}", file=sys.stderr)
        sys.exit(2)
    out = args.out or stream_path + ".baked.yaml"
    n_pts, dur = bake_stream(stream_path, out, args.vmax, args.amax)
    print(f"baked -> {out} ({n_pts} pts, {dur:.1f}s nominal)")


if __name__ == "__main__":
    main()
