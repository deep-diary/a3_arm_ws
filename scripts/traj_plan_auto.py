#!/usr/bin/env python3
"""F166 离线 auto 轨迹规划器：含 `exact` / `mode: auto` 的规划配置 → 最终 stream 配置。

对每个段：
  1. 读逐点 `exact`（是否需要精确过点）与 `mode`；
  2. `mode: auto` 时，按 exact 约束从 `arm_controller.TRAJ_MODES` 筛可行模式
     （`mode_exact_ok`），并做空间兼容过滤（笛卡尔模式需 pose、关节模式需 positions）；
  3. 离线预测各可行模式的 时长 / j_rms（软件模式用真实函数；MoveIt/pilz 模式用近似）；
  4. 归一化「时长 / 平滑」加权打分（默认各 0.5）择最优；
  5. 写出**最终 stream 配置**（mode 解析为规范名）+ 报告（各模式指标 + 选中项 + 理由）。

纯离线（复用编排层同源纯函数 + Pinocchio IK），无需 ROS 运行。

用法（先 source a3_shell_env.sh）：
  python3 scripts/traj_plan_auto.py --config <plan.yaml> [--out <final.yaml>]
                                    [--w-dur 0.5] [--w-j 0.5] [--report <report.md>]
"""
from __future__ import annotations

import argparse
import copy
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src", "a3_arm_controller", "a3_arm_controller"))

from arm_controller import (  # noqa: E402
    TRAJ_MODES, TrajModeSpec, mode_exact_ok, resolve_traj_mode,
    _round_corners, _linear_dense, _interpolate_via_spline, _approx_spline,
    _jerk_limited_timed_traj, _trapezoidal_timed_traj, _bell_timed_traj,
    _finite_diff_metrics,
)

JOINTS = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]
VMAX0 = 1.5    # goto_via_max_vel_rad_s 默认
AMAX0 = 15.0   # goto_via_max_acc_rad_s2 默认
CORNER_R = 0.15  # goto_via_corner_blend_rad 默认


# ---------------------------------------------------------------- IK / FK

def load_pin():
    import pinocchio as pin
    from ament_index_python.packages import get_package_share_directory
    urdf = os.path.join(get_package_share_directory("a3_description"), "urdf", "el_a3.urdf")
    model = pin.buildModelFromUrdf(urdf)
    q_idx = [int(model.idx_qs[model.getJointId(jn)]) for jn in JOINTS]
    fid = model.getFrameId("end_effector")
    return pin, model, q_idx, fid


def ik(pin, model, q_idx, fid, pose, ref_R=None, seed=None) -> Optional[List[float]]:
    from scipy.optimize import least_squares
    lo = [float(model.lowerPositionLimit[qi]) + 0.05 for qi in q_idx]
    hi = [float(model.upperPositionLimit[qi]) - 0.05 for qi in q_idx]
    x0 = seed or [(lo[i] + hi[i]) / 2 for i in range(6)]
    x0 = [min(max(x0[i], lo[i]), hi[i]) for i in range(6)]
    p = pose.get("position") or {}
    tgt = np.array([float(p["x"]), float(p["y"]), float(p["z"])])
    ref = None if ref_R is None else np.asarray(ref_R, float)

    def residual(q6):
        q = np.zeros(model.nq)
        for v, qi in zip(q6, q_idx):
            q[qi] = float(v)
        d = model.createData()
        pin.forwardKinematics(model, d, q)
        pin.updateFramePlacement(model, d, fid)
        te = d.oMf[fid].translation - tgt
        if ref is None:
            return te
        return np.concatenate([te, pin.log3(ref.T @ d.oMf[fid].rotation)])

    res = least_squares(residual, x0, bounds=(lo, hi),
                        xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=120)
    return [float(v) for v in res.x] if float(np.linalg.norm(res.fun)) < 1e-3 else None


def fk_R(pin, model, q_idx, fid, q6):
    q = np.zeros(model.nq)
    for v, qi in zip(q6, q_idx):
        q[qi] = float(v)
    d = model.createData()
    pin.forwardKinematics(model, d, q)
    pin.updateFramePlacement(model, d, fid)
    return np.array(d.oMf[fid].rotation, float)


def resolve_point(pin, model, q_idx, fid, point, ref_R=None, seed=None) -> Optional[List[float]]:
    positions = point.get("positions")
    if positions:
        q = [float(v) for v in positions]
        return (q + [0.0] * (6 - len(q)))[:6]
    pose = point.get("pose")
    if pose and "position" in pose:
        return ik(pin, model, q_idx, fid, pose, ref_R, seed)
    return None


# ---------------------------------------------------------------- 预测

def _metrics(t, q):
    m = _finite_diff_metrics(t, q)
    return (m[0], m[1]) if m else (0.0, 0.0)


def _leg_trapezoid(qa, qb, vmax, amax):
    dense = _linear_dense([qa, qb], 50)
    return _trapezoidal_timed_traj(dense, vmax, amax) if dense else None


def _split_exact(chain: List[List[float]], exact: List[bool]):
    """按 exact 内部点切分子链（exact 点须精确过点 → 视作子段边界、段末停）。"""
    sub = [[chain[0]]]
    for i, q in enumerate(chain[1:-1]):
        sub[-1].append(q)
        if i < len(exact) and exact[i]:
            sub.append([q])  # 在精确点处分段（下一子链从该点起）
    sub[-1].append(chain[-1])
    return [s for s in sub if len(s) >= 2]


def _continuous_dur(chain, spec, vmax, amax):
    """单条连续子链的 (dur, j_rms)。"""
    if len(chain) < 2:
        return 0.0, 0.0
    if spec.profile == "l3":
        dense = _interpolate_via_spline(chain, 200)
    elif spec.profile == "l3_fit":
        dense = _approx_spline(chain, 200) or _interpolate_via_spline(chain, 200)
    else:
        dense = _round_corners(chain, CORNER_R) if len(chain) >= 3 else _linear_dense(chain, 50)
    if not dense:
        return None
    if spec.profile == "l3":
        timed = _bell_timed_traj(dense, vmax, amax)
    elif spec.profile == "l2":
        timed = _trapezoidal_timed_traj(dense, vmax, amax)
    else:
        timed = _jerk_limited_timed_traj(dense, vmax, amax)
        if timed is None:  # 退化（如 2 点子链）→ 回落梯形
            timed = _trapezoidal_timed_traj(dense, vmax, amax)
    if not timed:
        return None
    return float(timed[0][-1]), _metrics(timed[0], timed[1])[1]


def predict(spec: TrajModeSpec, chain: List[List[float]], poses_ok: bool,
            speed: float, exact: List[bool]) -> Optional[Tuple[float, float]]:
    """离线预测 (dur, j_rms)。chain=[起点, *via, target]。exact 内部点为 True 则在该点停车分段。"""
    vmax, amax = VMAX0 * speed, AMAX0 * speed

    if spec.kind == "circ_single":
        if len(chain) < 3:
            return None
        d = float(np.linalg.norm(np.array(chain[-1]) - np.array(chain[0])))
        S = 1.2 * d  # 弧长 ~1.2× 弦长
        T = max(1.875 * S / max(vmax, 1e-3), np.sqrt(5.7735 * S / max(amax, 1e-3)))
        return T, 0.0

    # 逐腿停 类：每两点一段
    if spec.kind in ("ptp_legs", "lin_exact") or spec.stop_each:
        T = 0.0; ja = 0.0
        for i in range(len(chain) - 1):
            td = _leg_trapezoid(chain[i], chain[i + 1], vmax, amax)
            if td is None:
                return None
            T += float(td[0][-1]); ja = max(ja, _metrics(td[0], td[1])[1])
        return T, ja

    # 连续平滑类（soft_smooth / *_seq）：按 exact 切子链求和
    subs = _split_exact(chain, exact)
    T = 0.0; jmax = 0.0
    for s in subs:
        r = _continuous_dur(s, spec, vmax, amax)
        if r is None:
            return None
        T += r[0]; jmax = max(jmax, r[1])
    return T, jmax



# ---------------------------------------------------------------- 规划

def segment_chain(pin, model, q_idx, fid, seg, cur_q) -> Optional[Tuple[List[List[float]], bool]]:
    """把段解析成关节链 [起点, *via, target]；返回 (chain, poses_ok)。"""
    via = seg.get("via") or []
    target = seg.get("target") or {}
    poses_ok = True

    def pose_ok(p):
        pose = p.get("pose")
        return bool(pose and "position" in pose and "orientation" in pose)

    qt = resolve_point(pin, model, q_idx, fid, target, seed=cur_q)
    if qt is None:
        return None
    ref_R = fk_R(pin, model, q_idx, fid, qt)
    chain = [list(cur_q)]
    for p in via:
        q = resolve_point(pin, model, q_idx, fid, p, ref_R, seed=qt)
        if q is None:
            q = resolve_point(pin, model, q_idx, fid, p, seed=qt)
        if q is None:
            return None
        chain.append(q)
        poses_ok = poses_ok and pose_ok(p)
    chain.append(qt)
    poses_ok = poses_ok and pose_ok(target)
    return chain, poses_ok


def plan_config(cfg: dict, w_dur: float, w_j: float) -> Tuple[dict, List[dict]]:
    pin, model, q_idx, fid = load_pin()
    segs = cfg.get("segments") or []
    jnames = list(cfg.get("joint_names") or [f"L{i}_joint" for i in range(1, 8)])
    cur_q = [0.0] * 6

    def _q6(pos):
        q = [float(v) for v in pos]
        return (q + [0.0] * (6 - len(q)))[:6]

    # 起点：优先 cfg.start，否则首段第一个 via（≈路径起点），再否则首段 target
    st = ((cfg.get("start") or {}).get("positions"))
    if st:
        cur_q = _q6(st)
    elif segs:
        s0 = segs[0]
        v0 = (s0.get("via") or [None])[0]
        p0 = v0 if (v0 and v0.get("positions")) else (s0.get("target") or {})
        if p0.get("positions"):
            cur_q = _q6(p0["positions"])

    out_segs: List[dict] = []
    report: List[dict] = []
    for i, seg in enumerate(segs):
        speed = float(seg.get("speed") or 1.0)
        mode = str(seg.get("mode") or "auto").strip().lower()
        via = seg.get("via") or []
        need = [bool(p.get("exact", False)) for p in via]

        res = segment_chain(pin, model, q_idx, fid, seg, cur_q)
        if res is None:
            report.append({"i": i + 1, "mode": mode, "chosen": "?", "reason": "IK 解析失败"})
            out_segs.append(copy.deepcopy(seg))
            continue
        chain, poses_ok = res
        cur_q = chain[-1]

        base = resolve_traj_mode(mode)
        if base is not None:
            out_segs.append(copy.deepcopy(seg))
            report.append({"i": i + 1, "mode": mode, "chosen": base.canonical,
                           "reason": "显式指定"})
            continue

        # auto：筛可行模式
        cands: List[Tuple[TrajModeSpec, Optional[Tuple[float, float]]]] = []
        for spec in TRAJ_MODES.values():
            if not mode_exact_ok(spec, need):
                continue
            if spec.space == "cartesian" and not poses_ok:
                continue
            if spec.kind == "circ_single" and len(via) != 1:
                continue
            if spec.kind == "ptp_legs" and via:
                continue  # ptp 逐腿在 auto 里用多段表达，不选单段 ptp+via
            cands.append((spec, predict(spec, chain, poses_ok, speed, need)))

        valid = [(s, m) for s, m in cands if m is not None]
        if not valid:
            report.append({"i": i + 1, "mode": "auto", "chosen": "?",
                           "reason": "无可行模式"})
            out_segs.append(copy.deepcopy(seg))
            continue
        durs = [m[0] for _, m in valid]
        js = [m[1] for _, m in valid]
        dlo, dhi = min(durs), max(durs)
        jlo, jhi = min(js), max(js)

        def score(m):
            dh = (m[0] - dlo) / (dhi - dlo) if dhi > dlo else 0.0
            jh = (m[1] - jlo) / (jhi - jlo) if jhi > jlo else 0.0
            return w_dur * dh + w_j * jh

        best, best_m = min(valid, key=lambda t: score(t[1]))
        new_seg = copy.deepcopy(seg)
        new_seg["mode"] = best.canonical
        out_segs.append(new_seg)
        report.append({
            "i": i + 1, "mode": "auto", "chosen": best.canonical,
            "reason": f"score={score(best_m):.3f} dur={best_m[0]:.2f}s j_rms={best_m[1]:.2f}",
            "cands": [{"m": s.canonical, "dur": m[0], "j": m[1], "score": score(m)}
                      for s, m in sorted(valid, key=lambda t: score(t[1]))],
        })

    out = {"kind": cfg.get("kind", "stream"), "joint_names": jnames, "segments": out_segs}
    if "frame" in cfg:
        out["frame"] = cfg["frame"]
    if "anchor" in cfg:
        out["anchor"] = cfg["anchor"]
    return out, report


def main() -> None:
    ap = argparse.ArgumentParser(description="F166 离线 auto 轨迹规划器")
    ap.add_argument("--config", required=True, help="规划配置（stream schema，含 exact / mode:auto）")
    ap.add_argument("--out", default=None, help="输出最终 stream 配置（默认 <config>.resolved.yaml）")
    ap.add_argument("--report", default=None, help="报告 md（默认 <config>.auto_report.md）")
    ap.add_argument("--w-dur", type=float, default=0.5, help="时长权重（默认 0.5）")
    ap.add_argument("--w-j", type=float, default=0.5, help="平滑权重（默认 0.5）")
    args = ap.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    out, report = plan_config(cfg, args.w_dur, args.w_j)

    out_path = args.out or (os.path.splitext(args.config)[0] + ".resolved.yaml")
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True, default_flow_style=False)
    print(f"最终 stream -> {out_path}")

    lines = ["# F166 离线 auto 规划报告", "",
             f"- 时长/平滑权重：w_dur={args.w_dur} w_j={args.w_j}", ""]
    for r in report:
        lines.append(f"## 段 {r['i']}  mode={r['mode']} → **{r['chosen']}**")
        lines.append(f"- {r['reason']}")
        for c in r.get("cands", []):
            lines.append(f"  - {c['m']}: dur={c['dur']:.2f}s j_rms={c['j']:.2f} score={c['score']:.3f}")
        lines.append("")
    rep_path = args.report or (os.path.splitext(args.config)[0] + ".auto_report.md")
    with open(rep_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"报告 -> {rep_path}")
    for r in report:
        print(f"  段{r['i']:2d} {r['mode']:>8s} -> {r['chosen']:20s} {r['reason']}")


if __name__ == "__main__":
    main()
