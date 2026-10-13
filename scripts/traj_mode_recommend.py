#!/usr/bin/env python3
"""轨迹规划模式离线推荐器（F165.3）。

给定一条轨迹（waypoints 文件 / stream 名 / named_poses 序列），离线对候选规划模式：
  * ptp   —— 逐腿关节规划（MoveIt 语义近似：每两点一段、段末停车）
  * l1p5  —— G64 拐角圆滑（二次 Bezier）+ quintic 加-巡-减
  * l2    —— G64 拐角圆滑 + 梯形速度
  * l3    —— 三次样条路径 + 钟形速度（纯 quintic 无巡航）
计算预测时长、平滑度（a_rms/j_rms）、via 拐角统计（关节空间转角）、过冲/越限，
输出对比表 + 推荐（先按可行性过滤过冲/越限，再给「最快」「最平滑」两个候选 + 场景提示）。

纯离线（复用编排层同源纯函数 + Pinocchio 限位），无需 ROS 运行。

用法（先 source a3_shell_env.sh）：
  python3 scripts/traj_mode_recommend.py --waypoints trajectories/waypoints/latest.yaml
  python3 scripts/traj_mode_recommend.py --poses home_up,home_front,home_down,home_back
  python3 scripts/traj_mode_recommend.py --stream scale
  python3 scripts/traj_mode_recommend.py --waypoints ... --speed 0.4 --format md
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GJ = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]
ALL_JOINTS = [*GJ, "L7_joint"]
# 与 goto_via_* 参数默认一致（speed 线性缩放）
GOTO_VIA_MAX_VEL = 1.5
GOTO_VIA_MAX_ACC = 15.0
CORNER_BLEND = 0.15  # rad，与 goto_via_corner_blend_rad 默认一致

sys.path.insert(0, os.path.join(WS, "src", "a3_arm_controller", "a3_arm_controller"))
from arm_controller import (  # noqa: E402
    _round_corners,
    _linear_dense,
    _interpolate_via_spline,
    _jerk_limited_timed_traj,
    _trapezoidal_timed_traj,
    _bell_timed_traj,
    _finite_diff_metrics,
)

MODES = ("ptp", "l1p5", "l2", "l3")


# ---------------------------------------------------------------- 数据装载

def _named_poses_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("a3_description"),
                         "config", "named_poses.yaml")
        if os.path.exists(p):
            return p
    except Exception:  # noqa: BLE001
        pass
    return os.path.join(WS, "src", "a3_description", "config", "named_poses.yaml")


def load_named_poses() -> Dict[str, List[float]]:
    with open(_named_poses_path(), "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    out = {}
    for name, body in (data.get("poses") or {}).items():
        pos = body.get("positions")
        if pos:
            out[name] = [float(v) for v in pos]
    return out


def _to_q6(positions: List[float]) -> List[float]:
    q = [float(v) for v in positions]
    if len(q) < 6:
        q = q + [0.0] * (6 - len(q))
    return q[:6]


def load_chain(args) -> Tuple[List[List[float]], List[str]]:
    """返回 (chain_q6, labels)：一条关节空间折线（起点 + 各 via/target）。"""
    if args.poses:
        poses = load_named_poses()
        names = [s.strip() for s in args.poses.split(",") if s.strip()]
        missing = [n for n in names if n not in poses]
        if missing:
            raise SystemExit(f"named_poses 缺：{missing}（可用：{sorted(poses)[:12]}...）")
        return [_to_q6(poses[n]) for n in names], names

    if args.waypoints:
        with open(os.path.expanduser(args.waypoints), "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        wps = data.get("waypoints") or []
        chain = [_to_q6(wp["positions"]) for wp in wps if wp.get("positions")]
        labels = [f"wp{i}" for i in range(len(chain))]
        return chain, labels

    if args.stream:
        from ament_index_python.packages import get_package_share_directory
        st_path = os.path.join(get_package_share_directory("app_melodion"),
                               "config", "streams", args.stream + ".yaml")
        if not os.path.exists(st_path):
            raise SystemExit(f"stream 不存在: {st_path}")
        with open(st_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        segs = data.get("segments") or []
        chain: List[List[float]] = []
        labels: List[str] = []
        for i, seg in enumerate(segs):
            tgt = seg.get("target") or {}
            if tgt.get("positions"):
                if not chain:
                    chain.append(_to_q6(tgt["positions"]))
                    labels.append(f"seg{i}s")
                for v in (seg.get("via") or []):
                    if v.get("positions"):
                        chain.append(_to_q6(v["positions"]))
                        labels.append(f"seg{i}v")
                chain.append(_to_q6(tgt["positions"]))
                labels.append(f"seg{i}t")
        return chain, labels

    raise SystemExit("需指定 --waypoints / --poses / --stream 之一")


# ---------------------------------------------------------------- 关节限位

def load_limits() -> Dict[str, Tuple[float, float]]:
    try:
        import pinocchio as pin
        from ament_index_python.packages import get_package_share_directory
        urdf = os.path.join(get_package_share_directory("a3_description"),
                            "urdf", "el_a3.urdf")
        model = pin.buildModelFromUrdf(urdf)
        lim = {}
        for jn in GJ:
            qi = int(model.idx_qs[model.getJointId(jn)])
            lim[jn] = (float(model.lowerPositionLimit[qi]),
                       float(model.upperPositionLimit[qi]))
        return lim
    except Exception:  # noqa: BLE001
        return {}


# ---------------------------------------------------------------- 指标计算

def corner_stats(chain: List[List[float]]) -> Tuple[float, float]:
    """各内部顶点的关节空间方向转角（度），返回 (max, mean)。"""
    if len(chain) < 3:
        return 0.0, 0.0
    d = [np.asarray(chain[i + 1], float) - np.asarray(chain[i], float)
         for i in range(len(chain) - 1)]
    angs = []
    for i in range(1, len(d)):
        a, b = d[i - 1], d[i]
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-9 or nb < 1e-9:
            continue
        c = float(np.dot(a, b)) / (na * nb)
        angs.append(math.degrees(math.acos(max(-1.0, min(1.0, c)))))
    if not angs:
        return 0.0, 0.0
    return max(angs), sum(angs) / len(angs)


def overshoot_beyond_envelope(dense: List[List[float]],
                              via: List[List[float]]) -> float:
    """稠密路径超出 via 点包络的最大过冲（rad）。"""
    if not dense or not via:
        return 0.0
    over = 0.0
    m = len(dense[0])
    for j in range(m):
        col = [p[j] for p in dense]
        vmin = min(v[j] for v in via)
        vmax = max(v[j] for v in via)
        over = max(over, vmin - min(col), max(col) - vmax)
    return max(over, 0.0)


def limit_violation(dense: List[List[float]],
                    limits: Dict[str, Tuple[float, float]]) -> Optional[str]:
    # 容差与编排层 _spline_bounds_violation 同口径（LIM_TOL=0.05）：轻微越限由 JTC
    # 软限位 clamp 兜底不拦截，只有明显越限才判不可行。
    LIM_TOL = 0.05
    if not limits:
        return None
    for j, jn in enumerate(GJ):
        if jn not in limits:
            continue
        lo, hi = limits[jn]
        col = [p[j] for p in dense]
        if min(col) < lo - LIM_TOL or max(col) > hi + LIM_TOL:
            return f"{jn} 越限 [{min(col):+.3f},{max(col):+.3f}] vs [{lo:.3f},{hi:.3f}]"
    return None


def _metrics(times: List[float], pos: List[List[float]]) -> Tuple[float, float]:
    m = _finite_diff_metrics(times, pos)
    return (m[0], m[1]) if m else (0.0, 0.0)


def eval_mode(mode: str, chain: List[List[float]], vmax: float, amax: float,
              limits: Dict[str, Tuple[float, float]]) -> Dict:
    """对单个模式算 {dur, a_rms, j_rms, over, violation, feasible}。"""
    out = {"mode": mode, "dur": 0.0, "a_rms": 0.0, "j_rms": 0.0,
           "over": 0.0, "violation": None, "feasible": True}

    if mode == "ptp":
        # 逐腿：每两点一段（起点停、终点停），无 via 平滑；段末必停。
        if len(chain) < 2:
            out["feasible"] = False
            out["violation"] = "点数不足"
            return out
        times_all: List[float] = []
        pos_all: List[List[float]] = []
        t_off = 0.0
        denses: List[List[float]] = []
        for i in range(len(chain) - 1):
            dense = _linear_dense([chain[i], chain[i + 1]], 50)
            if not dense:
                out["feasible"] = False
                out["violation"] = "段路径退化"
                return out
            timed = _trapezoidal_timed_traj(dense, vmax, amax)
            if not timed:
                out["feasible"] = False
                out["violation"] = "段时间参数化失败"
                return out
            t, q = timed
            times_all += [t_off + float(x) for x in t]
            pos_all += [list(row) for row in q]
            t_off += float(t[-1])
            denses += dense
        out["dur"] = t_off
        out["a_rms"], out["j_rms"] = _metrics(times_all, pos_all)
        out["over"] = overshoot_beyond_envelope(denses, chain)
        out["violation"] = limit_violation(denses, limits)
        out["feasible"] = out["violation"] is None
        return out

    # l1p5 / l2 / l3
    if len(chain) < 2:
        out["feasible"] = False
        out["violation"] = "点数不足"
        return out
    if mode == "l3":
        dense = _interpolate_via_spline(chain, 200)
        timed = _bell_timed_traj(dense, vmax, amax) if dense else None
    else:
        dense = _round_corners(chain, CORNER_BLEND)
        if dense is None:
            dense = _linear_dense(chain, 200)
        timed = (_jerk_limited_timed_traj(dense, vmax, amax) if mode == "l1p5"
                 else _trapezoidal_timed_traj(dense, vmax, amax))
    if not dense or not timed:
        out["feasible"] = False
        out["violation"] = "路径/时间参数化失败"
        return out
    out["dur"] = float(timed[0][-1])
    out["a_rms"], out["j_rms"] = _metrics(timed[0], timed[1])
    out["over"] = overshoot_beyond_envelope(dense, chain)
    out["violation"] = limit_violation(dense, limits)
    # l1p5/l2 圆角是保形（不过冲）；l3 样条过冲 >0.3 rad 视为不可行
    if mode == "l3" and out["over"] > 0.3:
        out["feasible"] = False
        out["violation"] = f"过冲 {out['over']:.3f} rad > 0.3（via 包络外）"
    else:
        out["feasible"] = out["violation"] is None
    return out


# ---------------------------------------------------------------- 推荐

def recommend(rows: List[Dict], max_corner: float, mean_corner: float) -> List[str]:
    n_feas = [r for r in rows if r["feasible"]]
    lines: List[str] = []
    if mean_corner >= 60.0 or max_corner >= 90.0:
        lines.append(
            f"- **路径散布/急反向**（平均转角 {mean_corner:.0f}°、最大 {max_corner:.0f}°）："
            "全局样条（l3）易过冲/绕路，真机惯性下速度突变风险高；**建议 ptp 逐腿**"
            "（每点停车、最稳）或 **blend 降速**，不建议盲目用 l3。")
    elif max_corner >= 30.0:
        lines.append(
            f"- 路径有一定折角（最大 {max_corner:.0f}°）：G64 圆角（l1p5/l2）保形不过冲、"
            "拐角平滑，是「快且稳」的平衡选择。")
    else:
        lines.append(
            f"- 路径较平滑（最大转角 {max_corner:.0f}°）：l3 全局样条 C2 最平滑，或 l2 兼顾速度。")
    if not n_feas:
        lines.append("- **无可行模式**（均越限/过冲），请调整路点或降低速度。")
        return lines
    fastest = min(n_feas, key=lambda r: r["dur"])
    smoothest = min(n_feas, key=lambda r: r["j_rms"])
    lines.append(f"- **最快**：`{fastest['mode']}`（预测 {fastest['dur']:.2f}s）")
    lines.append(f"- **最平滑**：`{smoothest['mode']}`（j_rms={smoothest['j_rms']:.2f}）")
    return lines


def _fmt_table(rows: List[Dict]) -> str:
    hdr = ("| 模式 | 预测时长(s) | a_rms | j_rms | 过冲(rad) | 可行 | 备注 |\n"
           "|---|---|---|---|---|---|---|")
    body = []
    for r in rows:
        note = r["violation"] or ""
        body.append(f"| {r['mode']} | {r['dur']:.2f} | {r['a_rms']:.3f} | "
                    f"{r['j_rms']:.2f} | {r['over']:.3f} | "
                    f"{'✅' if r['feasible'] else '❌'} | {note} |")
    return hdr + "\n" + "\n".join(body)


def main() -> None:
    ap = argparse.ArgumentParser(description="轨迹规划模式离线推荐器（F165.3）")
    ap.add_argument("--waypoints", default=None, help="路点文件路径")
    ap.add_argument("--poses", default=None, help="named_poses 序列（逗号分隔）")
    ap.add_argument("--stream", default=None, help="stream 名（app_melodion/config/streams/）")
    ap.add_argument("--speed", type=float, default=0.4, help="速度缩放（0~1，默认 0.4）")
    args = ap.parse_args()

    chain, labels = load_chain(args)
    if len(chain) < 2:
        raise SystemExit(f"点数不足（{len(chain)}），需 ≥2 个点")
    limits = load_limits()
    vmax = args.speed * GOTO_VIA_MAX_VEL
    amax = args.speed * GOTO_VIA_MAX_ACC
    max_corner, mean_corner = corner_stats(chain)

    print(f"# 轨迹规划模式推荐（F165.3）\n")
    print(f"- 点位：{len(chain)} 个（{', '.join(labels[:8])}"
          f"{' ...' if len(labels) > 8 else ''}）")
    print(f"- 速度缩放 speed={args.speed} → vmax={vmax:.2f} rad/s, amax={amax:.2f} rad/s²"
          f"；圆角半径={CORNER_BLEND} rad")
    print(f"- via 拐角：平均 {mean_corner:.1f}°，最大 {max_corner:.1f}°\n")

    rows = [eval_mode(m, chain, vmax, amax, limits) for m in MODES]
    print("## 各模式对比\n")
    print(_fmt_table(rows))
    print("\n## 推荐\n")
    for line in recommend(rows, max_corner, mean_corner):
        print(line)
    print("\n> 说明：ptp 为逐腿 MoveIt 语义近似（段末停车）；l1p5/l2 用 G64 圆角保形；"
          "l3 为样条+钟形。时长为时间参数化预测（命令轨迹口径）。")


if __name__ == "__main__":
    main()
