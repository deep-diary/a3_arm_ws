#!/usr/bin/env python3
"""路点轨迹 → 多模式 stream 生成器（离线，只生成不触发运动）。

读路点录制文件（waypoints/latest.yaml，取前 6 点），生成一条 stream 轨迹文件：
**9 种轨迹模式** × N 档速度，每模式跑完整路点路径，段间 ptp 回起点保证各档对比同一路径。
运动由真机栈 playback type=stream 触发（或长按 Triangle）。

9 模式（F166 规范名，别名见 arm_controller.TRAJ_MODES）：
  ptp                关节 OMPL 逐腿（无 via，多段逐点停）
  ptp_blend          关节 PTP Sequence + blend（不停车）
  lin_exact          笛卡尔 pilz LIN blend=0（逐点精确停）
  lin_blend          笛卡尔 pilz LIN Sequence + blend（不停车）
  jnt_g64_quintic    关节 G64 圆角 + quintic 加-巡-减
  jnt_g64_trapezoid  关节 G64 圆角 + 梯形
  jnt_spline_bell    关节三次插值样条 + 钟形（精确过点）
  circ               笛卡尔 pilz CIRC 圆弧（3 点定弧：当前位→中间点→末点）
  jnt_spline_approx  关节逼近样条（首末精确、中间逼近）+ quintic

用法：
  python3 scripts/gen_stream_modes.py [--waypoints <path>] [--out <path>]
                                       [--speeds 0.4 | 0.4 0.8] [--modes a b c]
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, List

import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

MODES = [
    "ptp", "ptp_blend", "lin_exact", "lin_blend",
    "jnt_g64_quintic", "jnt_g64_trapezoid", "jnt_spline_bell",
    "circ", "jnt_spline_approx",
]
CARTESIAN_MODES = ("lin_exact", "lin_blend", "circ")  # 用 pose
MAX_POINTS = 6


def build_mode_segments(mode: str, wps: List[dict], speed: float) -> List[Dict]:
    """返回该模式「经过全部路点」的段列表（不含回起点段）。"""
    n = len(wps)

    def pos(i: int) -> list:
        return list(wps[i]["positions"])

    def pose(i: int) -> dict:
        return wps[i].get("pose") or {}

    if mode == "ptp":
        # 逐腿：每点一段（无 via），段末停 → 真正「逐点精确」
        return [{"mode": "ptp", "speed": speed, "target": {"positions": pos(i)}}
                for i in range(1, n)]
    if mode == "circ":
        # 圆弧：3 点定弧（起点=当前位≈P0、辅助点=P[n//2]、终点=P[n-1]）
        return [{"mode": "circ", "speed": speed,
                 "via": [{"pose": pose(n // 2)}],
                 "target": {"pose": pose(n - 1)}}]
    if mode in ("lin_exact", "lin_blend"):
        return [{"mode": mode, "speed": speed,
                 "via": [{"pose": pose(i)} for i in range(1, n - 1)],
                 "target": {"pose": pose(n - 1)}}]
    # 关节模式（G64 / 样条 / 逼近）
    return [{"mode": mode, "speed": speed,
             "via": [{"positions": pos(i)} for i in range(1, n - 1)],
             "target": {"positions": pos(n - 1)}}]


def main() -> None:
    ap = argparse.ArgumentParser(description="路点轨迹 → 多模式 stream 生成器")
    ap.add_argument("--waypoints", default=None,
                    help="路点文件（默认 <ws>/trajectories/waypoints/latest.yaml）")
    ap.add_argument("--out", default=None,
                    help="输出 stream 路径（默认 <ws>/trajectories/streams/modes_sweep.yaml）")
    ap.add_argument("--speeds", nargs="+", type=float, default=[0.4],
                    help="速度梯度（m/s，可多个如 0.4 或 0.4 0.8；默认仅 0.4）")
    ap.add_argument("--modes", nargs="+", default=None,
                    help=f"模式子集（默认全部 9 种：{' '.join(MODES)}）")
    args = ap.parse_args()
    speeds = args.speeds
    modes = args.modes or MODES

    wp_path = args.waypoints or os.path.join(WS, "trajectories", "waypoints", "latest.yaml")
    if not os.path.exists(wp_path):
        print(f"路点文件不存在: {wp_path}", file=sys.stderr)
        sys.exit(2)
    with open(wp_path, "r", encoding="utf-8") as f:
        wp_data = yaml.safe_load(f)

    waypoints = (wp_data.get("waypoints") or [])[:MAX_POINTS]
    n = len(waypoints)
    if n < 2:
        print(f"路点不足 2 个（实际 {n}），无法生成", file=sys.stderr)
        sys.exit(2)
    joint_names = list(wp_data.get("joint_names") or [])
    wp0 = list(waypoints[0]["positions"])

    segments: List[dict] = []
    for mode in modes:
        if mode in CARTESIAN_MODES:
            missing = [i for i in range(1, n) if not waypoints[i].get("pose")]
            if missing:
                print(f"[warn] {mode}: 路点 {missing} 缺 pose，LIN/CIRC 段将失败", file=sys.stderr)
        for idx, speed in enumerate(speeds):
            segments += build_mode_segments(mode, waypoints, speed)
            # 回程：ptp 回起点（供下一档对比同一路径、同一初始位形）
            ret = {"mode": "ptp", "speed": 0.5, "target": {"positions": list(wp0)}}
            if idx == len(speeds) - 1:
                ret["sleep"] = 2.0  # 每个模式跑完所有档速度后停 2s，便于区分
            segments.append(ret)

    doc = {"kind": "stream", "joint_names": joint_names, "segments": segments}
    out_path = args.out or os.path.join(WS, "trajectories", "streams", "modes_sweep.yaml")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=False)
    print(f"生成 stream -> {out_path}")
    print(f"  {len(modes)} 模式 × {len(speeds)} 速度 = {len(segments)} 段（含回程/逐腿展开）")
    print(f"  模式：{modes}")
    print(f"  路点：{n} 点（取前 {MAX_POINTS}）")


if __name__ == "__main__":
    main()
