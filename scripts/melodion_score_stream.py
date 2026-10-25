#!/usr/bin/env python3
"""极限键坐标 + 曲谱 → 乐曲 stream（口风琴，离线生成，不触发运动）。

流程：
  1. 读 ``trajectories/melodion/waypoints/extremes.yaml``（4 极限键按下点）；
  2. ``app_melodion.key_layout`` 反推键盘轴向/跨度/黑键后退，生成每键按下位姿；
  3. 曲谱音符 → 路点序列（sleep = 时值，休止累加到上一音符）；
  4. 复用 ``melodion_stream.build_press_stream``：α/β 抬升、进场/离场、
     末段回 home，输出与示教生成的 press stream 同一套 schema。

用法：
  python3 scripts/melodion_score_stream.py --score qinghuaci
  python3 scripts/melodion_score_stream.py --score scale --alpha 0.4 --beta 0.8
"""

from __future__ import annotations

import argparse
import os
import sys

import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src", "app_melodion"))
sys.path.insert(0, os.path.join(WS, "src", "a3_arm_controller", "a3_arm_controller"))

from app_melodion.key_layout import load_keys, build_layout, score_to_waypoints  # noqa: E402
from app_melodion.score import load_score, validate_score  # noqa: E402
from app_melodion.key_config import MelodionConfig  # noqa: E402
from melodion_stream import (  # noqa: E402
    ALPHA_DEFAULT, BETA_DEFAULT, CLEAR_MAX_MM, CLEAR_MIN_MM, PRESS_FLOOR_MM,
    SPEED_DEFAULT, build_press_stream, header_lines, load_yaml, write_stream,
)

PKG = os.path.join(WS, "src", "app_melodion")
DEFAULT_EXTREMES = os.path.join(WS, "trajectories", "melodion", "waypoints", "extremes.yaml")
DEFAULT_OUT = os.path.join(WS, "trajectories", "melodion", "streams", "music.yaml")
DEFAULT_NAMED_POSES = os.path.join(WS, "src", "a3_description", "config", "named_poses.yaml")


def _home_positions(path: str, name: str):
    if not os.path.exists(path):
        return None
    data = yaml.safe_load(open(path, "r", encoding="utf-8")) or {}
    root = data.get("poses") if isinstance(data.get("poses"), dict) else data
    entry = root.get(name)
    if isinstance(entry, dict) and entry.get("positions"):
        return [float(v) for v in entry["positions"]]
    return None


def main() -> None:
    ap = argparse.ArgumentParser(description="极限键坐标 + 曲谱 → 乐曲 stream")
    ap.add_argument("--score", default="qinghuaci", help="曲谱名（config/scores/<name>.yaml）或路径")
    ap.add_argument("--extremes", default=DEFAULT_EXTREMES, help="4 极限键路点文件")
    ap.add_argument("--keys", default=None, help="melodion_keys.json（默认包内）")
    ap.add_argument("--named-poses", default=DEFAULT_NAMED_POSES, help="named_poses.yaml")
    ap.add_argument("--home", default="home", help="收尾命名点（默认 home）")
    ap.add_argument("--no-home", action="store_true", help="不回收尾段")
    ap.add_argument("--mode", default="ptp_blend", help="轨迹模式（默认 ptp_blend）")
    ap.add_argument("--speed", type=float, default=SPEED_DEFAULT)
    ap.add_argument("--press", type=float, default=None, help="下压深度 mm（默认取几何配置）")
    ap.add_argument("--alpha", type=float, default=ALPHA_DEFAULT)
    ap.add_argument("--beta", type=float, default=BETA_DEFAULT)
    ap.add_argument("--clear-min", type=float, default=CLEAR_MIN_MM)
    ap.add_argument("--clear-max", type=float, default=CLEAR_MAX_MM)
    ap.add_argument("--min-dwell", type=float, default=0.05,
                    help="最短按住时长 s（音符时值小于此值时补齐）")
    ap.add_argument("--hold-orientation", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--target-exact", action=argparse.BooleanOptionalAction, default=False,
                    help="仅把「目标键正上方」设为 exact（保守选项）")
    ap.add_argument("--out", default=DEFAULT_OUT)
    args = ap.parse_args()

    if not os.path.exists(args.extremes):
        raise SystemExit(f"极限键路点不存在：{args.extremes}（PIANO 模式 ← 录制 4 个极限键）")
    extremes = load_yaml(args.extremes)
    if str(extremes.get("kind") or "") != "waypoint":
        raise SystemExit(f"expected kind: waypoint, got {extremes.get('kind')!r}")

    keys = load_keys(args.keys)
    layout = build_layout(extremes, keys)
    print(f"layout: span={layout['scale'] * (layout['x_high_mm'] - layout['x_low_mm']):.1f}mm, "
          f"retreat={layout['retreat_mm']:.1f}mm, "
          f"u=({layout['u'][0]:.3f},{layout['u'][1]:.3f})")

    if os.path.exists(args.score):
        score_path = args.score
    else:
        score_path = os.path.join(PKG, "config", "scores", args.score + ".yaml")
    if not os.path.exists(score_path):
        raise SystemExit(f"曲谱不存在：{score_path}")
    score = load_score(score_path)
    ok, msg = validate_score(score, MelodionConfig(args.keys))
    if not ok:
        raise SystemExit(f"曲谱校验失败：{msg}")

    waypoints, beat_s = score_to_waypoints(score, layout, keys)
    for wp in waypoints:
        wp["sleep"] = max(float(wp.get("sleep", 0.0)), float(args.min_dwell))

    geometry = MelodionConfig(args.keys).geometry
    cfg_press = float(geometry.get("press_depth_mm", 5.0))
    press_mm = max(float(args.press) if args.press is not None else cfg_press, PRESS_FLOOR_MM)
    home_positions = None if args.no_home else _home_positions(args.named_poses, args.home)

    doc = build_press_stream(
        waypoints,
        joint_names=list(extremes.get("joint_names") or []),
        press_mm=press_mm, speed=args.speed, dwell_s=float(args.min_dwell),
        alpha=args.alpha, beta=args.beta,
        clear_min_mm=args.clear_min, clear_max_mm=args.clear_max,
        mode=args.mode, hold_orientation=args.hold_orientation,
        home_positions=home_positions, waypoints_pressed=True,
        target_exact=args.target_exact,
    )
    score_name = os.path.splitext(os.path.basename(score_path))[0]
    doc["name"] = score_name
    header = header_lines(
        source=f"曲谱 {os.path.relpath(score_path, WS)} + 极限键 {os.path.relpath(args.extremes, WS)}",
        press_mm=press_mm, alpha=args.alpha, beta=args.beta,
        clear_min_mm=args.clear_min, clear_max_mm=args.clear_max,
        speed=args.speed, dwell_s=float(args.min_dwell), mode=args.mode,
        hold_orientation=args.hold_orientation, waypoints_pressed=True,
        home_note=(f"回命名点 '{args.home}'" if home_positions else "停在末键上方"),
        target_exact=args.target_exact,
    )
    write_stream(args.out, doc, header)
    total = sum(float(wp.get("sleep", 0.0)) for wp in waypoints)
    print(f"wrote music stream: {args.out} ({len(waypoints)} notes, "
          f"{len(doc['segments'])} segments, 按键总时长≈{total:.1f}s, beat={beat_s:.3f}s)")


if __name__ == "__main__":
    main()
