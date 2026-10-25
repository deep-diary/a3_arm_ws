#!/usr/bin/env python3
"""口风琴按键路点（``kind: waypoint``）→ ``kind: stream`` 轨迹（薄 CLI）。

核心编排在 ``a3_arm_controller/melodion_stream.py``（纯 Python，PS4 ↓ 示教、
↑ 生成走同一份代码）。本脚本只做参数解析 + 文件读写。

输入（默认）：
  - trajectories/melodion/waypoints/press.yaml（不存在时回落 waypoints/latest.yaml）
  - src/app_melodion/config/playing_geometry.yaml（press_depth_mm 兜底）

输出（默认）：
  - trajectories/melodion/streams/press.yaml

抬升规则（两键水平距离 d）：
  源/目标正上方 = press + clamp(alpha*d, clear_min, clear_max)
  中垂线上方   = press + clamp(beta *d, clear_min, clear_max)   （默认 alpha=0.5, beta=1.0）
每段首末精确、中间 blend；进场/离场悬停点 exact（垂直插入/拔出），末段回 home。

用法：
  python3 scripts/melodion_pts2stream.py
  python3 scripts/melodion_pts2stream.py --pts trajectories/melodion/waypoints/extremes.yaml \\
      --out trajectories/melodion/streams/extremes.yaml --names white1,white19,black13,black1
  python3 scripts/melodion_pts2stream.py --alpha 0.4 --beta 0.8 --clear-max 50 --no-home
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Any, Dict, List, Tuple

import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src", "a3_arm_controller", "a3_arm_controller"))

from melodion_stream import (  # noqa: E402
    ALPHA_DEFAULT,
    BETA_DEFAULT,
    CLEAR_MAX_MM,
    CLEAR_MIN_MM,
    DWELL_DEFAULT_S,
    MODE_DEFAULT,
    PRESS_FLOOR_MM,
    SPEED_DEFAULT,
    build_press_stream,
    header_lines,
    load_yaml,
    write_stream,
)

DEFAULT_PTS = os.path.join(WS, "trajectories", "melodion", "waypoints", "press.yaml")
LEGACY_PTS = os.path.join(WS, "trajectories", "waypoints", "latest.yaml")
DEFAULT_GEOMETRY = os.path.join(WS, "src", "app_melodion", "config", "playing_geometry.yaml")
DEFAULT_OUT = os.path.join(WS, "trajectories", "melodion", "streams", "press.yaml")
DEFAULT_NAMES = "white1,white19,black13,black1"
DEFAULT_NAMED_POSES = os.path.join(WS, "src", "a3_description", "config", "named_poses.yaml")

MODE_TABLE: Tuple[Tuple[str, str, str, str], ...] = (
    ("ptp",               "-",         "关节",   "MoveIt 关节 PTP 逐腿（OMPL+TOTG），逐点精确停"),
    ("ptp_blend",         "ptp_seq",   "关节",   "关节 PTP Sequence + blend，不停车、过点切角"),
    ("lin_exact",         "pilz",      "笛卡尔", "pilz LIN，blend=0 逐点精确停"),
    ("lin_blend",         "lin/blend", "笛卡尔", "pilz LIN Sequence + blend，不停车、切角"),
    ("jnt_g64_quintic",   "l1p5",      "关节",   "G64 拐角圆滑 + quintic 加-巡-减（自研）"),
    ("jnt_g64_trapezoid", "l2",        "关节",   "G64 拐角圆滑 + 梯形速度（自研）"),
    ("jnt_spline_bell",   "l3",        "关节",   "三次插值样条 + 钟形（精确过点、点间可能过冲）"),
    ("circ",              "-",         "笛卡尔", "pilz CIRC 圆弧（3 点定弧，弧过中间点）"),
    ("jnt_spline_approx", "l3_fit",    "关节",   "夹紧端点最小二乘 B 样条（首末精确、中间逼近）"),
)


def _load_home_positions(path: str, name: str) -> List[float] | None:
    if not path or not os.path.exists(path):
        return None
    try:
        data = load_yaml(path) or {}
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] named_poses 读取失败：{exc}", file=sys.stderr)
        return None
    root = data.get("poses") if isinstance(data.get("poses"), dict) else data
    entry = root.get(name)
    if isinstance(entry, dict) and entry.get("positions"):
        return [float(v) for v in entry["positions"]]
    return None


def print_named(names: List[str], waypoints: List[Dict[str, Any]]) -> None:
    rows = [(n, wp.get("positions")) for n, wp in zip(names, waypoints)]
    rows = [(n, p) for n, p in rows if p]
    if not rows:
        return
    print("# 追加到 src/a3_description/config/named_poses.yaml（7 关节，L7 补 0.0）：")
    for name, pos in rows:
        vals = [f"{float(v):.4f}" for v in pos] + ["0.0000"]
        print(f"  melodion_{name}:")
        print(f"    positions: [{', '.join(vals)}]")
    print()
    print("# 追加到 src/a3_description_6j/config/named_poses.yaml（6 关节）：")
    for name, pos in rows:
        vals = [f"{float(v):.4f}" for v in pos[:6]]
        print(f"  melodion_{name}:")
        print(f"    positions: [{', '.join(vals)}]")


def main() -> None:
    ap = argparse.ArgumentParser(description="口风琴按键路点 → stream 轨迹")
    ap.add_argument("--pts", default=None,
                    help=f"路点文件（默认 {DEFAULT_PTS}，缺失时回落 waypoints/latest.yaml）")
    ap.add_argument("--geometry", default=DEFAULT_GEOMETRY, help="playing_geometry.yaml")
    ap.add_argument("--names", default=DEFAULT_NAMES, help="逗号分隔的点名（仅用于打印命名点）")
    ap.add_argument("--mode", default=MODE_DEFAULT, help="轨迹模式（默认 ptp_blend）")
    ap.add_argument("--speed", type=float, default=SPEED_DEFAULT, help="段速度缩放 0~1")
    ap.add_argument("--dwell", type=float, default=DWELL_DEFAULT_S,
                    help="每个按键按住秒数（路点自带 sleep 时优先）")
    ap.add_argument("--press", type=float, default=None,
                    help="下压深度 mm（默认取 playing_geometry.press_depth_mm，下限 6mm）")
    ap.add_argument("--alpha", type=float, default=ALPHA_DEFAULT,
                    help="源/目标正上方抬升比例（×两键距离，默认 0.5）")
    ap.add_argument("--beta", type=float, default=BETA_DEFAULT,
                    help="中垂线上方抬升比例（×两键距离，默认 1.0）")
    ap.add_argument("--clear-min", type=float, default=CLEAR_MIN_MM, help="抬升下限 mm")
    ap.add_argument("--clear-max", type=float, default=CLEAR_MAX_MM, help="抬升上限 mm")
    ap.add_argument("--exact", action=argparse.BooleanOptionalAction, default=False,
                    help="键间 3 个过点是否全部 exact（默认 False，blend 切角）")
    ap.add_argument("--target-exact", action=argparse.BooleanOptionalAction, default=False,
                    help="仅把「目标键正上方」设为 exact（保守选项，多一次停车）")
    ap.add_argument("--hold-orientation", action=argparse.BooleanOptionalAction, default=True,
                    help="各点用示教姿态做 6D IK（默认 True）")
    ap.add_argument("--waypoints-pressed", action=argparse.BooleanOptionalAction, default=True,
                    help="路点是否为已按下点位（默认 True；False=旧的「刚触键面」语义）")
    ap.add_argument("--named-poses", default=DEFAULT_NAMED_POSES, help="named_poses.yaml")
    ap.add_argument("--home", default="home", help="收尾命名点（默认 home；--no-home 关闭）")
    ap.add_argument("--no-home", action="store_true", help="不回收尾段，停在末键上方")
    ap.add_argument("--out", default=DEFAULT_OUT, help="输出 stream yaml")
    args = ap.parse_args()

    pts_path = args.pts
    if pts_path is None:
        pts_path = DEFAULT_PTS if os.path.exists(DEFAULT_PTS) else LEGACY_PTS
    pts = load_yaml(pts_path)
    if str(pts.get("kind") or "") != "waypoint":
        raise SystemExit(f"expected kind: waypoint, got {pts.get('kind')!r}")
    waypoints = list(pts.get("waypoints") or [])
    if not waypoints:
        raise SystemExit(f"no waypoints in {pts_path}")

    geometry = load_yaml(args.geometry)
    cfg_press = float(geometry.get("press_depth_mm", 5.0))
    press_mm = max(float(args.press) if args.press is not None else cfg_press, PRESS_FLOOR_MM)

    home_positions = None
    if not args.no_home:
        home_positions = _load_home_positions(args.named_poses, args.home)
        if home_positions is None:
            print(f"[warn] named_poses 中找不到 '{args.home}' → 收尾改为抬到末键上方",
                  file=sys.stderr)

    doc = build_press_stream(
        waypoints,
        joint_names=pts.get("joint_names") or [],
        press_mm=press_mm,
        speed=args.speed,
        dwell_s=args.dwell,
        alpha=args.alpha,
        beta=args.beta,
        clear_min_mm=args.clear_min,
        clear_max_mm=args.clear_max,
        mode=args.mode,
        hold_orientation=args.hold_orientation,
        home_positions=home_positions,
        waypoints_pressed=args.waypoints_pressed,
        target_exact=args.target_exact,
    )
    if args.exact:
        for seg in doc["segments"]:
            for v in seg.get("via") or []:
                v.setdefault("exact", True)

    header = header_lines(
        source=os.path.relpath(pts_path, WS),
        press_mm=press_mm,
        alpha=args.alpha,
        beta=args.beta,
        clear_min_mm=args.clear_min,
        clear_max_mm=args.clear_max,
        speed=args.speed,
        dwell_s=args.dwell,
        mode=args.mode,
        hold_orientation=args.hold_orientation,
        waypoints_pressed=args.waypoints_pressed,
        home_note=(f"回命名点 '{args.home}'" if home_positions else "停在末键上方"),
        target_exact=args.target_exact,
    )
    header.append("#")
    header.append("# 轨迹模式（F166 规范名 | 旧别名 | 空间 | 说明）—— 改 mode 存盘即生效：")
    for i, (canon, alias, space, note) in enumerate(MODE_TABLE, start=1):
        mark = "  ← 本次所用" if canon == args.mode else ""
        header.append(f"#   {i}. {canon:<19} | {alias:<9} | {space} | {note}{mark}")

    write_stream(args.out, doc, header)
    print(f"wrote stream: {args.out} ({len(doc['segments'])} segments; source={pts_path}, "
          f"mode={args.mode}, press={press_mm}mm, alpha={args.alpha}, beta={args.beta}, "
          f"clear={args.clear_min}~{args.clear_max}mm, "
          f"pressed={args.waypoints_pressed}, home={'yes' if home_positions else 'no'})")
    print()
    names = [s.strip() for s in args.names.split(",") if s.strip()]
    print_named(names, waypoints)


if __name__ == "__main__":
    main()
