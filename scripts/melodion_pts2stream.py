#!/usr/bin/env python3
"""口风琴实测路点（`kind: waypoint` 录制产物）→ 「4 键按压 stream」+ D-pad 命名点片段。

输入（默认）：
  - trajectories/waypoints/latest.yaml（路点录制滚动槽：4 点 = white1/white19/black13/black1，
    均为「刚接触键面、未下压」的基准点：pose 笛卡尔位姿 + positions 关节角）
  - src/app_melodion/config/playing_geometry.yaml（下压深度）

输出（默认）：
  - trajectories/streams/latest.yaml（长按 Triangle 回放槽）：
      [起手] 先到「第 1 点上方 +lift_src」悬停 → 下压 down0
      →(3 过点抬升弧)→ down1 → down2 → down3
      [收尾] →(抬升弧)→ 回到「第 1 点上方 +lift_src」悬停。
    键间 3 个过点（默认不做精确过点，交给模式自带的 blend 切角）：
      ① 源按压点正上方 +lift_src（默认 40mm）
      ② 两按压点中垂线上方 +lift_apex（默认 80mm）
      ③ 目标点正上方 +lift_dst（默认 40mm）
    每段带 `hold_orientation`（默认 true）：编排层用该点 pose 的示教姿态做 **6D IK**，
    不再"只求末端到位"导致横着压键。
    文件头会备注 F166 的 9 种轨迹模式，改某一行的 `mode:` 存盘即生效（playback 每次现读）。
  - stdout 打印 4 个命名点 `melodion_<name>`（关节值取录制 positions，7J 档补 L7=0.0），
    供粘贴到 a3_description[_6j]/config/named_poses.yaml。

用法（纯离线，仅依赖 PyYAML）：
  python3 scripts/melodion_pts2stream.py
  python3 scripts/melodion_pts2stream.py --mode jnt_g64_quintic --lift-src 40 --lift-apex 80 --lift-dst 40
  python3 scripts/melodion_pts2stream.py --no-hold-orientation   # 回到 position-only IK（旧行为）
"""

from __future__ import annotations

import argparse
import os
from typing import Any, Dict, List, Tuple

import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PTS = os.path.join(WS, "trajectories", "waypoints", "latest.yaml")
DEFAULT_GEOMETRY = os.path.join(WS, "src", "app_melodion", "config", "playing_geometry.yaml")
DEFAULT_OUT = os.path.join(WS, "trajectories", "streams", "latest.yaml")
DEFAULT_NAMES = "white1,white19,black13,black1"
DEFAULT_MODE = "ptp_blend"  # F166 规范名；详见文件头备注
PRESS_FLOOR_MM = 6.0        # 下压深度下限：配置值 < 6mm 时按 6mm
LIFT_SRC_MM = 40.0          # 过点①：源按压点正上方（也用作起手/收尾悬停高度）
LIFT_APEX_MM = 80.0         # 过点②：两按压点中垂线上方
LIFT_DST_MM = 40.0          # 过点③：目标点正上方

# F166 轨迹模式 9 种：(规范名, 旧别名, 空间, 说明) —— 与 arm_controller.TRAJ_MODES 对齐
MODE_TABLE: Tuple[Tuple[str, str, str, str], ...] = (
    ("ptp",               "-",       "关节",   "MoveIt 关节 PTP 逐腿（OMPL+TOTG），逐点精确停"),
    ("ptp_blend",         "ptp_seq", "关节",   "关节 PTP Sequence + blend，不停车、过点切角"),
    ("lin_exact",         "pilz",    "笛卡尔", "pilz LIN，blend=0 逐点精确停"),
    ("lin_blend",         "lin/blend", "笛卡尔", "pilz LIN Sequence + blend，不停车、切角"),
    ("jnt_g64_quintic",   "l1p5",    "关节",   "G64 拐角圆滑 + quintic 加-巡-减（自研）"),
    ("jnt_g64_trapezoid", "l2",      "关节",   "G64 拐角圆滑 + 梯形速度（自研）"),
    ("jnt_spline_bell",   "l3",      "关节",   "三次插值样条 + 钟形（精确过点、点间可能过冲）"),
    ("circ",              "-",       "笛卡尔", "pilz CIRC 圆弧（3 点定弧，弧过中间点）"),
    ("jnt_spline_approx", "l3_fit",  "关节",   "夹紧端点最小二乘 B 样条（首末精确、中间逼近）"),
)


def load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _up_poses(pts: dict) -> List[Dict[str, Any]]:
    ups: List[Dict[str, Any]] = []
    for wp in pts["waypoints"]:
        pose = wp["pose"]
        ups.append({
            "position": {k: float(v) for k, v in pose["position"].items()},
            "orientation": {k: float(v) for k, v in pose["orientation"].items()},
        })
    return ups


def _down_pose(up: Dict[str, Any], press_mm: float) -> Dict[str, Any]:
    down = {"position": dict(up["position"]), "orientation": dict(up["orientation"])}
    down["position"]["z"] = up["position"]["z"] - press_mm / 1000.0
    return down


def _at(pose: Dict[str, Any], z_m: float, orientation: Dict[str, float]) -> Dict[str, Any]:
    """同 x/y、指定 z + 指定姿态的位姿（z 为绝对值）。"""
    return {"position": {"x": pose["position"]["x"], "y": pose["position"]["y"], "z": z_m},
            "orientation": dict(orientation)}


def _transition_vias(src: Dict[str, Any], dst: Dict[str, Any],
                     lift_src_mm: float, lift_apex_mm: float, lift_dst_mm: float,
                     exact: bool) -> List[Dict[str, Any]]:
    """src → dst 的 3 个抬高过点：源正上方 / 中垂线上方 / 目标正上方（z 相对按压点）。

    姿态：过点①②用 src 的示教姿态（保持抬起姿态），过点③用 dst 的姿态（下压姿态与目标一致）。
    """
    sp, dp = src["position"], dst["position"]

    def pt(x: float, y: float, z: float, orientation: Dict[str, float]) -> Dict[str, Any]:
        via: Dict[str, Any] = {"pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": dict(orientation),
        }}
        if exact:
            via["exact"] = True
        return via

    o_src, o_dst = src["orientation"], dst["orientation"]
    return [
        pt(sp["x"], sp["y"], sp["z"] + lift_src_mm / 1000.0, o_src),
        pt((sp["x"] + dp["x"]) / 2.0, (sp["y"] + dp["y"]) / 2.0,
           (sp["z"] + dp["z"]) / 2.0 + lift_apex_mm / 1000.0, o_src),
        pt(dp["x"], dp["y"], dp["z"] + lift_dst_mm / 1000.0, o_dst),
    ]


def _segment(target: Dict[str, Any], mode: str, speed: float, sleep_s: float,
             hold_orientation: bool, via: List[Dict[str, Any]] | None = None,
             ) -> Dict[str, Any]:
    seg: Dict[str, Any] = {"target": {"pose": target}, "mode": mode,
                           "speed": speed, "hold_orientation": hold_orientation}
    if sleep_s:
        seg["sleep"] = sleep_s
    if via:
        seg["via"] = via
    return seg


def build_stream(pts: dict, press_mm: float, speed: float, sleep_s: float,
                 lift_src_mm: float, lift_apex_mm: float, lift_dst_mm: float,
                 mode: str, exact: bool, hold_orientation: bool) -> dict:
    ups = _up_poses(pts)
    downs = [_down_pose(u, press_mm) for u in ups]
    # 起手/收尾悬停点：第 1 个（未按压）点正上方 lift_src mm
    home_hover = _at(ups[0], ups[0]["position"]["z"] + lift_src_mm / 1000.0, ups[0]["orientation"])

    # 段 1：先从远处挪到「第 1 点上方」悬停，再下压到第 1 键（避免横叉进来）
    segments: List[Dict[str, Any]] = [
        _segment(downs[0], mode, speed, sleep_s, hold_orientation, via=[{"pose": home_hover}]),
    ]
    for i in range(1, len(downs)):
        segments.append(_segment(
            downs[i], mode, speed, sleep_s, hold_orientation,
            via=_transition_vias(downs[i - 1], downs[i], lift_src_mm, lift_apex_mm,
                                 lift_dst_mm, exact)))
    # 末段：按下最后一键后抬升回「第 1 点上方」同一悬停位（过点①②抬升，目标本身已是悬停高度）
    segments.append(_segment(
        home_hover, mode, speed, 0.0, hold_orientation,
        via=_transition_vias(downs[-1], home_hover, lift_src_mm, lift_apex_mm,
                             lift_dst_mm, exact)[:2]))
    return {
        "kind": "stream",
        "joint_names": list(pts.get("joint_names") or []),
        "segments": segments,
    }


def header_lines(mode: str, press_mm: float, speed: float, sleep_s: float,
                 lift_src_mm: float, lift_apex_mm: float, lift_dst_mm: float,
                 exact: bool, hold_orientation: bool) -> List[str]:
    """文件头注释：本次生成参数 + F166 九种模式速查（便于直接改 mode 后存盘生效）。"""
    bar = "# " + "=" * 74
    out = [
        bar,
        "# 口风琴「4 键按压」stream —— 由 scripts/melodion_pts2stream.py 生成，可整份重生成。",
        "# 回放：PS4 长按 Triangle 1.5s（= /a3/arm/playback {type: stream, name: latest}）。",
        "# 路径：起手先到「第 1 点上方」悬停 → 下压 white1 → white19 → black13 → black1 → 抬回同一悬停位。",
        "#",
        f"# 本次生成：mode = {mode}，press = {press_mm}mm，lift = "
        f"{lift_src_mm}/{lift_apex_mm}/{lift_dst_mm}mm（源正上方/中垂线上方/目标正上方），"
        f"speed = {speed}，sleep = {sleep_s}s，exact = {exact}，hold_orientation = {hold_orientation}。",
        "# hold_orientation: true 时编排层用各点 pose 的示教姿态做 6D IK（不再只求末端到位→防横压）；",
        "#                   设为 false 则回落 position-only IK（旧行为）。",
        "#",
        "# 轨迹模式（F166 规范名 | 旧别名 | 空间 | 说明）—— 直接改各段的 `mode:` 存盘即生效，",
        "# 无需重建/重启栈（playback 每次调用现读本文件）。",
    ]
    for i, (canon, alias, space, note) in enumerate(MODE_TABLE, start=1):
        mark = "  ← 本次所用" if canon == mode else ""
        out.append(f"#   {i}. {canon:<19} | {alias:<9} | {space} | {note}{mark}")
    out += [
        "#",
        "# 附注：`mode: ptp` + via 会走历史兼容分支（实际等价 jnt_g64_quintic 关节过点样条）；",
        "#       via 逐点 `exact: true` 表示该点不参与 blend（精确过点），默认不加。",
        "#       笛卡尔模式（lin_*/circ）本身按 6D 位姿走，天然保持姿态。",
        bar,
    ]
    return out


def print_named(names: List[str], positions: List[List[float]]) -> None:
    print("# 追加到 src/a3_description/config/named_poses.yaml（7 关节，L7=0.0）：")
    for name, pos in zip(names, positions):
        vals = [f"{float(v):.4f}" for v in pos] + ["0.0000"]
        print(f"  melodion_{name}:")
        print(f"    positions: [{', '.join(vals)}]")
    print()
    print("# 追加到 src/a3_description_6j/config/named_poses.yaml（6 关节）：")
    for name, pos in zip(names, positions):
        vals = [f"{float(v):.4f}" for v in pos[:6]]
        print(f"  melodion_{name}:")
        print(f"    positions: [{', '.join(vals)}]")


def main() -> None:
    ap = argparse.ArgumentParser(description="口风琴实测路点 → 4 键按压 stream + 命名点片段")
    ap.add_argument("--pts", default=DEFAULT_PTS, help="路点文件（kind: waypoint）")
    ap.add_argument("--geometry", default=DEFAULT_GEOMETRY, help="playing_geometry.yaml")
    ap.add_argument("--names", default=DEFAULT_NAMES, help="逗号分隔的点名，与文件顺序一一对应")
    ap.add_argument("--mode", default=DEFAULT_MODE,
                    help="轨迹模式（F166 规范名，默认 ptp_blend；见输出文件头 9 模式表）")
    ap.add_argument("--speed", type=float, default=0.2, help="段速度缩放（0~1）")
    ap.add_argument("--press-sleep", type=float, default=0.2, help="每键按住秒数")
    ap.add_argument("--lift-src", type=float, default=LIFT_SRC_MM,
                    help="过点①/起手收尾悬停高度：源按压点（第 1 点）正上方 mm")
    ap.add_argument("--lift-apex", type=float, default=LIFT_APEX_MM, help="过点②：中垂线上方 mm")
    ap.add_argument("--lift-dst", type=float, default=LIFT_DST_MM, help="过点③：目标点正上方 mm")
    ap.add_argument("--exact", action=argparse.BooleanOptionalAction, default=False,
                    help="过点是否精确经过（默认 False，交给模式自带 blend 切角）")
    ap.add_argument("--hold-orientation", action=argparse.BooleanOptionalAction, default=True,
                    help="各点用示教姿态做 6D IK（默认 True，防横压；False=旧的 position-only）")
    ap.add_argument("--out", default=DEFAULT_OUT, help="输出 stream yaml 路径")
    args = ap.parse_args()

    canon = {m[0] for m in MODE_TABLE}
    alias = {a for m in MODE_TABLE for a in m[1].split("/") if a not in ("-",)}
    if args.mode not in canon | alias:
        print(f"[warn] mode '{args.mode}' 不在已知 9 种模式内，将原样写入 YAML")

    pts = load_yaml(args.pts)
    if str(pts.get("kind") or "") != "waypoint":
        raise SystemExit(f"expected kind: waypoint, got {pts.get('kind')!r}")
    geometry = load_yaml(args.geometry)

    cfg_press = float(geometry.get("press_depth_mm", 5.0))
    press_mm = max(cfg_press, PRESS_FLOOR_MM)

    names = [s.strip() for s in args.names.split(",") if s.strip()]
    n = len(pts["waypoints"])
    if len(names) != n:
        raise SystemExit(f"--names 数量({len(names)}) != 路点数({n})")

    doc = build_stream(pts, press_mm, args.speed, args.press_sleep,
                       args.lift_src, args.lift_apex, args.lift_dst, args.mode,
                       args.exact, args.hold_orientation)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        for line in header_lines(args.mode, press_mm, args.speed, args.press_sleep,
                                 args.lift_src, args.lift_apex, args.lift_dst,
                                 args.exact, args.hold_orientation):
            f.write(line + "\n")
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=False)

    print(f"wrote stream: {args.out} ({len(doc['segments'])} segments; mode={args.mode}, "
          f"press={press_mm}mm, lift={args.lift_src}/{args.lift_apex}/{args.lift_dst}mm, "
          f"exact={args.exact}, hold_orientation={args.hold_orientation}, "
          f"speed={args.speed}, sleep={args.press_sleep}s)")
    print()
    print_named(names, [wp["positions"] for wp in pts["waypoints"]])


if __name__ == "__main__":
    main()
