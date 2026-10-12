#!/usr/bin/env python3
"""把曲谱转成「轨迹流（stream）」配置文件（离线脚本，无 ROS 依赖）。

输入：键位 melodion_keys.json + 弹奏几何 playing_geometry.yaml + 曲谱（score）。
输出：stream YAML（segments = 逐段笛卡尔 target + mode + speed + sleep）。

段模型（编排层执行）：
  - 每个音符 = down 段（mode=lin，sleep=音符时长，按住）+ up 段（mode=lin，sleep=0，抬起）。
  - 键间移动由「上一键 up → 本键 down」这一段 LIN 自然完成。
  - REST（休止）累加到上一段 sleep（即在抬起位停留）。

用法：
  python3 -m app_melodion.melodion_score2stream --score qinghuaci
"""

from __future__ import annotations

import argparse
import os
from typing import List

import yaml

from app_melodion.key_config import DEFAULT_JOINT_NAMES, MelodionConfig
from app_melodion.score import REST, load_score

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _make_relative(doc: dict, anchor: dict) -> dict:
    """把 stream doc 的 target/via pose.position 转为相对 anchor 的偏移，顶层写 frame/anchor。

    anchor = 第一个按压点的绝对 position。relative 模式下换口风琴位置只改 anchor。
    """
    ax, ay, az = float(anchor["x"]), float(anchor["y"]), float(anchor["z"])

    def shift(point):
        pose = point.get("pose") if isinstance(point, dict) else None
        if not pose or "position" not in pose:
            return
        p = pose["position"]
        p["x"] = float(p.get("x", 0.0)) - ax
        p["y"] = float(p.get("y", 0.0)) - ay
        p["z"] = float(p.get("z", 0.0)) - az

    for seg in (doc.get("segments") or []):
        if seg.get("target"):
            shift(seg["target"])
        for v in (seg.get("via") or []):
            shift(v)
    doc["frame"] = "relative"
    doc["anchor"] = {"position": {"x": ax, "y": ay, "z": az}}
    return doc


def build_stream(config: MelodionConfig, score: dict, speed: float, mode: str = "ptp",
                 relative: bool = False) -> dict:
    tempo = float(score.get("tempo_bpm", config.geometry.get("tempo_bpm_default", 72)))
    beat_s = 60.0 / tempo
    segments: List[dict] = []
    prev_key = None
    last_key = None
    anchor = None
    for note in score["notes"]:
        name = note["name"]
        beats = float(note["beats"])
        if name == REST:
            # 休止：累加到上一段的 sleep（停在当前按键位）
            if segments:
                segments[-1]["sleep"] = float(segments[-1].get("sleep", 0.0)) + beats * beat_s
            continue
        key = config.get_key(name)
        if key is None:
            raise ValueError(f"unknown note '{name}'")
        down = config.key_down_pose(key)
        if prev_key is None:
            anchor = down["position"]  # 相对模式锚点 = 首音符按压点
            # 首音符：接近 + 下压 + 按住（无 via）
            segments.append({
                "target": {"pose": down},
                "mode": "ptp",
                "speed": speed,
                "sleep": round(beats * beat_s, 4),
            })
        else:
            # 后续音符：抬升过弧（via 中垂线弧顶）→ 下压 → 按住（F163）
            via = config.transition_via_pose(prev_key, key)
            segments.append({
                "target": {"pose": down},
                "mode": mode,   # ptp=关节过点样条 / lin=pilz blend（默认 ptp）
                "speed": speed,
                "sleep": round(beats * beat_s, 4),
                "via": [{"pose": via}],
            })
        prev_key = key
        last_key = key
    # 末尾抬离：把手臂抬离末键（避免停在按压位）
    if last_key is not None:
        segments.append({
            "target": {"pose": config.key_hover_pose(last_key)},
            "mode": "ptp",
            "speed": speed,
            "sleep": 0.0,
        })
    doc = {
        "kind": "stream",
        "joint_names": list(DEFAULT_JOINT_NAMES),
        "segments": segments,
    }
    if relative and anchor is not None:
        _make_relative(doc, anchor)
    return doc


def build_inspection_stream(
    config: MelodionConfig, speed: float = 1.0, mode: str = "ptp",
    press_sleep: float = 0.2, relative: bool = False,
) -> dict:
    """F16x: 出厂按键检测「蛇形」轨迹——先 19 白键左→右，再 13 黑键右→左。

    同类型键之间 x 坐标相同（白键恒 base.x、黑键恒 base.x±retreat），跨键只有
    y 位移 + 抬升，无需大幅抬升；白→黑切换只发生一次 x 位移。检测完停在最左
    黑键 F#3（靠近起点 F3），方便回起点。
    """
    whites = [k for k in config.keys if k.type == "white"]
    blacks = [k for k in config.keys if k.type == "black"]
    order = whites + list(reversed(blacks))
    segments: List[dict] = []
    prev_key = None
    anchor = None
    for key in order:
        down = config.key_down_pose(key)
        if prev_key is None:
            anchor = down["position"]
            segments.append({
                "target": {"pose": down}, "mode": "ptp",
                "speed": speed, "sleep": press_sleep,
            })
        else:
            via = config.transition_via_pose(prev_key, key)
            segments.append({
                "target": {"pose": down}, "mode": mode, "speed": speed,
                "sleep": press_sleep, "via": [{"pose": via}],
            })
        prev_key = key
    # 末尾抬离末键（最左黑键 F#3）
    segments.append({
        "target": {"pose": config.key_hover_pose(order[-1])},
        "mode": "ptp", "speed": speed, "sleep": 0.0,
    })
    doc = {
        "kind": "stream",
        "joint_names": list(DEFAULT_JOINT_NAMES),
        "segments": segments,
    }
    if relative and anchor is not None:
        _make_relative(doc, anchor)
    return doc


def build_sweep_stream(
    config: MelodionConfig, modes=("ptp", "lin"),
    speeds=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0),
    press_sleep: float = 0.1, relative: bool = False,
) -> dict:
    """F16x: 模式×速度扫描测试轨迹——最左 F3 ↔ 最右 C6 一来一回，逐档扫。

    段序列：首段 F3 down（单段接近）→ 每个 (mode, speed) 档「F3→C6 去 + C6→F3 回」。
    用于验收不同模式/速度下轨迹运行是否稳定（配合 sweep_run/sweep_report）。
    """
    f3 = config.keys[0]
    c6 = config.keys[-1]
    f3_down = config.key_down_pose(f3)
    c6_down = config.key_down_pose(c6)
    segments: List[dict] = [
        {"target": {"pose": f3_down}, "mode": "ptp", "speed": 0.5, "sleep": press_sleep},
    ]
    for mode in modes:
        for speed in speeds:
            speed = round(float(speed), 1)
            via_fwd = config.transition_via_pose(f3, c6)
            via_back = config.transition_via_pose(c6, f3)
            segments.append({
                "target": {"pose": c6_down}, "mode": mode, "speed": speed,
                "sleep": press_sleep, "via": [{"pose": via_fwd}],
            })
            segments.append({
                "target": {"pose": f3_down}, "mode": mode, "speed": speed,
                "sleep": press_sleep, "via": [{"pose": via_back}],
            })
    doc = {
        "kind": "stream",
        "joint_names": list(DEFAULT_JOINT_NAMES),
        "segments": segments,
    }
    if relative:
        _make_relative(doc, f3_down["position"])
    return doc


def main() -> None:
    parser = argparse.ArgumentParser(description="曲谱 -> 轨迹流(stream) 配置文件")
    parser.add_argument("--score", default=None, help="曲谱名（config/scores/<name>.yaml）或路径")
    parser.add_argument("--inspection", action="store_true",
                        help="生成出厂按键检测蛇形轨迹（先 19 白键左→右，再 13 黑键右→左）")
    parser.add_argument("--sweep", action="store_true",
                        help="生成模式(ptp/lin)×速度(0.1~1.0) 的 F3↔C6 扫描测试轨迹")
    parser.add_argument("--keys", default=None, help="melodion_keys.json 路径（默认包内 config）")
    parser.add_argument("--geometry", default=None, help="playing_geometry.yaml 路径（默认包内 config）")
    parser.add_argument("--speed", type=float, default=0.5, help="段速度缩放（0~1，MoveIt velocity_scaling）")
    parser.add_argument("--mode", choices=["ptp", "lin"], default=None,
                        help="过点段 mode（默认读 playing_geometry.yaml 的 via_apex.mode，再默认 ptp）")
    parser.add_argument("--press-sleep", type=float, default=0.2, help="检测轨迹每键按住秒数")
    parser.add_argument("--relative", action="store_true",
                        help="生成相对位置（frame=relative + anchor 锚点，target/via 存相对偏移）")
    parser.add_argument("--out", default=None, help="输出 stream yaml 路径（默认 config/streams/<name>.yaml）")
    args = parser.parse_args()

    config = MelodionConfig(args.keys, args.geometry)
    mode = args.mode or str(
        (config.geometry.get("via_apex", {}) or {}).get("mode", "ptp")).strip().lower()

    if args.inspection:
        doc = build_inspection_stream(config, args.speed, mode, args.press_sleep,
                                      relative=args.relative)
        doc["name"] = "inspection"
        out = args.out or os.path.join(_PKG_DIR, "config", "streams", "inspection.yaml")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=False)
        print(f"wrote inspection stream: {out} ({len(doc['segments'])} segments)")
        return

    if args.sweep:
        doc = build_sweep_stream(config, relative=args.relative)
        doc["name"] = "sweep"
        out = args.out or os.path.join(_PKG_DIR, "config", "streams", "sweep.yaml")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=False)
        print(f"wrote sweep stream: {out} ({len(doc['segments'])} segments)")
        return

    if not args.score:
        parser.error("需要 --score <曲谱名> 或 --inspection 或 --sweep")

    if os.path.exists(args.score):
        score_path = args.score
        score_name = os.path.splitext(os.path.basename(args.score))[0]
    else:
        score_path = os.path.join(_PKG_DIR, "config", "scores", args.score + ".yaml")
        score_name = args.score
    score = load_score(score_path)

    doc = build_stream(config, score, args.speed, mode, relative=args.relative)
    doc["name"] = score_name

    out = args.out or os.path.join(_PKG_DIR, "config", "streams", score_name + ".yaml")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=False)

    print(f"wrote stream config: {out} ({len(doc['segments'])} segments)")


if __name__ == "__main__":
    main()
