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


def build_stream(config: MelodionConfig, score: dict, speed: float, mode: str = "ptp") -> dict:
    tempo = float(score.get("tempo_bpm", config.geometry.get("tempo_bpm_default", 72)))
    beat_s = 60.0 / tempo
    segments: List[dict] = []
    prev_key = None
    last_key = None
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
    return {
        "kind": "stream",
        "joint_names": list(DEFAULT_JOINT_NAMES),
        "segments": segments,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="曲谱 -> 轨迹流(stream) 配置文件")
    parser.add_argument("--score", required=True, help="曲谱名（config/scores/<name>.yaml）或路径")
    parser.add_argument("--keys", default=None, help="melodion_keys.json 路径（默认包内 config）")
    parser.add_argument("--geometry", default=None, help="playing_geometry.yaml 路径（默认包内 config）")
    parser.add_argument("--speed", type=float, default=0.5, help="段速度缩放（0~1，MoveIt velocity_scaling）")
    parser.add_argument("--mode", choices=["ptp", "lin"], default=None,
                        help="过点段 mode（默认读 playing_geometry.yaml 的 via_apex.mode，再默认 ptp）")
    parser.add_argument("--out", default=None, help="输出 stream yaml 路径（默认 config/streams/<score>.yaml）")
    args = parser.parse_args()

    config = MelodionConfig(args.keys, args.geometry)

    if os.path.exists(args.score):
        score_path = args.score
        score_name = os.path.splitext(os.path.basename(args.score))[0]
    else:
        score_path = os.path.join(_PKG_DIR, "config", "scores", args.score + ".yaml")
        score_name = args.score
    score = load_score(score_path)

    mode = args.mode or str(
        (config.geometry.get("via_apex", {}) or {}).get("mode", "ptp")).strip().lower()
    doc = build_stream(config, score, args.speed, mode)
    doc["name"] = score_name

    out = args.out or os.path.join(_PKG_DIR, "config", "streams", score_name + ".yaml")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=False)

    print(f"wrote stream config: {out} ({len(doc['segments'])} segments)")


if __name__ == "__main__":
    main()
