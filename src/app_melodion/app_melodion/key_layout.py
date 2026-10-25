"""从 4 个极限键路点反推键盘布局，并生成任意键的按下位姿（纯 Python，无 ROS 依赖）。

极限键录制顺序（PIANO 模式 ← 开始，按顺序各点一次）：
  1. 白键最左（F3, white1）
  2. 白键最右（C6, white19）
  3. 黑键最右（A#5, black13）
  4. 黑键最左（F#3, black1）

反推量：键盘轴向单位向量 u、实测跨度、黑键后退量（沿 u 的垂直方向）、
白/黑键各自的首末 z 与姿态（姿态按位置 slerp）。
每键沿键盘坐标取 ``melodion_keys.json`` 的 ``x_mm``，按实测跨度缩放。
"""

from __future__ import annotations

import json
import math
import os
from typing import Any, Dict, List, Sequence, Tuple

DEFAULT_KEYS_JSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "melodion_keys.json",
)
EXTREME_NAMES = ("white1", "white19", "black13", "black1")


def _xy(p: Dict[str, float]) -> Tuple[float, float]:
    return float(p["x"]), float(p["y"])


def _dot(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _norm(a: Tuple[float, float]) -> float:
    return math.hypot(a[0], a[1])


def _quat(q: Dict[str, float]) -> Tuple[float, float, float, float]:
    return float(q["w"]), float(q["x"]), float(q["y"]), float(q["z"])


def _slerp(q0: Tuple[float, float, float, float],
           q1: Tuple[float, float, float, float], t: float) -> Dict[str, float]:
    """四元数球面插值（短弧）。"""
    d = sum(a * b for a, b in zip(q0, q1))
    if d < 0.0:
        q1 = tuple(-v for v in q1)
        d = -d
    if d > 0.9995:
        out = [a + t * (b - a) for a, b in zip(q0, q1)]
    else:
        th0 = math.acos(max(-1.0, min(1.0, d)))
        th = th0 * t
        s0 = math.sin(th0 - th) / math.sin(th0)
        s1 = math.sin(th) / math.sin(th0)
        out = [s0 * a + s1 * b for a, b in zip(q0, q1)]
    n = math.sqrt(sum(v * v for v in out)) or 1.0
    w, x, y, z = (v / n for v in out)
    return {"w": w, "x": x, "y": y, "z": z}


def load_keys(path: str | None = None) -> List[Dict[str, Any]]:
    with open(path or DEFAULT_KEYS_JSON, "r", encoding="utf-8") as f:
        return list(json.load(f)["keys"])


def build_layout(extremes_doc: dict, keys: Sequence[Dict[str, Any]] | None = None) -> dict:
    """extremes_doc = kind: waypoint，4 点顺序同 EXTREME_NAMES。返回布局 dict。"""
    wps = list(extremes_doc.get("waypoints") or [])
    if len(wps) != len(EXTREME_NAMES):
        raise ValueError(
            f"极限键路点需要 {len(EXTREME_NAMES)} 个（{'/'.join(EXTREME_NAMES)}），"
            f"实际 {len(wps)}")
    keys = list(keys or load_keys())
    whites = [k for k in keys if k["type"] == "white"]
    blacks = [k for k in keys if k["type"] == "black"]
    if len(whites) < 19 or len(blacks) < 13:
        raise ValueError("melodion_keys.json 白键/黑键数量不足")
    k_wl, k_wh = whites[0], whites[18]
    k_bl, k_bh = blacks[0], blacks[12]

    poses = [wp["pose"] for wp in wps]
    p_wl, p_wh, p_bh, p_bl = (poses[0]["position"], poses[1]["position"],
                              poses[2]["position"], poses[3]["position"])

    vx = float(p_wh["x"]) - float(p_wl["x"])
    vy = float(p_wh["y"]) - float(p_wl["y"])
    span_m = math.hypot(vx, vy)
    if span_m < 0.01:
        raise ValueError(f"白键两端跨度过小（{span_m * 1000:.1f}mm），检查极限键录制顺序")
    u = (vx / span_m, vy / span_m)

    span_cfg_mm = float(k_wh["x_mm"]) - float(k_wl["x_mm"])
    if span_cfg_mm <= 0:
        raise ValueError("melodion_keys.json 白键跨度无效")
    scale = span_m * 1000.0 / span_cfg_mm  # 实测 mm / 配置 mm

    n = (-u[1], u[0])
    white_mid = ((float(p_wl["x"]) + float(p_wh["x"])) / 2.0,
                 (float(p_wl["y"]) + float(p_wh["y"])) / 2.0)
    black_mid = ((float(p_bl["x"]) + float(p_bh["x"])) / 2.0,
                 (float(p_bl["y"]) + float(p_bh["y"])) / 2.0)
    offset = (black_mid[0] - white_mid[0], black_mid[1] - white_mid[1])
    if _dot(offset, n) < 0.0:
        n = (-n[0], -n[1])
    retreat_mm = _dot(offset, n) * 1000.0

    return {
        "u": u,
        "n": n,
        "scale": scale,
        "retreat_mm": retreat_mm,
        "x_low_mm": float(k_wl["x_mm"]),
        "x_high_mm": float(k_wh["x_mm"]),
        "white_low": {"p": p_wl, "ori": poses[0]["orientation"], "z": float(p_wl["z"])},
        "white_high": {"p": p_wh, "ori": poses[1]["orientation"], "z": float(p_wh["z"])},
        "black_low": {"p": p_bl, "ori": poses[3]["orientation"], "z": float(p_bl["z"])},
        "black_high": {"p": p_bh, "ori": poses[2]["orientation"], "z": float(p_bh["z"])},
    }


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def key_down_pose(layout: dict, key: Dict[str, Any]) -> Dict[str, Any]:
    """按布局求某键的「按下」位姿（与极限键同一按下语义）。"""
    is_black = key["type"] == "black"
    low = layout["black_low"] if is_black else layout["white_low"]
    high = layout["black_high"] if is_black else layout["white_high"]
    x_low, x_high = layout["x_low_mm"], layout["x_high_mm"]
    t = 0.0 if x_high <= x_low else (float(key["x_mm"]) - x_low) / (x_high - x_low)
    t = max(0.0, min(1.0, t))

    # 水平坐标统一以「白键最左」为原点：黑键再加一个垂直后退量。
    # 注意不能以 black_low 为原点再叠加后退量（会重复偏移 retreat）。
    origin = layout["white_low"]["p"]
    u, n = layout["u"], layout["n"]
    along_m = float(key["x_mm"]) * layout["scale"] / 1000.0
    lat_m = layout["retreat_mm"] / 1000.0 if is_black else 0.0
    x = float(origin["x"]) + u[0] * along_m + n[0] * lat_m
    y = float(origin["y"]) + u[1] * along_m + n[1] * lat_m

    return {
        "position": {"x": x, "y": y, "z": _lerp(low["z"], high["z"], t)},
        "orientation": _slerp(_quat(low["ori"]), _quat(high["ori"]), t),
    }


def score_to_waypoints(score: dict, layout: dict, keys: Sequence[Dict[str, Any]] | None = None,
                       default_tempo: float = 72.0) -> Tuple[List[Dict[str, Any]], float]:
    """曲谱 → 按键路点序列（每音符一个按下点，sleep=时值；休止累加到上一音符）。

    返回 (waypoints, beat_s)。waypoints 直接喂给 melodion_stream.build_press_stream。
    """
    keys = list(keys or load_keys())
    by_name = {k["name"]: k for k in keys}
    tempo = float(score.get("tempo_bpm", default_tempo) or default_tempo)
    beat_s = 60.0 / tempo if tempo > 0 else 0.5
    out: List[Dict[str, Any]] = []
    for note in score.get("notes") or []:
        name = str(note.get("name"))
        beats = float(note.get("beats", 1.0))
        if name == "REST":
            if out:
                out[-1]["sleep"] = float(out[-1].get("sleep", 0.0)) + beats * beat_s
            continue
        key = by_name.get(name)
        if key is None:
            raise ValueError(f"曲谱音名 '{name}' 不在 melodion_keys.json 中")
        out.append({"pose": key_down_pose(layout, key),
                    "sleep": round(beats * beat_s, 4)})
    if not out:
        raise ValueError("曲谱没有有效音符")
    return out, beat_s
