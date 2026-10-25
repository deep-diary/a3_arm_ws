"""口风琴按键路点 → stream 轨迹（纯 Python，无 ROS 依赖）。

路点语义（默认 ``waypoints_pressed=True``）：每个 ``waypoint.pose`` 是**已按下**的
6D 位姿；``press_mm`` 用于还原键面高度。抬升高度按相邻两键水平距离 ``d`` 线性缩放：

    源/目标正上方：``press_mm + clamp(alpha * d, clear_min, clear_max)``
    中垂线上方  ：``press_mm + clamp(beta  * d, clear_min, clear_max)``

``alpha=0.5``、``beta=1.0`` 为默认；必须夹紧（远距离跨键时 β·d 会到几百 mm，
不可达），因此有 ``clear_min_mm`` / ``clear_max_mm``。

生成的段模型与编排层 ``_execute_segment`` 一致：
``{target, mode, speed, sleep, via:[{pose, exact}], hold_orientation}``。
- 每段首末点精确（段末速度 0），3 个键间 via 交给模式自带 blend 切角（默认与源/目标对称）；
- 进场/离场的关键悬停点加 ``exact: true``：从任意当前位/回 home 时先停稳再垂直插入/拔出；
- ``target_exact=True`` 可把「目标键正上方」也设为 exact（多一次停车，保守选项）；
- 结束段可给 ``home_positions``（关节命名点 home），仅关节类模式支持。
"""

from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Optional, Sequence

import yaml

PRESS_FLOOR_MM = 6.0
ALPHA_DEFAULT = 0.5
BETA_DEFAULT = 1.0
CLEAR_MIN_MM = 20.0
CLEAR_MAX_MM = 60.0
DWELL_DEFAULT_S = 0.2
SPEED_DEFAULT = 0.2
MODE_DEFAULT = "ptp_blend"

JOINT_MODES = frozenset({
    "ptp", "ptp_blend", "ptp_seq",
    "jnt_g64_quintic", "jnt_g64_trapezoid", "jnt_spline_bell", "jnt_spline_approx",
    "l1p5", "l2", "l3", "l3_fit",
})


def load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _copy_pose(pose: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "position": {k: float(v) for k, v in pose["position"].items()},
        "orientation": {k: float(v) for k, v in pose["orientation"].items()},
    }


def _horizontal_distance(a: Dict[str, Any], b: Dict[str, Any]) -> float:
    dx = float(a["position"]["x"]) - float(b["position"]["x"])
    dy = float(a["position"]["y"]) - float(b["position"]["y"])
    return math.hypot(dx, dy)


def _hover_pose(down: Dict[str, Any], surface_z: float, clear_mm: float) -> Dict[str, Any]:
    """按压点正上方：同 x/y，z = 键面 + 抬升量，姿态沿用该点示教姿态。"""
    return {
        "position": {
            "x": float(down["position"]["x"]),
            "y": float(down["position"]["y"]),
            "z": float(surface_z) + float(clear_mm) / 1000.0,
        },
        "orientation": dict(down["orientation"]),
    }


def _segment(target: Dict[str, Any], mode: str, speed: float, sleep_s: float,
             hold_orientation: bool,
             via: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
    seg: Dict[str, Any] = {
        "target": target,
        "mode": mode,
        "speed": float(speed),
        "hold_orientation": bool(hold_orientation),
    }
    if sleep_s > 0.0:
        seg["sleep"] = round(float(sleep_s), 4)
    if via:
        seg["via"] = via
    return seg


def _via(pose: Dict[str, Any], exact: bool = False) -> Dict[str, Any]:
    v: Dict[str, Any] = {"pose": pose}
    if exact:
        v["exact"] = True
    return v


def build_press_stream(
    waypoints: Sequence[Dict[str, Any]],
    *,
    joint_names: Optional[Sequence[str]] = None,
    press_mm: float = PRESS_FLOOR_MM,
    speed: float = SPEED_DEFAULT,
    dwell_s: float = DWELL_DEFAULT_S,
    alpha: float = ALPHA_DEFAULT,
    beta: float = BETA_DEFAULT,
    clear_min_mm: float = CLEAR_MIN_MM,
    clear_max_mm: float = CLEAR_MAX_MM,
    mode: str = MODE_DEFAULT,
    hold_orientation: bool = True,
    home_positions: Optional[Sequence[float]] = None,
    waypoints_pressed: bool = True,
    target_exact: bool = False,
) -> dict:
    """把「按下点」路点序列编排为一条 stream。

    ``waypoints``：``[{pose: {position, orientation}, sleep?: float}, ...]``。
    ``home_positions``：结束段目标（关节值）；非关节模式或为空时改为抬到末键上方。
    """
    if not waypoints:
        raise ValueError("waypoints is empty")
    press_m = max(float(press_mm), PRESS_FLOOR_MM) / 1000.0
    clear_min_mm = max(0.0, float(clear_min_mm))
    clear_max_mm = max(clear_min_mm, float(clear_max_mm))

    downs: List[Dict[str, Any]] = []
    surfaces: List[float] = []
    sleeps: List[float] = []
    for wp in waypoints:
        pose = _copy_pose(wp["pose"])
        z = float(pose["position"]["z"])
        if waypoints_pressed:
            down_z, surface_z = z, z + press_m
        else:
            down_z, surface_z = z - press_m, z
        pose["position"]["z"] = down_z
        downs.append(pose)
        surfaces.append(surface_z)
        sleeps.append(float(wp.get("sleep", dwell_s)))

    def clear_of(d_m: float, ratio: float) -> float:
        return _clamp(ratio * d_m * 1000.0, clear_min_mm, clear_max_mm)

    def pair_clears(i: int) -> tuple:
        """第 i-1 → i 两键的（源/目标正上方, 中垂线上方）抬升量（mm）。"""
        d = _horizontal_distance(downs[i - 1], downs[i])
        c = clear_of(d, alpha)
        apex = max(clear_of(d, beta), c)
        return c, apex

    segments: List[Dict[str, Any]] = []

    # 进场：当前位 → 第 1 键正上方（exact，先停稳）→ 垂直下压
    c0 = pair_clears(1)[0] if len(downs) > 1 else clear_min_mm
    hover0 = _hover_pose(downs[0], surfaces[0], c0)
    segments.append(_segment(
        {"pose": downs[0]}, mode, speed, sleeps[0], hold_orientation,
        via=[_via(hover0, exact=True)],
    ))

    # 键间：源正上方 → 中垂线上方 → 目标正上方 → 目标按压点
    for i in range(1, len(downs)):
        c, apex = pair_clears(i)
        mid_surface = (surfaces[i - 1] + surfaces[i]) / 2.0
        mid_xy = (
            (float(downs[i - 1]["position"]["x"]) + float(downs[i]["position"]["x"])) / 2.0,
            (float(downs[i - 1]["position"]["y"]) + float(downs[i]["position"]["y"])) / 2.0,
        )
        apex_pose = {
            "position": {"x": mid_xy[0], "y": mid_xy[1],
                         "z": mid_surface + apex / 1000.0},
            "orientation": dict(downs[i - 1]["orientation"]),
        }
        vias = [
            # 源键上方：blend 切角（clear≥20mm，离开键面后才开始横移，不会拖键）
            _via(_hover_pose(downs[i - 1], surfaces[i - 1], c)),
            # 中垂线弧顶：blend 切角
            _via(apex_pose),
            # 目标键上方：默认 blend。clear≥20mm 时圆角切点在键面上方 ~8mm 以上，
            # 之后才是竖直下压；如需极限保守可开 target_exact（多一次停车）。
            _via(_hover_pose(downs[i], surfaces[i], c), exact=target_exact),
        ]
        segments.append(_segment(
            {"pose": downs[i]}, mode, speed, sleeps[i], hold_orientation, via=vias))

    # 离场：末键正上方（exact，垂直抬起）→ 再抬高 → home（或停在末键上方）
    c_last, apex_last = pair_clears(len(downs) - 1) if len(downs) > 1 else (clear_min_mm,
                                                                           clear_min_mm)
    hover_last = _hover_pose(downs[-1], surfaces[-1], c_last)
    retract = _hover_pose(downs[-1], surfaces[-1], c_last + apex_last)
    use_home = home_positions is not None and len(home_positions) > 0 and mode in JOINT_MODES
    if use_home:
        segments.append(_segment(
            {"positions": [float(v) for v in home_positions]}, mode, speed, 0.0,
            hold_orientation,
            via=[_via(hover_last, exact=True), _via(retract)],
        ))
    else:
        # 无 home：垂直拔到末键正上方（exact 停稳）再升到安全高度收尾
        segments.append(_segment(
            {"pose": retract}, mode, speed, 0.0, hold_orientation,
            via=[_via(hover_last, exact=True)],
        ))

    return {
        "kind": "stream",
        "joint_names": list(joint_names or []),
        "segments": segments,
    }


def header_lines(
    *,
    source: str,
    press_mm: float,
    alpha: float,
    beta: float,
    clear_min_mm: float,
    clear_max_mm: float,
    speed: float,
    dwell_s: float,
    mode: str,
    hold_orientation: bool,
    waypoints_pressed: bool,
    home_note: str,
    target_exact: bool = False,
) -> List[str]:
    bar = "# " + "=" * 74
    return [
        bar,
        "# 口风琴按键 stream —— 由 melodion_stream.build_press_stream() 生成，可整份重生成。",
        f"# 源路点：{source}",
        "# 路径：起手到第 1 键正上方（先停稳）→ 垂直下压 → 键间 3 过点弧 → … → "
        "末键垂直抬起 → home。",
        f"# 参数：press={press_mm}mm, alpha={alpha}, beta={beta}, "
        f"clear={clear_min_mm}~{clear_max_mm}mm, speed={speed}, "
        f"dwell={dwell_s}s, mode={mode}, hold_orientation={hold_orientation}, "
        f"waypoints_pressed={waypoints_pressed}, target_exact={target_exact}",
        f"# 收尾：{home_note}",
        "# hold_orientation: true → 各点用自身示教姿态做 6D IK（防横压）；"
        "false → position-only IK。",
        "# exact 悬停：进场/离场（停稳后垂直插入/拔出）；"
        "键间 3 个 via（源上方/弧顶/目标上方）默认全部 blend 切角。",
        bar,
    ]


def write_stream(path: str, doc: dict, header: Optional[Sequence[str]] = None) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for line in header or []:
            f.write(line + "\n")
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=False)
