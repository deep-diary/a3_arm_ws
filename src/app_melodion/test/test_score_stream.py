"""Pure-logic tests for melodion key config + score -> stream trajectory config（无 ROS 依赖）."""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from app_melodion.key_config import MelodionConfig  # noqa: E402
from app_melodion import key_layout  # noqa: E402
from app_melodion.melodion_score2stream import build_stream  # noqa: E402
from app_melodion.score import REST, load_score  # noqa: E402

_WS = os.path.dirname(os.path.dirname(_PKG))
_CTRL = os.path.join(_WS, "src", "a3_arm_controller", "a3_arm_controller")
if _CTRL not in sys.path:
    sys.path.insert(0, _CTRL)
from melodion_stream import build_press_stream  # noqa: E402

KEYS = os.path.join(_PKG, "config", "melodion_keys.json")
GEO = os.path.join(_PKG, "config", "playing_geometry.yaml")
SCORES = os.path.join(_PKG, "config", "scores")


def test_key_span_and_counts():
    cfg = MelodionConfig(KEYS, GEO)
    assert len(cfg.keys) == 32
    assert abs(cfg.key_span_mm() - 340.2) < 1e-6
    assert len([k for k in cfg.keys if k.type == "white"]) == 19
    assert len([k for k in cfg.keys if k.type == "black"]) == 13


def test_key_pose_up_down_retreat():
    cfg = MelodionConfig(KEYS, GEO)
    f3 = cfg.get_key("F3")
    up = cfg.key_pose(f3, "up")
    down = cfg.key_pose(f3, "down")
    assert down["position"]["z"] < up["position"]["z"]  # 下压更低
    fs3 = cfg.key_pose(cfg.get_key("F#3"), "up")
    axis = cfg.geometry["keyboard"].get("axis", "x")
    if axis == "y":
        assert abs(fs3["position"]["x"] - up["position"]["x"]) > 1e-4  # 黑键后退
    else:
        assert abs(fs3["position"]["y"] - up["position"]["y"]) > 1e-4


def test_transition_via_midpoint():
    cfg = MelodionConfig(KEYS, GEO)
    k0, k1 = cfg.keys[0], cfg.keys[1]
    via = cfg.transition_via_pose(k0, k1)
    d_from = cfg.key_down_pose(k0)["position"]
    d_to = cfg.key_down_pose(k1)["position"]
    # 中垂线弧顶：水平取两按压点中点、高度取 hover
    assert abs(via["position"]["x"] - (d_from["x"] + d_to["x"]) / 2) < 1e-9
    assert abs(via["position"]["y"] - (d_from["y"] + d_to["y"]) / 2) < 1e-9
    base_z = cfg._base_position()["z"]
    approach = float(cfg.geometry.get("approach_height_mm", 20.0))
    assert abs(via["position"]["z"] - (base_z + approach / 1000.0)) < 1e-9


def test_build_stream_scale():
    cfg = MelodionConfig(KEYS, GEO)
    sc = load_score(os.path.join(SCORES, "scale.yaml"))
    doc = build_stream(cfg, sc, 0.5)
    assert doc["kind"] == "stream"
    assert len(doc["joint_names"]) == 7
    # scale 32 音符、无 REST → 32 音符段 + 1 末段抬离 = 33 段（F163 过 via 单条弧）
    assert len(doc["segments"]) == 33
    n = len(doc["segments"])
    for i, seg in enumerate(doc["segments"]):
        assert "pose" in seg["target"]
        assert "position" in seg["target"]["pose"]
        assert "orientation" in seg["target"]["pose"]
        assert seg["mode"] == "ptp"
        assert "speed" in seg and "sleep" in seg
        if 0 < i < n - 1:
            # 跨键段含 via（中垂线弧顶过点），且抬升高度 > 目标按压点高度
            assert "via" in seg and len(seg["via"]) == 1
            assert "pose" in seg["via"][0]
            assert seg["via"][0]["pose"]["position"]["z"] > seg["target"]["pose"]["position"]["z"]


def test_build_stream_qinghuaci_rest():
    cfg = MelodionConfig(KEYS, GEO)
    sc = load_score(os.path.join(SCORES, "qinghuaci.yaml"))
    doc = build_stream(cfg, sc, 0.5)
    real = [n for n in sc["notes"] if n["name"] != REST]
    # 段数 = 音符数 + 1（末段抬离）；休止累加到上一段 sleep
    assert len(doc["segments"]) == len(real) + 1
    # 休止符累加到上一段 sleep（按键位停顿）
    assert any(seg["sleep"] > 0 for seg in doc["segments"])


def _fake_extremes(cfg: MelodionConfig) -> dict:
    """用静态几何伪造 4 极限键路点（顺序 white1/white19/black13/black1）."""
    names = ("F3", "C6", "A#5", "F#3")
    return {
        "kind": "waypoint",
        "joint_names": ["L1_joint", "L2_joint", "L3_joint",
                        "L4_joint", "L5_joint", "L6_joint"],
        "waypoints": [{"pose": cfg.key_down_pose(cfg.get_key(n))} for n in names],
    }


def test_inspection_order_and_count():
    keys = key_layout.load_keys(KEYS)
    order = key_layout.inspection_order(keys)
    assert len(order) == 32
    assert [k["type"] for k in order[:19]] == ["white"] * 19
    assert [k["type"] for k in order[19:]] == ["black"] * 13
    assert order[0]["name"] == "F3" and order[18]["name"] == "C6"
    assert order[19]["name"] == "A#5" and order[-1]["name"] == "F#3"


def test_inspection_waypoints_from_extremes():
    cfg = MelodionConfig(KEYS, GEO)
    keys = key_layout.load_keys(KEYS)
    layout = key_layout.build_layout(_fake_extremes(cfg), keys)
    wps = key_layout.inspection_waypoints(layout, keys, dwell_s=0.2)
    assert len(wps) == 32

    pos = [wp["pose"]["position"] for wp in wps]
    u = layout["u"]
    # 白键 1→19 沿键盘正向单调；黑键 13→1 反向单调
    def along(a, b):
        return (b["x"] - a["x"]) * u[0] + (b["y"] - a["y"]) * u[1]

    assert all(along(pos[i], pos[i + 1]) > 0 for i in range(18))
    assert all(along(pos[i], pos[i + 1]) < 0 for i in range(19, 31))
    # 所有按下点 z 落在 4 个极限点 z 的区间内（按类型插值，不外推）
    zs = [wp["pose"]["position"]["z"] for wp in _fake_extremes(cfg)["waypoints"]]
    assert all(min(zs) - 1e-9 <= p["z"] <= max(zs) + 1e-9 for p in pos)
    # 每键按住时长透传
    assert all(abs(wp["sleep"] - 0.2) < 1e-9 for wp in wps)


def test_build_press_stream_inspection_segments():
    cfg = MelodionConfig(KEYS, GEO)
    keys = key_layout.load_keys(KEYS)
    layout = key_layout.build_layout(_fake_extremes(cfg), keys)
    wps = key_layout.inspection_waypoints(layout, keys)
    doc = build_press_stream(wps, joint_names=["L1_joint"], home_positions=None)
    # 32 个按键段 + 1 个末段抬离
    assert len(doc["segments"]) == 33
    assert doc["segments"][0]["target"]["pose"]["position"]["x"] == wps[0]["pose"]["position"]["x"]
