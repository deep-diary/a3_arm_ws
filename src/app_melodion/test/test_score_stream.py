"""Pure-logic tests for melodion key config + score -> stream trajectory config（无 ROS 依赖）."""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from app_melodion.key_config import MelodionConfig  # noqa: E402
from app_melodion.melodion_score2stream import build_stream  # noqa: E402
from app_melodion.score import REST, load_score  # noqa: E402

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
