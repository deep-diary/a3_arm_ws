"""Pure-logic tests for melodion key config + score -> trajectory (no Pinocchio needed)."""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from app_melodion.key_config import MelodionConfig  # noqa: E402
from app_melodion.score import load_score, validate_score  # noqa: E402
from app_melodion.trajectory import build_trajectory  # noqa: E402

KEYS = os.path.join(_PKG, "config", "melodion_keys.json")
GEO = os.path.join(_PKG, "config", "playing_geometry.yaml")
SCORES = os.path.join(_PKG, "config", "scores")


class FakeIK:
    """无 Pinocchio 的假 IK：任意位姿返回固定 6 关节解，只记录调用点用于断言。"""

    def __init__(self):
        self.calls = []

    def solve(self, pose, seed=None, position_only=False):
        self.calls.append((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
        return True, "ok", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


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
    assert down.pose.position.z < up.pose.position.z  # 下压更低
    # 黑键沿键长方向后退（垂直于键盘轴向有偏移）
    fs3 = cfg.key_pose(cfg.get_key("F#3"), "up")
    axis = cfg.geometry["keyboard"].get("axis", "x")
    if axis == "y":
        assert abs(fs3.pose.position.x - up.pose.position.x) > 1e-4
    else:
        assert abs(fs3.pose.position.y - up.pose.position.y) > 1e-4


def test_validate_scores():
    cfg = MelodionConfig(KEYS, GEO)
    for name in ("scale", "qinghuaci"):
        sc = load_score(os.path.join(SCORES, name + ".yaml"))
        ok, msg = validate_score(sc, cfg)
        assert ok, msg


def test_build_scale_trajectory():
    cfg = MelodionConfig(KEYS, GEO)
    sc = load_score(os.path.join(SCORES, "scale.yaml"))
    fake = FakeIK()
    ok, traj = build_trajectory(cfg, fake, sc, [0.0] * 7)
    assert ok, traj
    # 1 起点 + 1 接近 + 32 键 × (hover/down/hold/up 共 4) = 130
    assert len(traj.points) == 130
    assert list(traj.joint_names) == list(cfg.geometry["joint_names"])
    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in traj.points]
    assert times[0] == 0.0
    assert all(times[i] <= times[i + 1] for i in range(len(times) - 1))
