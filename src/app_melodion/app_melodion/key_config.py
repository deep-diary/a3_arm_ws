"""Melodion key config + playing geometry -> per-key up/down Cartesian poses."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

DEFAULT_JOINT_NAMES = [
    "L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint", "L7_joint",
]


@dataclass
class Key:
    index: int
    name: str
    label: str
    type: str
    midi_note: int
    solfa: Optional[str]
    jianpu: Optional[int]
    x_mm: float


class MelodionConfig:
    """加载合并键位 + 弹奏几何，并提供每键 up/down 的 base_link 笛卡尔位姿。"""

    def __init__(self, keys_json_path: Optional[str] = None,
                 geometry_yaml_path: Optional[str] = None) -> None:
        self._load_keys(keys_json_path)
        self._load_geometry(geometry_yaml_path)

    # ---- 键位 ----
    def _load_keys(self, path: Optional[str]) -> None:
        if path is None:
            path = os.path.join(
                get_package_share_directory("app_melodion"),
                "config", "melodion_keys.json",
            )
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.instrument = data.get("instrument", {})
        self.unit = data.get("unit", "mm")
        self.keys: List[Key] = [Key(**k) for k in data["keys"]]
        self._by_name: Dict[str, Key] = {k.name: k for k in self.keys}
        self._by_midi: Dict[int, Key] = {k.midi_note: k for k in self.keys}

    def get_key(self, name: str) -> Optional[Key]:
        return self._by_name.get(name)

    def key_span_mm(self) -> float:
        return self.keys[-1].x_mm - self.keys[0].x_mm

    # ---- 弹奏几何 ----
    def _load_geometry(self, path: Optional[str]) -> None:
        if path is None:
            path = os.path.join(
                get_package_share_directory("app_melodion"),
                "config", "playing_geometry.yaml",
            )
        with open(path, "r", encoding="utf-8") as f:
            self.geometry = yaml.safe_load(f)

    def _base_position(self) -> Point:
        p = self.geometry["base_pose"]["position"]
        return Point(x=float(p["x"]), y=float(p["y"]), z=float(p["z"]))

    def _orientation(self) -> Quaternion:
        o = self.geometry["base_pose"]["orientation"]
        return Quaternion(x=float(o["x"]), y=float(o["y"]),
                          z=float(o["z"]), w=float(o["w"]))

    def key_pose(self, key: Key, state: str) -> PoseStamped:
        """返回某键 up/down 在 base_link 下的位姿（末端姿态固定朝下）。

        键盘局部系：x 沿键盘低音→高音、y 沿键长（黑键朝 -y 后退）、z 向上。
        映射：axis=="x" 时键盘沿 base x（后退沿 base -y）；axis=="y" 时键盘沿
        base y（后退沿 base -x）。
        """
        g = self.geometry
        press = float(g.get("press_depth_mm", 8.0))
        approach = float(g.get("approach_height_mm", 20.0))
        retreat = float(g.get("black_retreat_mm", 12.0))
        white_y = float(g.get("white_y_mm", 0.0))
        axis = g.get("keyboard", {}).get("axis", "x")
        direction = float(g.get("keyboard", {}).get("direction", 1))

        koff_m = direction * key.x_mm / 1000.0
        lateral_mm = (white_y - retreat) if key.type == "black" else white_y
        lateral_m = lateral_mm / 1000.0

        base = self._base_position()
        if axis == "y":
            pos = Point(x=base.x - lateral_m, y=base.y + koff_m, z=base.z)
        else:
            pos = Point(x=base.x + koff_m, y=base.y + lateral_m, z=base.z)

        # 下压/抬起：z 相对键面（base.z）增减
        pos.z = base.z + (approach if state == "up" else -press) / 1000.0

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position = pos
        pose.pose.orientation = self._orientation()
        return pose

    def key_hover_pose(self, key: Key) -> PoseStamped:
        return self.key_pose(key, "up")

    def key_down_pose(self, key: Key) -> PoseStamped:
        return self.key_pose(key, "down")
