"""Melodion key config + playing geometry -> per-key up/down Cartesian poses (纯 dict，无 ROS 依赖)."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional

import yaml

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
    """加载合并键位 + 弹奏几何，并提供每键 up/down 的 base_link 笛卡尔位姿（dict）。"""

    def __init__(self, keys_json_path: Optional[str] = None,
                 geometry_yaml_path: Optional[str] = None) -> None:
        self._load_keys(keys_json_path)
        self._load_geometry(geometry_yaml_path)

    @staticmethod
    def _default_path(rel: str) -> str:
        # 开发树：app_melodion/key_config.py -> ../config/<rel>
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "config", rel)

    # ---- 键位 ----
    def _load_keys(self, path: Optional[str]) -> None:
        if path is None:
            path = self._default_path("melodion_keys.json")
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
            path = self._default_path("playing_geometry.yaml")
        with open(path, "r", encoding="utf-8") as f:
            self.geometry = yaml.safe_load(f)

    def _base_position(self) -> Dict[str, float]:
        p = self.geometry["base_pose"]["position"]
        return {"x": float(p["x"]), "y": float(p["y"]), "z": float(p["z"])}

    def _orientation(self) -> Dict[str, float]:
        o = self.geometry["base_pose"]["orientation"]
        return {"x": float(o["x"]), "y": float(o["y"]),
                "z": float(o["z"]), "w": float(o["w"])}

    def key_pose(self, key: Key, state: str) -> Dict[str, object]:
        """返回某键 up/down 在 base_link 下的位姿 dict。

        结构：{"position": {"x","y","z"}, "orientation": {"x","y","z","w"}}。
        键盘局部系：x 沿键盘低音→高音、y 沿键长（黑键朝 -y 后退）、z 向上。
        axis=="x" 时键盘沿 base x（后退沿 base -y）；axis=="y" 时键盘沿 base y（后退沿 base -x）。
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
            pos = {"x": base["x"] - lateral_m, "y": base["y"] + koff_m, "z": base["z"]}
        else:
            pos = {"x": base["x"] + koff_m, "y": base["y"] + lateral_m, "z": base["z"]}

        # 下压/抬起：z 相对键面（base.z）增减
        pos["z"] = base["z"] + (approach if state == "up" else -press) / 1000.0

        return {"position": pos, "orientation": self._orientation()}

    def key_hover_pose(self, key: Key) -> Dict[str, object]:
        return self.key_pose(key, "up")

    def key_down_pose(self, key: Key) -> Dict[str, object]:
        return self.key_pose(key, "down")

    def transition_via_pose(self, key_from: Key, key_to: Key) -> Dict[str, object]:
        """返回 key_from 按压点 → key_to 按压点的「中间抬高点」via 位姿（F163）。

        策略取 geometry['via_apex']['strategy']：
          midpoint       —— 两按压点水平中点（中垂线弧顶，默认，方式一）
          target_above   —— 目标键正上方（方式二）
          source_above   —— 当前键正上方
        z 取 base.z + via 高度（via_apex.height_mm，缺省用 approach_height_mm）。
        """
        g = self.geometry
        via_cfg = g.get("via_apex", {}) or {}
        strategy = str(via_cfg.get("strategy", "midpoint")).strip().lower()
        height_mm = via_cfg.get("height_mm")
        approach = float(height_mm) if height_mm else float(g.get("approach_height_mm", 20.0))

        p_from = self.key_down_pose(key_from)["position"]
        p_to = self.key_down_pose(key_to)["position"]
        base_z = self._base_position()["z"]

        if strategy == "target_above":
            px, py = float(p_to["x"]), float(p_to["y"])
        elif strategy == "source_above":
            px, py = float(p_from["x"]), float(p_from["y"])
        else:  # midpoint（默认，方式一）
            px = (float(p_from["x"]) + float(p_to["x"])) / 2.0
            py = (float(p_from["y"]) + float(p_to["y"])) / 2.0

        return {
            "position": {"x": px, "y": py, "z": base_z + approach / 1000.0},
            "orientation": self._orientation(),
        }
