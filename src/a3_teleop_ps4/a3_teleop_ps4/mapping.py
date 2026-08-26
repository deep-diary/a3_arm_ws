"""Load DS4 layout + mapping YAML and decode sensor_msgs/Joy."""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Joy


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def package_config_path(*parts: str) -> str:
    share = get_package_share_directory("a3_teleop_ps4")
    return os.path.join(share, "config", *parts)


class Ds4Layout:
    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.deadzone = float(cfg.get("deadzone", 0.12))
        self.trigger_rest = float(cfg.get("trigger_rest", 1.0))
        self.trigger_pressed = float(cfg.get("trigger_pressed", -1.0))
        self.invert_axes: Dict[str, bool] = dict(cfg.get("invert_axes") or {})
        self.buttons: Dict[str, int] = {
            str(k): int(v) for k, v in (cfg.get("buttons") or {}).items()
        }
        self.axes: Dict[str, int] = {
            str(k): int(v) for k, v in (cfg.get("axes") or {}).items()
        }
        self.virtual_buttons: Dict[str, Dict[str, Any]] = dict(
            cfg.get("virtual_buttons") or {}
        )

    def axis_raw(self, joy: Joy, name: str, extra: Optional[Dict[str, float]] = None) -> float:
        if extra and name in extra:
            v = float(extra[name])
        else:
            idx = self.axes.get(name)
            if idx is None or idx < 0 or idx >= len(joy.axes):
                return 0.0
            v = float(joy.axes[idx])
        if self.invert_axes.get(name):
            v = -v
        return v

    def axis_n11(self, joy: Joy, name: str, extra: Optional[Dict[str, float]] = None) -> float:
        v = self.axis_raw(joy, name, extra)
        if abs(v) < self.deadzone:
            return 0.0
        return max(-1.0, min(1.0, v))

    def trigger_01(self, joy: Joy, name: str) -> float:
        v = self.axis_raw(joy, name)
        span = self.trigger_rest - self.trigger_pressed
        if abs(span) < 1e-9:
            return 0.0
        t = (self.trigger_rest - v) / span
        return max(0.0, min(1.0, t))

    def button(self, joy: Joy, name: str) -> bool:
        if name in self.virtual_buttons:
            return self._virtual(joy, name)
        idx = self.buttons.get(name)
        if idx is None or idx < 0 or idx >= len(joy.buttons):
            return False
        return int(joy.buttons[idx]) == 1

    def _virtual(self, joy: Joy, name: str) -> bool:
        spec = self.virtual_buttons[name]
        axis = str(spec.get("axis", ""))
        v = self.axis_raw(joy, axis)
        th = float(spec.get("threshold", 0.5))
        op = str(spec.get("op", "gt"))
        if op == "lt":
            return v < th
        if op == "abs_gt":
            return abs(v) > abs(th)
        return v > th


def analog_01_from_axis(
    layout: Ds4Layout, joy: Joy, axis_name: str, source: str
) -> float:
    if source == "trigger":
        return layout.trigger_01(joy, axis_name)
    v = layout.axis_n11(joy, axis_name)
    return max(0.0, min(1.0, abs(v) if source == "abs" else (v + 1.0) * 0.5))


class ButtonEdgeTracker:
    """Rising-edge and long-press detectors keyed by binding name."""

    def __init__(self) -> None:
        self._prev: Dict[str, bool] = {}
        self._press_t: Dict[str, Optional[float]] = {}
        self._fired: Dict[str, bool] = {}

    def reset(self) -> None:
        self._prev.clear()
        self._press_t.clear()
        self._fired.clear()

    def rising(self, key: str, held: bool) -> bool:
        prev = self._prev.get(key, False)
        self._prev[key] = held
        return held and not prev

    def longpress(self, key: str, held: bool, now: float, hold_s: float) -> bool:
        if held:
            t0 = self._press_t.get(key)
            if t0 is None:
                self._press_t[key] = now
                self._fired[key] = False
                return False
            if not self._fired.get(key, False) and now - t0 >= hold_s:
                self._fired[key] = True
                return True
            return False
        self._press_t[key] = None
        self._fired[key] = False
        return False


def validate_mapping(
    mapping: Dict[str, Any], registry: Dict[str, Any]
) -> List[str]:
    fns = (registry.get("functions") or {}) if registry else {}
    errors: List[str] = []

    def check(fn: str, kind: str, where: str) -> None:
        spec = fns.get(fn)
        if not spec:
            errors.append(f"{where}: unknown function {fn}")
            return
        want = str(spec.get("kind", ""))
        if want and want != kind:
            errors.append(f"{where}: {fn} is {want}, binding says {kind}")

    for axis, spec in (mapping.get("axes") or {}).items():
        check(str(spec.get("fn")), str(spec.get("kind", "")), f"axes.{axis}")
    for btn, spec in (mapping.get("buttons") or {}).items():
        check(str(spec.get("fn")), "discrete", f"buttons.{btn}")
    for btn, spec in (mapping.get("held") or {}).items():
        check(str(spec.get("fn")), str(spec.get("kind", "analog_01")), f"held.{btn}")
    return errors


def iter_axis_bindings(mapping: Dict[str, Any]) -> List[Tuple[str, Dict[str, Any]]]:
    return list((mapping.get("axes") or {}).items())


def iter_button_bindings(mapping: Dict[str, Any]) -> List[Tuple[str, Dict[str, Any]]]:
    return list((mapping.get("buttons") or {}).items())


def iter_held_bindings(mapping: Dict[str, Any]) -> List[Tuple[str, Dict[str, Any]]]:
    return list((mapping.get("held") or {}).items())
