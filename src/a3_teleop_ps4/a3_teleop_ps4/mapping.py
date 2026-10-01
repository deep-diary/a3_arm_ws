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
        self._virt_held: Dict[str, bool] = {}

    def reset_virtual_buttons(self) -> None:
        self._virt_held.clear()

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
        th_on = float(spec.get("threshold", 0.5))
        th_off = float(spec.get("release_threshold", abs(th_on) * 0.6))
        op = str(spec.get("op", "gt"))
        held = self._virt_held.get(name, False)
        if op == "lt":
            if not held and v < th_on:
                held = True
            elif held and v > -th_off:
                held = False
        elif op == "abs_gt":
            if not held and abs(v) > abs(th_on):
                held = True
            elif held and abs(v) < abs(th_off):
                held = False
        else:
            if not held and v > th_on:
                held = True
            elif held and v < th_off:
                held = False
        self._virt_held[name] = held
        return held


def analog_01_from_axis(
    layout: Ds4Layout, joy: Joy, axis_name: str, source: str
) -> float:
    if source == "trigger":
        return layout.trigger_01(joy, axis_name)
    v = layout.axis_n11(joy, axis_name)
    return max(0.0, min(1.0, abs(v) if source == "abs" else (v + 1.0) * 0.5))


class ButtonEdgeTracker:
    """
    Rising-edge, short-press and long-press detectors keyed by binding name.

    同一物理按键的多条绑定用不同 key（iter_button_bindings 生成 `btn#i` 后缀），
    各自独立跟踪——这是「短按=功能 A / 长按=功能 B」双义的基础（F55）。
    """

    def __init__(self) -> None:
        self._prev: Dict[str, bool] = {}
        self._press_t: Dict[str, Optional[float]] = {}
        self._fired: Dict[str, bool] = {}
        self._overshoot: Dict[str, bool] = {}

    def reset(self) -> None:
        self._prev.clear()
        self._press_t.clear()
        self._fired.clear()
        self._overshoot.clear()

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

    def shortpress(self, key: str, held: bool, now: float, hold_s: float) -> bool:
        """
        释放时判定：按住持续时长 < hold_s → 触发一次；按住超时则本次作废.

        长按作废用 overshoot 标记（而不是直接不放行），保证释放帧既不误触发、
        也不影响其他同键绑定（key 唯一）。
        """
        if held:
            t0 = self._press_t.get(key)
            if t0 is None:
                self._press_t[key] = now
                self._overshoot[key] = False
            elif now - t0 >= hold_s:
                self._overshoot[key] = True
            return False
        t0 = self._press_t.pop(key, None)
        if t0 is None:
            return False
        overshot = self._overshoot.pop(key, False)
        if overshot or now - t0 >= hold_s:
            return False
        return True


class TouchGestureTracker:
    """
    F131/F132: 触摸板 tap 检测（输入 /a3/ds4/touch Vector3）。

    Vector3 语义（ds4_hid_node）：x/y 归一化坐标 [-1,1]，z=fingers（1=有接触/0=抬起）。
    tap 判定：接触时长 < max_dur_s 且接触期间位移 < max_move（归一化），抬起瞬间触发。
    蓝牙 HID 丢帧会导致 fingers 抖动；调用方可在 update() 前自行去抖，tracker
    对无 release 的异常中断不补发。
    """

    def __init__(self, max_dur_s: float = 0.4, max_move: float = 0.08) -> None:
        self.max_dur_s = float(max_dur_s)
        self.max_move = float(max_move)
        self._active = False
        self._t0: Optional[float] = None
        self._x0 = 0.0
        self._y0 = 0.0
        self._moved = False

    def reset(self) -> None:
        self._active = False
        self._t0 = None
        self._moved = False

    def update(self, x: float, y: float, fingers: float, now: float) -> bool:
        """喂入一帧触摸数据；抬起帧判定为 tap 时返回 True（仅一次）."""
        touching = fingers is not None and float(fingers) >= 0.5
        if touching:
            if not self._active:
                self._active = True
                self._t0 = now
                self._x0 = float(x)
                self._y0 = float(y)
                self._moved = False
            elif ((float(x) - self._x0) ** 2 + (float(y) - self._y0) ** 2) ** 0.5 > self.max_move:
                self._moved = True
            return False
        if not self._active:
            return False
        self._active = False
        t0 = self._t0
        self._t0 = None
        moved = self._moved
        if t0 is None or moved or (now - t0) >= self.max_dur_s:
            return False
        return True


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
        specs = spec if isinstance(spec, list) else [spec]
        for i, s in enumerate(specs):
            check(str(s.get("fn")), "discrete", f"buttons.{btn}[{i}]")
    for i, spec in enumerate(mapping.get("dpad") or []):
        for side in ("neg", "pos"):
            entry = spec.get(side) or {}
            if entry:
                check(str(entry.get("fn")), "discrete", f"dpad[{i}].{side}")
    for gesture, spec in (mapping.get("touch") or {}).items():
        check(str(spec.get("fn")), "discrete", f"touch.{gesture}")
    return errors


def iter_touch_bindings(
    mapping: Dict[str, Any],
) -> List[Tuple[str, Dict[str, Any]]]:
    """展开触摸手势绑定为 (gesture, spec)（F132；当前仅 tap）."""
    return list((mapping.get("touch") or {}).items())


def iter_axis_bindings(mapping: Dict[str, Any]) -> List[Tuple[str, Dict[str, Any]]]:
    return list((mapping.get("axes") or {}).items())


def iter_button_bindings(
    mapping: Dict[str, Any],
) -> List[Tuple[str, str, Dict[str, Any]]]:
    """
    展开按钮绑定为 (bind_key, button_name, spec) 三元组.

    - 普通 dict 条目：bind_key = button 名（single binding）；
    - list 条目（F55 双义，如 options 短按+长按）：逐条展开，
      bind_key = `{btn}#{i}` 保证同键多条绑定各自独立边沿跟踪。
    button_name 始终是原始 `btn`（用来从 /joy 取样）。
    """
    out: List[Tuple[str, str, Dict[str, Any]]] = []
    for btn, spec in (mapping.get("buttons") or {}).items():
        if isinstance(spec, list):
            for i, s in enumerate(spec):
                out.append((f"{btn}#{i}", str(btn), s))
        else:
            out.append((str(btn), str(btn), spec))
    return out
