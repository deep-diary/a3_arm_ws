#!/usr/bin/env python3
"""YAML-driven PS4 mapper: axes → analog_01/n11 functions, buttons → discrete."""

from __future__ import annotations

from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy

from a3_teleop_ps4.actions import ActionExecutor
from a3_teleop_ps4.mapping import (
    ButtonEdgeTracker,
    Ds4Layout,
    analog_01_from_axis,
    iter_axis_bindings,
    iter_button_bindings,
    iter_held_bindings,
    load_yaml,
    package_config_path,
    validate_mapping,
)


class Ps4Mapper(Node):
    def __init__(self) -> None:
        super().__init__("ps4_mapper")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("layout_file", package_config_path("ds4_linux.yaml"))
        self.declare_parameter("registry_file", package_config_path("action_registry.yaml"))
        self.declare_parameter(
            "mapping_file", package_config_path("mappings", "default.yaml")
        )
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("auto_start_servo", True)
        self.declare_parameter("gyro_scale", 1.5)

        layout_cfg = load_yaml(str(self.get_parameter("layout_file").value))
        registry = load_yaml(str(self.get_parameter("registry_file").value))
        mapping = load_yaml(str(self.get_parameter("mapping_file").value))
        errs = validate_mapping(mapping, registry)
        for e in errs:
            self.get_logger().error(e)
        if errs:
            raise RuntimeError("invalid mapping YAML: " + "; ".join(errs))

        self._layout = Ds4Layout(layout_cfg)
        self._mapping = mapping
        self._edges = ButtonEdgeTracker()
        twist_frame = str(mapping.get("twist_frame", "base_link"))
        self._exec = ActionExecutor(self, twist_frame=twist_frame)
        self._exec.speed_scale = float(mapping.get("speed_normal", 0.35))
        self._joy: Optional[Joy] = None
        self._joy_ticks = 0
        self._input_armed = False
        self._warmup_ticks = int(mapping.get("joy_warmup_ticks", 30))
        self._dpad_arm_ticks = int(mapping.get("dpad_arm_ticks", 15))
        self._dpad_calm_ticks = 0
        self._extra: Dict[str, float] = {}
        deadman_cfg = mapping.get("deadman") or {}
        self._deadman_enabled = bool(deadman_cfg.get("enabled", True))
        self._deadman_name = str(deadman_cfg.get("button") or "l1")
        self._deadman_kinds = set(
            deadman_cfg.get("applies_to") or ["analog_01", "analog_n11"]
        )
        if not self._deadman_enabled:
            self.get_logger().info("deadman disabled — axes active without L1")
        self._auto_servo = bool(self.get_parameter("auto_start_servo").value)
        self._gyro_scale = float(self.get_parameter("gyro_scale").value)

        self.create_subscription(
            Joy, str(self.get_parameter("joy_topic").value), self._on_joy, 10
        )
        self.create_subscription(Imu, "/a3/ds4/imu", self._on_imu, 10)
        self.create_subscription(Vector3, "/a3/ds4/touch", self._on_touch, 10)

        rate = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate, 1.0)
        self.create_timer(self._dt, self._on_timer)
        self.create_timer(1.0, self._on_servo_retry)
        self.get_logger().info(
            f"ps4_mapper ready mapping={self.get_parameter('mapping_file').value} "
            f"twist_frame={twist_frame}"
        )

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg

    def _on_imu(self, msg: Imu) -> None:
        s = max(self._gyro_scale, 1e-3)
        self._extra["gyro_roll"] = _clamp(msg.angular_velocity.x / s)
        self._extra["gyro_pitch"] = _clamp(msg.angular_velocity.y / s)
        self._extra["gyro_yaw"] = _clamp(msg.angular_velocity.z / s)

    def _on_touch(self, msg: Vector3) -> None:
        self._extra["touch_x"] = _clamp(float(msg.x))
        self._extra["touch_y"] = _clamp(float(msg.y))

    def _on_servo_retry(self) -> None:
        if self._auto_servo:
            self._exec.try_start_servo()

    def _on_timer(self) -> None:
        if self._joy is None:
            return
        self._joy_ticks += 1
        joy = self._joy
        if not self._input_armed:
            if self._joy_ticks < self._warmup_ticks:
                return
            if self._dpad_centered(joy):
                self._dpad_calm_ticks += 1
            else:
                self._dpad_calm_ticks = 0
                self._edges.reset()
            if self._dpad_calm_ticks < self._dpad_arm_ticks:
                return
            self._input_armed = True
            self._edges.reset()
            self.get_logger().info(
                f"joy input armed after {self._joy_ticks} ticks "
                f"(warmup={self._warmup_ticks}, dpad_calm={self._dpad_arm_ticks})"
            )
        now = self.get_clock().now().nanoseconds * 1e-9
        deadman_pressed = self._layout.button(joy, self._deadman_name)
        motion_allowed = (not self._deadman_enabled) or deadman_pressed
        self._exec.tick_begin()

        for name, spec in iter_held_bindings(self._mapping):
            pressed = self._layout.button(joy, name)
            fn = str(spec.get("fn"))
            if pressed:
                self._exec.apply_analog(fn, float(spec.get("pressed", 1.0)))
            else:
                self._exec.apply_analog(fn, float(spec.get("released", 0.35)))

        pose_block = self._exec.pose_blocking(now)

        if motion_allowed and not pose_block:
            for axis_name, spec in iter_axis_bindings(self._mapping):
                fn = str(spec.get("fn"))
                kind = str(spec.get("kind", "analog_n11"))
                if (
                    self._deadman_enabled
                    and kind in self._deadman_kinds
                    and not deadman_pressed
                ):
                    continue
                if kind == "analog_01":
                    source = str(spec.get("source", "trigger" if axis_name in ("l2", "r2") else "unit"))
                    if axis_name in self._extra and source != "trigger":
                        v = max(0.0, min(1.0, (self._extra[axis_name] + 1.0) * 0.5))
                    else:
                        v = analog_01_from_axis(self._layout, joy, axis_name, source)
                    self._exec.apply_analog(fn, v)
                else:
                    extra = self._extra if axis_name in self._extra else None
                    v = self._layout.axis_n11(joy, axis_name, extra)
                    if spec.get("invert"):
                        v = -v
                    self._exec.apply_analog(fn, v)

        for btn, spec in iter_button_bindings(self._mapping):
            held = self._layout.button(joy, btn)
            edge = str(spec.get("edge", "rising"))
            fire = False
            if edge == "longpress":
                fire = self._edges.longpress(
                    btn, held, now, float(spec.get("hold_s", 1.0))
                )
            else:
                fire = self._edges.rising(btn, held)
            if fire:
                kwargs = dict(spec.get("kwargs") or {})
                self._exec.apply_discrete(str(spec.get("fn")), kwargs)

        self._exec.tick_end(now, motion_allowed and not pose_block, self._dt)

    def _dpad_centered(self, joy: Joy) -> bool:
        th = 0.35
        for axis in ("dpad_x", "dpad_y"):
            if abs(self._layout.axis_raw(joy, axis)) > th:
                return False
        return True


def _clamp(v: float) -> float:
    return max(-1.0, min(1.0, float(v)))


def main() -> None:
    rclpy.init()
    node = Ps4Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
