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
        initial_scale = float(mapping.get("speed_normal", 0.35))
        self._exec.linear_scale = initial_scale
        self._exec.angular_scale = initial_scale
        self._joy: Optional[Joy] = None
        self._joy_received_at = 0.0
        self._joy_alive_ticks = 0
        self._joy_alive_threshold = int(mapping.get("joy_alive_ticks", 25))
        self._extra: Dict[str, float] = {}
        # F64：hat 轴各方向 0→±1 边沿只触发一次，回中后才能再步（不连发）
        self._dpad_active: Dict[str, int] = {}
        self._auto_servo = bool(self.get_parameter("auto_start_servo").value)
        self._gyro_scale = float(self.get_parameter("gyro_scale").value)
        self._servo_started = False

        self.create_subscription(
            Joy, str(self.get_parameter("joy_topic").value), self._on_joy, 10
        )
        self.create_subscription(Imu, "/a3/ds4/imu", self._on_imu, 10)
        self.create_subscription(Vector3, "/a3/ds4/touch", self._on_touch, 10)

        rate = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate, 1.0)
        self.create_timer(self._dt, self._on_timer)
        self.get_logger().info(
            f"ps4_mapper ready mapping={self.get_parameter('mapping_file').value} "
            f"twist_frame={twist_frame}"
        )

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg
        self._joy_received_at = self.get_clock().now().nanoseconds * 1e-9

    def _on_imu(self, msg: Imu) -> None:
        s = max(self._gyro_scale, 1e-3)
        self._extra["gyro_roll"] = _clamp(msg.angular_velocity.x / s)
        self._extra["gyro_pitch"] = _clamp(msg.angular_velocity.y / s)
        self._extra["gyro_yaw"] = _clamp(msg.angular_velocity.z / s)

    def _on_touch(self, msg: Vector3) -> None:
        self._extra["touch_x"] = _clamp(float(msg.x))
        self._extra["touch_y"] = _clamp(float(msg.y))

    def _joy_ready(self) -> bool:
        return self._joy is not None and self._joy_alive_ticks >= self._joy_alive_threshold

    def _maybe_start_servo(self) -> None:
        if not self._auto_servo or self._servo_started or not self._joy_ready():
            return
        if self._exec.try_start_servo():
            self._servo_started = True
            self.get_logger().info("servo started after joy became ready")

    def _on_timer(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._joy is None or (now - self._joy_received_at) > 1.0:
            self._joy_alive_ticks = 0
            return

        self._joy_alive_ticks += 1
        if not self._joy_ready():
            return

        self._maybe_start_servo()
        self._exec.poll(now)
        joy = self._joy
        self._exec.tick_begin()
        pose_block = self._exec.pose_blocking(now)

        # F64：逐轴 gates——轴任一 gate 按住才取样；未列 gates 的轴不门控（R2 夹爪）
        if not pose_block:
            for axis_name, spec in iter_axis_bindings(self._mapping):
                gates = spec.get("gates") or []
                if gates and not any(self._layout.button(joy, g) for g in gates):
                    continue
                fn = str(spec.get("fn"))
                kind = str(spec.get("kind", "analog_n11"))
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

        for bind_key, btn, spec in iter_button_bindings(self._mapping):
            held = self._layout.button(joy, btn)
            edge = str(spec.get("edge", "rising"))
            fire = False
            if edge == "longpress":
                fire = self._edges.longpress(
                    bind_key, held, now, float(spec.get("hold_s", 1.0))
                )
            elif edge == "shortpress":
                fire = self._edges.shortpress(
                    bind_key, held, now, float(spec.get("hold_s", 3.0))
                )
            else:
                fire = self._edges.rising(bind_key, held)
            if fire:
                kwargs = dict(spec.get("kwargs") or {})
                self._exec.apply_discrete(str(spec.get("fn")), kwargs)

        for dspec in self._mapping.get("dpad") or []:
            axis_name = str(dspec.get("axis", ""))
            raw = self._layout.axis_raw(joy, axis_name)
            sign = 1 if raw > 0.5 else (-1 if raw < -0.5 else 0)
            if sign != 0 and self._dpad_active.get(axis_name, 0) == 0:
                entry = dspec.get("pos" if sign > 0 else "neg") or {}
                kwargs = dict(entry.get("kwargs") or {})
                self._exec.apply_discrete(str(entry.get("fn")), kwargs)
            self._dpad_active[axis_name] = sign

        self._exec.tick_end(now, not pose_block, self._dt)


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
