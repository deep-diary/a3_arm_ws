#!/usr/bin/env python3
"""YAML-driven PS4 mapper: axes → analog_01/n11 functions, buttons → discrete.

F189 起支持"固定层 + 模式层"：`common` 里的绑定在所有模式生效，`modes.<name>`
可覆盖同名轴/键或屏蔽公共轴；**按住 Share + 方向/面键**直接点名模式，
Options 短按退出回 NORMAL。切模式前先调 `/a3/arm/stop` 停掉当前运动，
避免模式之间串扰（F190）。
"""

from __future__ import annotations

from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from a3_teleop_ps4.actions import ActionExecutor
from a3_teleop_ps4.mapping import (
    ButtonEdgeTracker,
    Ds4Layout,
    TouchGestureTracker,
    analog_01_from_axis,
    iter_axis_bindings,
    iter_button_bindings,
    iter_touch_bindings,
    load_yaml,
    mode_specs,
    package_config_path,
    resolve_mode,
    select_bindings,
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
        # F189：切模式时等 /a3/arm/stop 的最长时间（超时也继续切换，只 WARN）
        self.declare_parameter("mode_stop_timeout_s", 1.0)

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
        # F132: 触摸板手势跟踪器（无 touch 配置节则为空，缺省不启用）
        self._touch_trackers: Dict[str, TouchGestureTracker] = {}
        self._touch_specs: Dict[str, dict] = {}
        for gesture, tspec in iter_touch_bindings(mapping):
            self._touch_trackers[gesture] = TouchGestureTracker(
                max_dur_s=float(tspec.get("max_dur_s", 0.4)),
                max_move=float(tspec.get("max_move", 0.08)),
            )
            self._touch_specs[gesture] = tspec
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

        # ---- F189 模式状态 ----
        self._mode_names = mode_specs(mapping)
        self._select_map = select_bindings(mapping)
        select_spec = ((mapping.get("modes") or {}).get("select") or {})
        self._select_modifier = str(select_spec.get("modifier", "share"))
        self._exit_spec = dict((mapping.get("modes") or {}).get("exit") or {})
        self._mode: Optional[str] = None
        self._resolved: Dict[str, object] = {}
        if self._mode_names:
            self._mode = "normal" if "normal" in self._mode_names else \
                next(iter(self._mode_names))
            self._resolved = resolve_mode(mapping, self._mode)
        else:
            self._resolved = resolve_mode(mapping, "")
        # 负载优化：绑定表只在切模式时变化，50 Hz tick 不再每帧重建 list/元组/f-string
        self._axis_bindings = iter_axis_bindings(self._resolved)
        self._button_bindings = iter_button_bindings(self._resolved)
        self._servo_retry_at = 0.0
        self._select_latch: Optional[str] = None
        # F189：切模式时仍按住的键要先松开才生效，避免 Share+△ 的 △ 立刻又触发
        # 新模式里的 △ 绑定（"进 DEMO 却执行了别的动作"）
        self._held_latch: set[str] = set()
        self._pending_mode: Optional[str] = None
        self._pending_since = 0.0
        self._pending_run_exit = True
        self._pending_from_select = False
        self._pending_stop_future = None
        self._mode_guard_until = 0.0
        self._mode_t = 0.0            # 1 Hz 重发模式话题（晚启动的订阅者也能拿到）
        self._mode_pub = self.create_publisher(String, "/a3/teleop/mode", 10)
        self._stop_cli = self.create_client(Trigger, "/a3/arm/stop")

        self.create_subscription(
            Joy, str(self.get_parameter("joy_topic").value), self._on_joy, 10
        )
        # 50 Hz 传感器流用 BEST_EFFORT：发布端是 RELIABLE，兼容；避免可靠传输积压
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, "/a3/ds4/imu", self._on_imu, sensor_qos)
        self.create_subscription(Vector3, "/a3/ds4/touch", self._on_touch, sensor_qos)
        self.create_subscription(
            String, "/a3/teleop/mode", self._on_mode_echo,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

        rate = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate, 1.0)
        self.create_timer(self._dt, self._on_timer)
        if self._mode:
            self._publish_mode_state(self._mode)
        self.get_logger().info(
            f"ps4_mapper ready mapping={self.get_parameter('mapping_file').value} "
            f"twist_frame={twist_frame} mode={self._mode or 'legacy'}"
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
        if not self._touch_trackers:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for gesture, tracker in self._touch_trackers.items():
            if tracker.update(float(msg.x), float(msg.y), float(msg.z), now):
                spec = self._touch_specs[gesture]
                kwargs = dict(spec.get("kwargs") or {})
                self.get_logger().info(f"touch {gesture} detected -> {spec.get('fn')}")
                self._exec.apply_discrete(str(spec.get("fn")), kwargs)

    def _on_mode_echo(self, msg: String) -> None:
        """外部（launch/脚本）也可指定模式；与当前不同则走同一切换流程。"""
        target = str(msg.data).strip()
        if not target or target == self._mode or target not in self._mode_names:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self._switch_mode(target, now, run_exit=True, from_select=False)

    def _joy_ready(self) -> bool:
        return self._joy is not None and self._joy_alive_ticks >= self._joy_alive_threshold

    def _maybe_start_servo(self) -> None:
        if not self._auto_servo or self._servo_started or not self._joy_ready():
            return
        # service_is_ready() 是 graph 查询；servo 未起时不要 50 Hz 空查，退避 1 s
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._servo_retry_at:
            return
        self._servo_retry_at = now + 1.0
        if self._exec.try_start_servo():
            self._servo_started = True
            self.get_logger().info("servo started after joy became ready")

    # ------------------------------------------------------------ F189 模式
    def _handle_mode_select(self, joy: Joy, now: float) -> bool:
        """Share 按住期间：方向/面键直接点名模式。返回 True = 处于选模式窗口。"""
        if not self._select_map:
            return False
        if not self._layout.button(joy, self._select_modifier):
            self._select_latch = None
            return False
        for btn, target in self._select_map.items():
            if not self._layout.button(joy, btn) or self._select_latch == target:
                continue
            self._select_latch = target
            self.get_logger().info(f"选模式：{self._select_modifier}+{btn} → {target}")
            self._switch_mode(target, now, run_exit=True, from_select=True)
        return True

    def _handle_mode_exit(self, joy: Joy, now: float) -> None:
        spec = self._exit_spec
        if not spec or self._mode in (None, "normal"):
            return
        btn = str(spec.get("button", "options"))
        held = self._layout.button(joy, btn)
        hold_s = float(spec.get("hold_s", 1.5))
        if str(spec.get("edge", "shortpress")) == "shortpress":
            fire = self._edges.shortpress("mode_exit", held, now, hold_s)
        else:
            fire = self._edges.rising("mode_exit", held)
        if fire:
            self.get_logger().info(f"退出模式 {self._mode} → normal")
            self._switch_mode("normal", now, run_exit=True, from_select=False)

    def _switch_mode(self, target: str, now: float, run_exit: bool,
                     from_select: bool) -> None:
        if target not in self._mode_names or target == self._mode:
            return
        if self._pending_mode is not None:
            return
        self._pending_mode = target
        self._pending_since = now
        self._pending_run_exit = run_exit
        self._pending_from_select = from_select
        # 先停当前运动（编排层 stop → 回 READY），完成或超时后再提交模式
        self._pending_stop_future = None
        if self._stop_cli.service_is_ready():
            self._pending_stop_future = self._stop_cli.call_async(Trigger.Request())
        self._exec.stop_motion()

    def _check_pending_mode(self, now: float) -> None:
        if self._pending_mode is None:
            return
        fut = self._pending_stop_future
        if fut is not None:
            if not fut.done():
                if now - self._pending_since < float(
                        self.get_parameter("mode_stop_timeout_s").value):
                    return
                self.get_logger().warn("arm/stop 超时 → 仍切换模式")
            else:
                try:
                    resp = fut.result()
                    if not getattr(resp, "success", True):
                        self.get_logger().warn(
                            f"arm/stop 拒绝：{getattr(resp, 'message', '')}")
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"arm/stop 调用失败：{exc}")
        target = self._pending_mode
        run_exit = self._pending_run_exit
        from_select = self._pending_from_select
        self._pending_mode = None
        self._pending_stop_future = None
        self._commit_mode(target, now, run_exit, from_select)

    def _run_hooks(self, mode: str, which: str) -> bool:
        spec = self._mode_names.get(mode) or {}
        for hook in spec.get(which) or []:
            fn = str(hook.get("fn"))
            kwargs = dict(hook.get("kwargs") or {})
            required = bool(hook.get("required", False))
            method = getattr(self._exec, fn, None)
            if method is None:
                self.get_logger().error(f"模式 {mode} 的 {which} 钩子未知：{fn}")
                if required:
                    return False
                continue
            try:
                result = method(**kwargs) if kwargs else method()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"{mode}.{which}.{fn} 失败：{exc}")
                if required:
                    return False
                continue
            if required and result is False:
                self.get_logger().error(f"{mode}.{which}.{fn} 返回失败 → 拒绝进入")
                return False
        return True

    def _commit_mode(self, target: str, now: float, run_exit: bool,
                     from_select: bool) -> None:
        prev = self._mode
        if run_exit and prev:
            self._run_hooks(prev, "exit")
        self._mode = target
        self._resolved = resolve_mode(self._mapping, target)
        self._axis_bindings = iter_axis_bindings(self._resolved)
        self._button_bindings = iter_button_bindings(self._resolved)
        self._mode_guard_until = now + 0.3
        # 清边沿/摇杆/方向键状态：避免旧模式的按键在切换瞬间重复触发
        self._edges = ButtonEdgeTracker()
        self._dpad_active.clear()
        # 仍按住的键/方向键先锁存，松开后才允许触发（否则 Share+△ 会在新模式里
        # 立刻又执行一次 △ 的动作）
        self._held_latch = set()
        joy = self._joy
        if joy is not None:
            names = set(self._select_map) | set(self._resolved.get("buttons") or {})
            for name in names:
                if self._layout.button(joy, name):
                    self._held_latch.add(name)
            for dspec in self._resolved.get("dpad") or []:
                axis = str(dspec.get("axis", ""))
                raw = self._layout.axis_raw(joy, axis)
                self._dpad_active[axis] = 1 if raw > 0.5 else (-1 if raw < -0.5 else 0)
        if not from_select:
            self._select_latch = None
        self._exec.tick_begin()
        if not self._run_hooks(target, "enter"):
            self.get_logger().error(f"进入 {target} 失败 → 回退")
            self._run_hooks(target, "exit")
            fallback = "normal" if "normal" in self._mode_names else (prev or target)
            self._mode = fallback
            self._resolved = resolve_mode(self._mapping, fallback)
            self._axis_bindings = iter_axis_bindings(self._resolved)
            self._button_bindings = iter_button_bindings(self._resolved)
            self._publish_mode_state(fallback, rejected=target)
            return
        self._publish_mode_state(target)

    def _publish_mode_state(self, mode: str, rejected: Optional[str] = None) -> None:
        spec = self._mode_names.get(mode) or {}
        label = str(spec.get("label", mode.upper()))
        rumble = int(spec.get("rumble_index", 0))
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)
        event = f"mode_rejected:{rejected}" if rejected else \
            f"mode_changed:{label}:{rumble}"
        self._exec.notify_operator_event(event)
        self.get_logger().warn(f"模式 → {label}{('（进入失败：' + rejected + '）') if rejected else ''}")

    # ---------------------------------------------------------------- 主循环
    def _on_timer(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._joy is None or (now - self._joy_received_at) > 1.0:
            self._joy_alive_ticks = 0
            return

        self._joy_alive_ticks += 1
        if self._joy_ready():
            self._maybe_start_servo()
            self._exec.poll(now)
        else:
            return
        if self._mode and now - self._mode_t > 1.0:
            msg = String()
            msg.data = self._mode
            self._mode_pub.publish(msg)
            self._mode_t = now

        joy = self._joy
        # 切模式瞬间仍按住的键：松开之前不参与新模式的绑定
        if self._held_latch:
            self._held_latch = {
                b for b in self._held_latch if self._layout.button(joy, b)}
        self._exec.tick_begin()
        pose_block = self._exec.pose_blocking(now)

        # 模式选择/退出（先处理，选模式窗口内屏蔽其它输入）
        select_window = False
        if self._mode_names:
            self._handle_mode_exit(joy, now)
            select_window = self._handle_mode_select(joy, now)
        if self._pending_mode is not None:
            self._check_pending_mode(now)
            select_window = True

        block_inputs = pose_block or select_window or now < self._mode_guard_until
        resolved = self._resolved

        # F64：逐轴 gates——轴任一 gate 按住才取样；未列 gates 的轴不门控（R2 夹爪）
        # gate 键每 tick 只求值一次（同一 gate 被多轴复用时不再重复查表）
        gate_cache: Dict[str, bool] = {}

        def gate_on(name: str) -> bool:
            val = gate_cache.get(name)
            if val is None:
                val = self._layout.button(joy, name)
                gate_cache[name] = val
            return val

        if not block_inputs:
            for axis_name, spec in self._axis_bindings:
                gates = spec.get("gates") or []
                if gates and not any(gate_on(g) for g in gates):
                    continue
                fn = str(spec.get("fn"))
                kind = str(spec.get("kind", "analog_n11"))
                if kind == "analog_01":
                    default_source = (
                        "trigger" if axis_name in ("l2", "r2") else "unit"
                    )
                    source = str(spec.get("source", default_source))
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

        if not block_inputs:
            for bind_key, btn, spec in self._button_bindings:
                if btn in self._held_latch:
                    continue
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

            for dspec in resolved.get("dpad") or []:
                axis_name = str(dspec.get("axis", ""))
                raw = self._layout.axis_raw(joy, axis_name)
                sign = 1 if raw > 0.5 else (-1 if raw < -0.5 else 0)
                if sign != 0 and self._dpad_active.get(axis_name, 0) == 0:
                    entry = dspec.get("pos" if sign > 0 else "neg") or {}
                    kwargs = dict(entry.get("kwargs") or {})
                    self._exec.apply_discrete(str(entry.get("fn")), kwargs)
                self._dpad_active[axis_name] = sign

        self._exec.tick_end(now, not block_inputs, self._dt)


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
