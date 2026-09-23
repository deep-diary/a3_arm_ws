#!/usr/bin/env python3
"""
DS4 lightbar + rumble feedback from arm/power/monitor state (F61).

派生态五色灯：
  红双闪   FAULT / COOLING / temp_warn / 看门狗 TRIGGERED
  红闪     失电(Idle/SoftProne+gate关) / 硬急停（强震 600ms）
  橙       上电未使能（Precheck/EnableInit/SoftStand、gate 开但 IDLE/DISABLED）
  绿       READY / SERVO
  蓝呼吸   TEACH
  紫       TRAJ 非 jog（goto/playback/move_to/SAFE_PARK）
  白闪一次 init 完成（arm INIT→READY）
震动：使能/到位 READY 与 DISABLED 弱震 120ms；硬急停强震 600ms；故障双震。

无 hidraw 设备时 WARN 一次但节点存活：逻辑经 /a3/ds4/feedback（String JSON）
全程可验（无头仿真）。报告构造移植 scripts/ps4/deep_dog_ds4_hid.py：
USB 0x05/32B；BT 0x11/78B + 0xC0 + CRC32 seed 0xA2。
"""

from __future__ import annotations

import glob
import json
import math
import os
import struct
import time
import zlib
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from a3_msgs.msg import ArmStatus, MonitorStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

DS4_VID = 0x054C
DS4_PIDS = {0x05C4, 0x09CC, 0x0BA0}

# Linux input.h bus 号（sysfs HID_ID 首段）
BUS_USB = 0x03
BUS_BLUETOOTH = 0x05

# hidapi 语义的 bus 号（报告构造函数用）
_HID_BUS_USB = 1
_HID_BUS_BT = 2

DS4_OUTPUT_USB_ID = 0x05
DS4_OUTPUT_USB_SIZE = 32
DS4_OUTPUT_BT_ID = 0x11
DS4_OUTPUT_BT_SIZE = 78
DS4_OUTPUT_HWCTL_HID = 0x80
DS4_OUTPUT_HWCTL_CRC32 = 0x40
DS4_OUTPUT_CRC_SEED = 0xA2
DS4_OUTPUT_VALID_MOTOR = 0x01
DS4_OUTPUT_VALID_LED = 0x02

COLORS = {
    "off": (0, 0, 0),
    "red": (255, 0, 0),
    "orange": (255, 96, 0),
    "green": (0, 200, 0),
    "blue": (0, 90, 255),
    "purple": (170, 0, 255),
    "white": (255, 255, 255),
}

# 派生态类别（边沿震动/闪白据此判定）
CLS_FAULT = "fault"
CLS_OFFLINE = "offline"
CLS_POWERING = "powering"
CLS_TEACH = "teach"
CLS_TRAJ = "traj"
CLS_READY = "ready"
CLS_DISABLED = "disabled"
CLS_IDLE = "idle"

_POWER_INIT_STATES = {"Precheck", "EnableInit", "SoftStand"}
_POWER_DOWN_STATES = {"", "Idle", "SoftProne"}


def _clamp_u8(v: int) -> int:
    return max(0, min(255, int(v)))


def _ds4_output_crc32(report_without_crc: bytes) -> int:
    crc = zlib.crc32(bytes([DS4_OUTPUT_CRC_SEED]), 0xFFFFFFFF) & 0xFFFFFFFF
    crc = zlib.crc32(report_without_crc, crc) & 0xFFFFFFFF
    return (~crc) & 0xFFFFFFFF


def build_output_report(
    weak: float,
    strong: float,
    rgb: Tuple[int, int, int],
    hid_bus: int = _HID_BUS_USB,
) -> bytes:
    common = bytes([
        DS4_OUTPUT_VALID_MOTOR | DS4_OUTPUT_VALID_LED,
        0x00,
        0x00,
        _clamp_u8(max(0.0, min(1.0, weak)) * 255.0),
        _clamp_u8(max(0.0, min(1.0, strong)) * 255.0),
        _clamp_u8(rgb[0]),
        _clamp_u8(rgb[1]),
        _clamp_u8(rgb[2]),
        0,
        0,
    ])
    if hid_bus == _HID_BUS_BT:
        pkt = bytearray(DS4_OUTPUT_BT_SIZE)
        pkt[0] = DS4_OUTPUT_BT_ID
        pkt[1] = DS4_OUTPUT_HWCTL_HID | DS4_OUTPUT_HWCTL_CRC32
        pkt[3:13] = common
        crc = _ds4_output_crc32(bytes(pkt[:-4]))
        struct.pack_into("<I", pkt, DS4_OUTPUT_BT_SIZE - 4, crc)
        return bytes(pkt)
    pkt = bytearray(DS4_OUTPUT_USB_SIZE)
    pkt[0] = DS4_OUTPUT_USB_ID
    pkt[1:11] = common
    return bytes(pkt)


def _hid_uevent_ids(hidraw: str) -> Optional[Tuple[int, int, int]]:
    """Return (bus, vendor, product) from /sys/class/hidraw/hidrawN/device/uevent."""
    base = os.path.basename(hidraw)
    uevent = f"/sys/class/hidraw/{base}/device/uevent"
    try:
        with open(uevent, "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("HID_ID="):
                    parts = line.strip().split("=")[1].split(":")
                    if len(parts) >= 3:
                        return int(parts[0], 16), int(parts[1], 16), int(parts[2], 16)
    except OSError:
        return None
    return None


def _is_gamepad_interface(hidraw: str) -> bool:
    """
    DS4 over USB exposes two HID interfaces (gamepad + touchpad).

    Output reports must go to the gamepad interface; identify it via the
    sibling input device name.
    """
    base = os.path.basename(hidraw)
    pattern = f"/sys/class/hidraw/{base}/device/input/input*/name"
    names: List[str] = []
    for name_file in glob.glob(pattern):
        try:
            with open(name_file, "r", encoding="utf-8") as f:
                names.append(f.read().strip())
        except OSError:
            continue
    if not names:
        return True  # BT 单接口/无法判定时不排除
    return any("Touchpad" not in n for n in names)


def find_ds4_hidraw() -> List[Tuple[str, int]]:
    found: List[Tuple[str, int]] = []
    for path in sorted(glob.glob("/dev/hidraw*")):
        ids = _hid_uevent_ids(path)
        if ids and ids[1] == DS4_VID and ids[2] in DS4_PIDS and _is_gamepad_interface(path):
            found.append((path, ids[0]))
    return found


@dataclass
class _RumblePhase:
    start_offset: float
    duration: float
    weak: float
    strong: float


@dataclass
class _Effect:
    cls: str
    color: str
    pattern: str = "solid"  # solid / blink / double / breathe
    reason: str = ""
    rumble_phases: List[_RumblePhase] = field(default_factory=list)


class Ds4FeedbackNode(Node):
    def __init__(self) -> None:
        super().__init__("ds4_feedback_node")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("scan_period_s", 2.0)
        self.declare_parameter("hidraw_path", "")
        self.declare_parameter("arm_status_topic", "/a3/arm_status")
        self.declare_parameter("monitor_status_topic", "/a3/monitor/status")
        self.declare_parameter("feedback_topic", "/a3/ds4/feedback")
        self.declare_parameter("rumble_enable_s", 0.12)
        self.declare_parameter("rumble_estop_s", 0.6)
        self.declare_parameter("white_flash_s", 0.6)

        latch_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ArmStatus, str(self.get_parameter("arm_status_topic").value),
            self._on_arm_status, 10,
        )
        self.create_subscription(
            MonitorStatus, str(self.get_parameter("monitor_status_topic").value),
            self._on_monitor_status, 10,
        )
        self.create_subscription(String, "/power_sequence/state", self._on_power_state, latch_qos)
        self.create_subscription(Bool, "/power_sequence/gate_open", self._on_gate, latch_qos)
        self._fb_pub = self.create_publisher(
            String, str(self.get_parameter("feedback_topic").value), 10
        )

        self._arm_state = ""
        self._arm_msg = ""
        self._temp_warn = False
        self._monitor_status = ""
        self._monitor_action = ""
        self._power_state = ""
        self._gate_open = False

        self._prev_cls = ""
        self._prev_arm_state = ""
        self._rumble_started_at = 0.0
        self._rumble_phases: List[_RumblePhase] = []
        self._white_until = 0.0
        self._init_done_latched = False
        self._last_feedback_json = ""

        self._fd: Optional[int] = None
        self._hid_bus = _HID_BUS_USB
        self._device_path = ""
        self._warned_no_device = False
        self._last_report = b""

        explicit = str(self.get_parameter("hidraw_path").value).strip()
        if explicit:
            self._open_device(explicit)
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._on_effect_timer)
        scan = max(0.5, float(self.get_parameter("scan_period_s").value))
        self.create_timer(scan, self._on_scan_timer)
        self.get_logger().info("ds4_feedback_node ready (F61; 无设备时仅发 /a3/ds4/feedback)")

    # ---- subscriptions ----
    def _on_arm_status(self, msg: ArmStatus) -> None:
        # 消息粒度抓 INIT→READY：INIT 可能只存在两帧之间（sim 仅几 ms），
        # 15Hz 效果定时器的 prev_arm_state 采样会漏掉（LL-064）。
        if self._arm_state == "INIT" and msg.state == "READY":
            self._init_done_latched = True
        self._arm_state = msg.state
        self._arm_msg = msg.message
        self._temp_warn = bool(msg.temp_warn)

    def _on_monitor_status(self, msg: MonitorStatus) -> None:
        self._monitor_status = msg.status
        self._monitor_action = msg.action

    def _on_power_state(self, msg: String) -> None:
        self._power_state = msg.data

    def _on_gate(self, msg: Bool) -> None:
        self._gate_open = bool(msg.data)

    # ---- effect derivation ----
    def _derive(self, now: float) -> _Effect:
        fault = (
            self._arm_state in ("FAULT", "COOLING")
            or self._temp_warn
            or self._monitor_status == "TRIGGERED"
        )
        if fault:
            reason = self._arm_msg or self._monitor_action or "fault"
            return _Effect(
                CLS_FAULT, "red", "double", reason,
                [_RumblePhase(0.0, 0.2, 0.0, 0.9), _RumblePhase(0.35, 0.2, 0.0, 0.9)],
            )
        # 只按实际带电状态判红：shutdown_recent 不能在已恢复 Running 时继续强制红，
        # 否则快速 L3 恢复会有 ~2s 红灯滞后（强震相位不受影响，到时自然结束）。
        if self._power_state in _POWER_DOWN_STATES and not self._gate_open:
            return _Effect(
                CLS_OFFLINE, "red", "blink",
                f"power={self._power_state or '?'} gate={int(self._gate_open)}",
                [_RumblePhase(0.0, float(self.get_parameter("rumble_estop_s").value), 0.0, 0.9)],
            )
        if self._arm_state == "INIT" or self._power_state in _POWER_INIT_STATES:
            return _Effect(
                CLS_POWERING, "orange", "solid",
                f"power={self._power_state} arm={self._arm_state or '?'}",
            )
        if self._arm_state == "TEACH":
            return _Effect(CLS_TEACH, "blue", "breathe", "teach")
        if self._arm_state == "SAFE_PARK":
            return _Effect(CLS_TRAJ, "purple", "solid", "safe park")
        if self._arm_state == "TRAJ" and self._arm_msg.strip() != "jog":
            return _Effect(CLS_TRAJ, "purple", "solid", self._arm_msg or "trajectory")
        if self._arm_state in ("READY", "SERVO"):
            return _Effect(CLS_READY, "green", "solid", self._arm_state.lower())
        if self._arm_state == "DISABLED":
            return _Effect(
                CLS_DISABLED, "orange", "solid",
                f"disabled gate={int(self._gate_open)}",
                [_RumblePhase(0.0, float(self.get_parameter("rumble_enable_s").value), 0.6, 0.0)],
            )
        # gate 开但 IDLE / 状态未知
        status_line = (
            f"power={self._power_state or '?'} gate={int(self._gate_open)} "
            f"arm={self._arm_state or '?'}"
        )
        return _Effect(
            CLS_IDLE, "orange", "solid",
            status_line,
        )

    def _on_class_entry(self, eff: _Effect, now: float) -> None:
        if eff.cls == self._prev_cls:
            return
        if eff.rumble_phases:
            self._rumble_phases = list(eff.rumble_phases)
            self._rumble_started_at = now
        # init 完成：INIT→READY 白闪一次（READY 类别自身的弱震仍照发）
        if eff.cls == CLS_READY and self._prev_arm_state == "INIT":
            self._white_until = now + float(self.get_parameter("white_flash_s").value)
        elif eff.cls == CLS_READY and self._prev_cls in (
            CLS_IDLE, CLS_POWERING, CLS_TRAJ, CLS_TEACH, CLS_DISABLED,
        ):
            self._rumble_phases = [
                _RumblePhase(0.0, float(self.get_parameter("rumble_enable_s").value), 0.6, 0.0)
            ]
            self._rumble_started_at = now

    @staticmethod
    def _pattern_rgb(color: str, pattern: str, now: float) -> Tuple[int, int, int]:
        base = COLORS[color]
        if pattern == "blink":
            on = (now % 1.0) < 0.5
            return base if on else (0, 0, 0)
        if pattern == "double":
            ph = now % 1.4
            on = ph < 0.12 or 0.27 <= ph < 0.39
            return base if on else (0, 0, 0)
        if pattern == "breathe":
            scale = 0.2 + 0.8 * (0.5 + 0.5 * math.sin(2.0 * math.pi * 0.5 * now))
            return tuple(int(c * scale) for c in base)  # type: ignore[return-value]
        return base

    def _current_rumble(self, now: float) -> Tuple[float, float]:
        weak = strong = 0.0
        for ph in self._rumble_phases:
            t = now - self._rumble_started_at - ph.start_offset
            if 0.0 <= t < ph.duration:
                weak = max(weak, ph.weak)
                strong = max(strong, ph.strong)
        return weak, strong

    # ---- timers ----
    def _on_effect_timer(self) -> None:
        now = time.monotonic()
        if self._init_done_latched:
            self._init_done_latched = False
            self._white_until = now + float(self.get_parameter("white_flash_s").value)
        eff = self._derive(now)
        self._on_class_entry(eff, now)
        self._prev_cls = eff.cls
        self._prev_arm_state = self._arm_state

        if now < self._white_until:
            color, pattern = "white", "solid"
        else:
            color, pattern = eff.color, eff.pattern
        rgb = self._pattern_rgb(color, pattern, now)
        weak, strong = self._current_rumble(now)

        payload = {
            "state": self._arm_state,
            "power_state": self._power_state,
            "gate": bool(self._gate_open),
            "fault": eff.cls == CLS_FAULT,
            "class": eff.cls,
            "color": color,
            "pattern": pattern,
            "rumble_weak": round(weak, 3),
            "rumble_strong": round(strong, 3),
            "reason": eff.reason,
            "device": os.path.basename(self._device_path) if self._device_path else "",
        }
        data = json.dumps(payload, ensure_ascii=False)
        if data != self._last_feedback_json:
            self._fb_pub.publish(String(data=data))
            self._last_feedback_json = data

        self._write_hid(build_output_report(weak, strong, rgb, self._hid_bus))

    def _on_scan_timer(self) -> None:
        if self._fd is not None:
            return
        explicit = str(self.get_parameter("hidraw_path").value).strip()
        candidates = [(explicit, BUS_USB)] if explicit else find_ds4_hidraw()
        if not candidates:
            if not self._warned_no_device:
                self.get_logger().warn(
                    "no DualShock 4 hidraw (VID 054C); 灯/震动降级，逻辑见 /a3/ds4/feedback"
                )
                self._warned_no_device = True
            return
        path, bus = candidates[0]
        self._open_device(path, bus)

    # ---- hidraw ----
    def _open_device(self, path: str, bus: Optional[int] = None) -> None:
        ids = _hid_uevent_ids(path)
        if ids is not None and bus is None:
            bus = ids[0]
        try:
            fd = os.open(path, os.O_RDWR)
        except OSError as exc:
            self.get_logger().warn(f"cannot open {path} O_RDWR: {exc}")
            return
        self._fd = fd
        self._device_path = path
        self._hid_bus = _HID_BUS_BT if bus == BUS_BLUETOOTH else _HID_BUS_USB
        self._last_report = b""
        if self._hid_bus == _HID_BUS_BT:
            proto = "BT 0x11+CRC"
        else:
            proto = "USB 0x05"
        self.get_logger().info(
            f"DS4 feedback 输出已打开 {path} ({proto})"
        )

    def _write_hid(self, report: bytes) -> None:
        if self._fd is None:
            return
        if report == self._last_report:
            return
        try:
            os.write(self._fd, report)
            self._last_report = report
        except OSError as exc:
            self.get_logger().warn(f"hidraw write failed ({exc}); 关闭等待 2s 重扫")
            self._close_hid()

    def _close_hid(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
        self._fd = None
        self._device_path = ""
        self._last_report = b""

    def destroy_node(self) -> bool:
        # 离场熄灯停震（best effort）
        if self._fd is not None:
            try:
                os.write(self._fd, build_output_report(0.0, 0.0, (0, 0, 0), self._hid_bus))
            except OSError:
                pass
        self._close_hid()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Ds4FeedbackNode()
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
