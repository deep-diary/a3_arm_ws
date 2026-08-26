#!/usr/bin/env python3
"""Optional DualShock 4 hidraw reader: IMU, touchpad, battery.

Parses USB report 0x01 and BT report 0x11 (psdevwiki DualShock 4).
If hidraw is missing or unreadable, logs once and stays idle.
"""

from __future__ import annotations

import glob
import math
import os
import struct
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

DS4_VID = 0x054C
DS4_PIDS = {0x05C4, 0x09CC, 0x0BA0}

# Approximate scales used by common DS4 userspace parsers.
_GYRO_DIV = 16.0  # LSB / deg/s
_ACCEL_DIV = 8192.0  # LSB / g


@dataclass
class Ds4Sample:
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    touch_x: float = 0.0
    touch_y: float = 0.0
    fingers: float = 0.0
    battery: float = 0.0
    ok: bool = False


def _hid_ids(hidraw: str) -> Optional[tuple]:
    base = os.path.basename(hidraw)
    uevent = f"/sys/class/hidraw/{base}/device/uevent"
    try:
        with open(uevent, "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("HID_ID="):
                    parts = line.strip().split("=")[1].split(":")
                    if len(parts) >= 3:
                        return int(parts[1], 16), int(parts[2], 16)
    except OSError:
        return None
    return None


def find_ds4_hidraw() -> List[str]:
    found: List[str] = []
    for path in sorted(glob.glob("/dev/hidraw*")):
        ids = _hid_ids(path)
        if ids and ids[0] == DS4_VID and ids[1] in DS4_PIDS:
            found.append(path)
    return found


def _i16(buf: bytes, off: int) -> int:
    if off + 2 > len(buf):
        return 0
    return struct.unpack_from("<h", buf, off)[0]


def parse_ds4_report(buf: bytes) -> Optional[Ds4Sample]:
    if not buf:
        return None
    rid = buf[0]
    if rid == 0x01:
        g0 = 13
        t0 = 35
        bat = 30
    elif rid == 0x11:
        g0 = 15
        t0 = 37
        bat = 32
    else:
        return None
    if len(buf) < g0 + 12:
        return None
    sample = Ds4Sample(ok=True)
    gx = _i16(buf, g0) / _GYRO_DIV
    gy = _i16(buf, g0 + 2) / _GYRO_DIV
    gz = _i16(buf, g0 + 4) / _GYRO_DIV
    ax = _i16(buf, g0 + 6) / _ACCEL_DIV
    ay = _i16(buf, g0 + 8) / _ACCEL_DIV
    az = _i16(buf, g0 + 10) / _ACCEL_DIV
    deg2rad = math.pi / 180.0
    sample.gx = gx * deg2rad
    sample.gy = gy * deg2rad
    sample.gz = gz * deg2rad
    sample.ax = ax * 9.81
    sample.ay = ay * 9.81
    sample.az = az * 9.81
    if bat < len(buf):
        nibble = buf[bat] & 0x0F
        sample.battery = max(0.0, min(1.0, nibble / 10.0 if nibble <= 10 else 1.0))
    if t0 + 4 < len(buf):
        f1 = buf[t0 + 1]
        active = (f1 & 0x80) == 0
        if active:
            x = ((buf[t0 + 3] & 0x0F) << 8) | buf[t0 + 2]
            y = (buf[t0 + 4] << 4) | ((buf[t0 + 3] & 0xF0) >> 4)
            sample.touch_x = max(-1.0, min(1.0, (x / 1920.0) * 2.0 - 1.0))
            sample.touch_y = max(-1.0, min(1.0, (y / 943.0) * 2.0 - 1.0))
            sample.fingers = 1.0
    return sample


class Ds4HidNode(Node):
    def __init__(self) -> None:
        super().__init__("ds4_hid_node")
        self.declare_parameter("hidraw_path", "")
        self.declare_parameter("rate_hz", 50.0)
        self._lock = threading.Lock()
        self._sample = Ds4Sample()
        self._imu_pub = self.create_publisher(Imu, "/a3/ds4/imu", 10)
        self._bat_pub = self.create_publisher(Float32, "/a3/ds4/battery", 10)
        self._touch_pub = self.create_publisher(Vector3, "/a3/ds4/touch", 10)

        path = str(self.get_parameter("hidraw_path").value).strip()
        if not path:
            found = find_ds4_hidraw()
            path = found[0] if found else ""
            if found:
                self.get_logger().info(f"DS4 hidraw: {found}")
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        if not path:
            self.get_logger().warn("no DualShock 4 hidraw (VID 054C); IMU disabled")
        else:
            self._thread = threading.Thread(
                target=self._read_loop, args=(path,), daemon=True
            )
            self._thread.start()

        hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(hz, 1.0), self._on_timer)

    def _read_loop(self, path: str) -> None:
        try:
            fd = os.open(path, os.O_RDONLY)
        except OSError as exc:
            self.get_logger().warn(f"cannot open {path}: {exc}")
            return
        self.get_logger().info(f"reading {path}")
        try:
            while not self._stop.is_set():
                try:
                    buf = os.read(fd, 128)
                except OSError:
                    time.sleep(0.05)
                    continue
                parsed = parse_ds4_report(buf)
                if parsed and parsed.ok:
                    with self._lock:
                        self._sample = parsed
        finally:
            os.close(fd)

    def _on_timer(self) -> None:
        with self._lock:
            s = self._sample
        if not s.ok:
            return
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "ds4"
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.x = s.gx
        imu.angular_velocity.y = s.gy
        imu.angular_velocity.z = s.gz
        imu.linear_acceleration.x = s.ax
        imu.linear_acceleration.y = s.ay
        imu.linear_acceleration.z = s.az
        self._imu_pub.publish(imu)
        bat = Float32()
        bat.data = s.battery
        self._bat_pub.publish(bat)
        touch = Vector3()
        touch.x = s.touch_x
        touch.y = s.touch_y
        touch.z = s.fingers
        self._touch_pub.publish(touch)

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Ds4HidNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
