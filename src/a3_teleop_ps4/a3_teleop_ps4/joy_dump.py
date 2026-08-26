#!/usr/bin/env python3
"""Print all Joy axes/buttons and list /dev/input/js* devices."""

from __future__ import annotations

import glob
import os
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


def list_js_devices() -> List[str]:
    rows: List[str] = []
    for path in sorted(glob.glob("/dev/input/js*")):
        name = ""
        try:
            js = os.path.basename(path)
            npath = f"/sys/class/input/{js}/device/name"
            with open(npath, "r", encoding="utf-8") as f:
                name = f.read().strip()
        except OSError:
            pass
        rows.append(f"{path}  {name}".rstrip())
    return rows


class JoyDump(Node):
    def __init__(self) -> None:
        super().__init__("joy_dump")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("period_s", 0.5)
        self._joy: Optional[Joy] = None
        self.create_subscription(
            Joy, str(self.get_parameter("joy_topic").value), self._on_joy, 10
        )
        period = float(self.get_parameter("period_s").value)
        self.create_timer(max(period, 0.1), self._on_timer)

        devs = list_js_devices()
        if devs:
            self.get_logger().info("js devices:\n  " + "\n  ".join(devs))
        else:
            self.get_logger().warn("no /dev/input/js* found")
        self.get_logger().info(
            "Move sticks/buttons. Copy indices into a3_teleop_ps4/config/ds4_linux.yaml"
        )

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg

    def _on_timer(self) -> None:
        if self._joy is None:
            self.get_logger().info("waiting for /joy …")
            return
        axes = ", ".join(f"{i}:{v:+.2f}" for i, v in enumerate(self._joy.axes))
        btns = ", ".join(f"{i}:{int(b)}" for i, b in enumerate(self._joy.buttons))
        self.get_logger().info(f"axes [{len(self._joy.axes)}] {axes}")
        self.get_logger().info(f"buttons [{len(self._joy.buttons)}] {btns}")


def main() -> None:
    rclpy.init()
    node = JoyDump()
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
