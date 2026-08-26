#!/usr/bin/env python3
"""Publish SERVO control_mode while Twist commands are fresh."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import String


class ServoModeBridge(Node):
    def __init__(self) -> None:
        super().__init__("a3_servo_mode_bridge")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("control_mode_topic", "/a3/control_mode")
        self.declare_parameter("timeout_s", 0.25)
        self._last = self.get_clock().now()
        self._active = False
        self._pub = self.create_publisher(
            String, self.get_parameter("control_mode_topic").value, 10
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter("twist_topic").value,
            self._on_twist,
            10,
        )
        self.create_timer(0.05, self._on_timer)

    def _on_twist(self, _msg: TwistStamped) -> None:
        self._last = self.get_clock().now()
        if not self._active:
            self._active = True
            self._publish("SERVO")

    def _on_timer(self) -> None:
        timeout = float(self.get_parameter("timeout_s").value)
        age = (self.get_clock().now() - self._last).nanoseconds * 1e-9
        if self._active and age > timeout:
            self._active = False
            self._publish("IDLE")

    def _publish(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self._pub.publish(msg)
        self.get_logger().info(f"control_mode → {mode}")


def main() -> None:
    rclpy.init()
    node = ServoModeBridge()
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
