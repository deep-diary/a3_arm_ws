#!/usr/bin/env python3
"""No-CAN power sequence simulator (gate_open=true, state=Running).

Replaces the real C++ `power_sequence_node` in `edge_web_sim.launch.py`. The real
node gates motor trajectory forwarding until a start/handshake sequence completes;
in simulation there are no motors to protect, so the gate is latched open at boot
and `/power_sequence/state` reports "Running". `/power_sequence/command`
(start/prone/shutdown/set_zero) is still accepted so the frontend / tests can
exercise the power-state round trip.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String


class SimPowerSequenceNode(Node):
    def __init__(self) -> None:
        super().__init__("power_sequence_node")

        self.declare_parameter("gate_topic", "/power_sequence/gate_open")
        self.declare_parameter("state_topic", "/power_sequence/state")
        self.declare_parameter("command_topic", "/power_sequence/command")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._gate_pub = self.create_publisher(
            Bool, str(self.get_parameter("gate_topic").value), latched
        )
        self._state_pub = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), latched
        )
        self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self._on_command,
            10,
        )

        self._gate_open = True
        self._state = "Running"
        self._publish_gate()
        self._publish_state()
        # 周期性重发（1 Hz）：bridge 用 volatile 订阅，收不到 transient_local 的
        # 一次性锁存消息，须持续重发才能让 gate_open/state 进入 telemetry。
        self.create_timer(1.0, self._on_timer)
        self.get_logger().info("sim_power_sequence_node ready: gate_open=true state=Running")

    def _on_timer(self) -> None:
        self._publish_gate()
        self._publish_state()

    def _publish_gate(self) -> None:
        msg = Bool()
        msg.data = self._gate_open
        self._gate_pub.publish(msg)

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._state
        self._state_pub.publish(msg)

    def _on_command(self, msg: String) -> None:
        cmd = (msg.data or "").strip().lower()
        if cmd == "shutdown":
            self._gate_open = False
            self._state = "Idle"
        elif cmd in ("start", "prone", "set_zero"):
            self._gate_open = True
            self._state = "Running" if cmd != "prone" else "ProneHold"
        else:
            self.get_logger().warn(f"unknown power command '{cmd}'")
            return
        self._publish_gate()
        self._publish_state()
        self.get_logger().info(
            f"sim power: command={cmd} -> gate_open={self._gate_open} state={self._state}"
        )


def main() -> None:
    rclpy.init()
    node = SimPowerSequenceNode()
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
