#!/usr/bin/env python3
"""PS4 teleop for A3 arm.

Power / safety (aligned with trotbot power_sequence):
  - Square long-press or L1+R1 long-press -> 'start'
  - L1+R1+Share long-press or Triangle -> 'shutdown'
  - Options long-press -> 'set_zero'

Motion jog (publishes JointTrajectory to /a3/joint_trajectory):
  - Left stick  -> L1 / L2
  - Right stick -> L3 / L4
  - L2/R2       -> L5
  - D-pad       -> L6 / L7
  - Hold R1     -> faster jog
"""

from __future__ import annotations

import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [
    "L1_joint", "L2_joint", "L3_joint", "L4_joint",
    "L5_joint", "L6_joint", "L7_joint",
]


class Ps4ArmTeleop(Node):
    def __init__(self) -> None:
        super().__init__("ps4_arm_teleop")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("command_topic", "/power_sequence/command")
        self.declare_parameter("traj_topic", "/a3/joint_trajectory")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("jog_rate_hz", 50.0)
        self.declare_parameter("base_jog_speed", 0.35)
        self.declare_parameter("fast_jog_scale", 2.0)
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("start_longpress_s", 1.0)
        self.declare_parameter("shutdown_longpress_s", 1.0)
        self.declare_parameter("set_zero_longpress_s", 2.0)
        self.declare_parameter("btn_square", 3)
        self.declare_parameter("btn_triangle", 2)
        self.declare_parameter("btn_l1", 4)
        self.declare_parameter("btn_r1", 5)
        self.declare_parameter("btn_share", 8)
        self.declare_parameter("btn_options", 9)

        self._cmd_pub = self.create_publisher(
            String, self.get_parameter("command_topic").value, 10
        )
        self._traj_pub = self.create_publisher(
            JointTrajectory, self.get_parameter("traj_topic").value, 10
        )
        self.create_subscription(Joy, self.get_parameter("joy_topic").value, self._on_joy, 10)
        self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value, self._on_js, 10
        )

        self._q = [0.0] * 7
        self._have_js = False
        self._joy: Optional[Joy] = None
        self._press_t = {"start": None, "shutdown": None, "set_zero": None}

        rate = float(self.get_parameter("jog_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._on_timer)
        self.get_logger().info("PS4 arm teleop ready")

    def _btn(self, joy: Joy, name: str) -> bool:
        idx = int(self.get_parameter(name).value)
        return 0 <= idx < len(joy.buttons) and joy.buttons[idx] == 1

    def _axis(self, joy: Joy, idx: int) -> float:
        dz = float(self.get_parameter("deadzone").value)
        if idx >= len(joy.axes):
            return 0.0
        v = float(joy.axes[idx])
        return 0.0 if abs(v) < dz else v

    def _on_js(self, msg: JointState) -> None:
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        for i, jn in enumerate(JOINTS):
            if jn in name_to_pos:
                self._q[i] = float(name_to_pos[jn])
        self._have_js = True

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg
        now = time.monotonic()

        start_held = self._btn(msg, "btn_square") or (
            self._btn(msg, "btn_l1")
            and self._btn(msg, "btn_r1")
            and not self._btn(msg, "btn_share")
        )
        shutdown_held = (
            self._btn(msg, "btn_l1")
            and self._btn(msg, "btn_r1")
            and self._btn(msg, "btn_share")
        ) or self._btn(msg, "btn_triangle")
        set_zero_held = self._btn(msg, "btn_options")

        self._update_longpress("start", start_held, now, "start_longpress_s", "start")
        self._update_longpress(
            "shutdown", shutdown_held, now, "shutdown_longpress_s", "shutdown"
        )
        self._update_longpress(
            "set_zero", set_zero_held, now, "set_zero_longpress_s", "set_zero"
        )

    def _update_longpress(
        self, key: str, held: bool, now: float, param: str, command: str
    ) -> None:
        if held:
            if self._press_t[key] is None:
                self._press_t[key] = now
            elif now - self._press_t[key] >= float(self.get_parameter(param).value):
                msg = String()
                msg.data = command
                self._cmd_pub.publish(msg)
                self.get_logger().warn(f"power_sequence command: {command}")
                self._press_t[key] = now + 1e9
        else:
            self._press_t[key] = None

    def _on_timer(self) -> None:
        if self._joy is None or not self._have_js:
            return
        joy = self._joy
        speed = float(self.get_parameter("base_jog_speed").value)
        if self._btn(joy, "btn_r1") and not self._btn(joy, "btn_l1"):
            speed *= float(self.get_parameter("fast_jog_scale").value)
        dt = 1.0 / max(float(self.get_parameter("jog_rate_hz").value), 1.0)

        dq = [0.0] * 7
        dq[0] = self._axis(joy, 0) * speed * dt
        dq[1] = -self._axis(joy, 1) * speed * dt
        dq[2] = self._axis(joy, 3) * speed * dt
        dq[3] = -self._axis(joy, 4) * speed * dt
        l2 = self._axis(joy, 2)
        r2 = self._axis(joy, 5)
        # Triggers often rest at +1; treat deflection toward -1 as pressed
        l2_p = max(0.0, (1.0 - l2) * 0.5) if abs(l2) > 0 else 0.0
        r2_p = max(0.0, (1.0 - r2) * 0.5) if abs(r2) > 0 else 0.0
        dq[4] = (r2_p - l2_p) * speed * dt
        dq[5] = self._axis(joy, 6) * speed * dt
        dq[6] = -self._axis(joy, 7) * speed * dt

        if all(abs(v) < 1e-9 for v in dq):
            return

        target: List[float] = [self._q[i] + dq[i] for i in range(7)]
        traj = JointTrajectory()
        traj.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = target
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(dt * 1e9)
        traj.points = [pt]
        self._traj_pub.publish(traj)


def main() -> None:
    rclpy.init()
    node = Ps4ArmTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
