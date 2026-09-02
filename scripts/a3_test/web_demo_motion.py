#!/usr/bin/env python3
"""网页演示：使能 ID7 后温和往复摆动一段时间，结束自动失能。

供 web 阶段在浏览器观察 pos_L7 实时曲线 / 3D 随动。运动经 safety_limits 限幅。
用法: python3 web_demo_motion.py [周期数，默认8]
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from a3_can_bridge.srv import MotorCommand

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import safety_limits as SL
from common import call_service


class WebDemo(Node):
    def __init__(self):
        super().__init__("a3_web_demo_motion")
        self.l7 = None
        self.create_subscription(
            JointState, "/joint_states", self._js,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10)
        self.gains_pub = self.create_publisher(String, "/mit_gains_cmd", 10)

    def _js(self, msg):
        if "L7_joint" in msg.name:
            i = msg.name.index("L7_joint")
            if i < len(msg.position) and math.isfinite(msg.position[i]):
                self.l7 = msg.position[i]

    def spin(self, d):
        end = time.time() + d
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def cmd(self, c):
        req = MotorCommand.Request()
        req.motor_id = SL.MOTOR_ID
        req.command = c
        return call_service(self, MotorCommand, "/a3/motor/" +
                            {1: "enable", 2: "reset", 3: "set_zero"}[c], req)

    def set_gains(self, kp, kd):
        m = String(data="scope=rear kp=%.1f kd=%.1f" % (kp, kd))
        for _ in range(5):
            self.gains_pub.publish(m)
            self.spin(0.1)

    def goto(self, target):
        traj = SL.build_safe_trajectory([target], segment_s=3.0)
        self.traj_pub.publish(traj)
        self.spin(3.6)


def main():
    cycles = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    rclpy.init()
    node = WebDemo()
    try:
        node.cmd(3)                 # set_zero
        node.spin(1.5)
        node.set_gains(SL.MIT_KP_MAX, SL.MIT_KD_MAX)
        node.cmd(1)                 # enable
        node.spin(2.0)
        print("[demo] 电机已使能，开始温和往复摆动 %d 周期（±0.15rad）..." % cycles, flush=True)
        for i in range(cycles):
            tgt = 0.15 if i % 2 == 0 else -0.15
            node.goto(tgt)
            print("[demo] 周期 %d -> %.2f rad (实测 L7=%s)" %
                  (i + 1, tgt, "%.3f" % node.l7 if node.l7 is not None else "NA"), flush=True)
        node.goto(0.0)
        print("[demo] 回零完成，失能。", flush=True)
    finally:
        try:
            req = MotorCommand.Request()
            req.motor_id = SL.MOTOR_ID
            req.command = 2
            call_service(node, MotorCommand, "/a3/motor/reset", req, timeout=3.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
