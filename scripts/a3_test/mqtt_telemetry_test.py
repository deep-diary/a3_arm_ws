#!/usr/bin/env python3
"""阶段三：MQTT 遥测上行测试（真机 can_bridge + a3_mqtt_bridge 已运行）。

驱动 can1 ID7 小角度转动，订阅 EMQX telemetry，断言 points.pos_L7 同步变化。
"""

import json
import math
import os
import sys
import threading
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
from common import (Reporter, call_service, mqtt_connect,
                    TOPIC_TELEMETRY, MQTT_HOST)


class TelemetryTest(Node):
    def __init__(self):
        super().__init__("a3_mqtt_telemetry_test")
        self.rep = Reporter("阶段三 MQTT 遥测上行 (pos_L7)")
        self.l7_pos = None
        self.create_subscription(JointState, "/joint_states", self._js,
                                 QoSProfile(depth=20,
                                            reliability=ReliabilityPolicy.BEST_EFFORT))
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10)
        self.gains_pub = self.create_publisher(String, "/mit_gains_cmd", 10)
        self.tele = []          # 收到的 points 列表
        self.discrete_seen = set()

    def _js(self, msg):
        if "L7_joint" in msg.name:
            i = msg.name.index("L7_joint")
            if i < len(msg.position) and math.isfinite(msg.position[i]):
                self.l7_pos = msg.position[i]

    def spin(self, dur):
        end = time.time() + dur
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def motor_cmd(self, command):
        req = MotorCommand.Request()
        req.motor_id = SL.MOTOR_ID
        req.command = command
        return call_service(self, MotorCommand, "/a3/motor/" +
                            {1: "enable", 2: "reset", 3: "set_zero"}[command], req)

    def set_gains(self, kp, kd):
        m = String(data="scope=rear kp=%.1f kd=%.1f" % (kp, kd))
        for _ in range(5):
            self.gains_pub.publish(m)
            self.spin(0.1)

    def run(self):
        rep = self.rep
        # ---- MQTT 连接 ----
        try:
            cli = mqtt_connect("a3-test-telemetry")
        except Exception as e:
            rep.check("连接 EMQX %s:1883" % MQTT_HOST, False, str(e))
            return rep.summary()

        def on_msg(client, obj, msg):
            try:
                payload = json.loads(msg.payload.decode("utf-8"))
                pts = payload.get("points", {})
                self.tele.append(pts)
                for k in ("control_mode", "gate_open", "power_state", "motor_feedback"):
                    if k in pts:
                        self.discrete_seen.add(k)
            except Exception:
                pass

        cli.subscribe(TOPIC_TELEMETRY)
        cli.on_message = on_msg
        rep.check("连接 EMQX 并订阅 telemetry", True, TOPIC_TELEMETRY)

        self.spin(2.0)
        rep.check("收到 telemetry 消息", len(self.tele) > 0,
                  "收到 %d 条" % len(self.tele))

        # ---- 使能电机 ----
        self.motor_cmd(3)
        self.spin(1.0)
        self.set_gains(SL.MIT_KP_MAX, SL.MIT_KD_MAX)
        self.motor_cmd(1)
        self.spin(2.0)

        def latest_pos_l7():
            for pts in reversed(self.tele):
                if "pos_L7" in pts and pts["pos_L7"] is not None:
                    try:
                        return float(pts["pos_L7"])
                    except (TypeError, ValueError):
                        continue
            return None

        base = latest_pos_l7()
        rep.check("telemetry 含 pos_L7", base is not None,
                  "pos_L7=%s" % ("%.4f" % base if base is not None else "缺失"))

        # ---- 驱动到 +0.25 rad ----
        traj = SL.build_safe_trajectory([0.25], segment_s=SL.MIN_SEG_DURATION_S)
        self.traj_pub.publish(traj)
        rep.info("驱动 L7 -> +0.25rad，采样 telemetry ...")
        self.spin(SL.MIN_SEG_DURATION_S + 2.0)
        peak = latest_pos_l7()

        # 本地 ROS 对照
        local = self.l7_pos
        delta_mqtt = abs((peak or 0.0) - (base or 0.0))
        rep.check("pos_L7 随转角变化(Δ>0.10rad)",
                  peak is not None and base is not None and delta_mqtt > 0.10,
                  "base=%.3f peak=%.3f Δ=%.3f | 本地ROS=%.3f" %
                  (base or 0.0, peak or 0.0, delta_mqtt, local if local is not None else -9))

        # ---- 离散点 / 其他关节占位 ----
        rep.check("telemetry 含离散信号(control_mode 等)",
                  len(self.discrete_seen) > 0,
                  "出现: %s" % sorted(self.discrete_seen))
        # 取最后一条来自 /joint_states 的 telemetry（含 pos_L7），
        # 而非标量 motor_feedback（只有 1 个键）。
        js_pts = {}
        for pts in reversed(self.tele):
            if "pos_L7" in pts:
                js_pts = pts
                break
        all_pos = all(("pos_L%d" % i) in js_pts for i in range(1, 8))
        l1to6 = [js_pts.get("pos_L%d" % i) for i in range(1, 7)]
        rep.check("telemetry /joint_states 含 pos_L1..L7 全部键",
                  all_pos,
                  "pos_L1..L6=%s pos_L7=%.3f (键数=%d)" %
                  (["%.2f" % v if isinstance(v, (int, float)) else "NA" for v in l1to6],
                   js_pts.get("pos_L7", -9), len(js_pts)))

        # ---- 回零并失能 ----
        back = SL.build_safe_trajectory([0.0], segment_s=SL.MIN_SEG_DURATION_S)
        self.traj_pub.publish(back)
        self.spin(SL.MIN_SEG_DURATION_S + 1.5)
        self.motor_cmd(2)
        self.spin(1.0)
        cli.loop_stop()
        cli.disconnect()
        return rep.summary()


def main():
    rclpy.init()
    node = TelemetryTest()
    try:
        ok = node.run()
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
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
