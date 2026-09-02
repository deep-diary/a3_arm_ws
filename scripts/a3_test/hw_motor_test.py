#!/usr/bin/env python3
"""阶段一：真机单电机（can1 / CAN_ID=7 / L7_joint）底层功能测试。

前置：can_bridge.launch.py use_power_sequence:=false 已在运行。
所有运动经 safety_limits 限幅；结束自动失能。
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
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from a3_can_bridge.srv import MotorCommand, MotorScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import safety_limits as SL
from common import Reporter, call_service, wait_for_topic


class HwMotorTest(Node):
    def __init__(self):
        super().__init__("a3_hw_motor_test")
        self.rep = Reporter("阶段一 真机单电机 (can1 ID7)")
        self.l7_pos = None
        self.l7_vel = None
        self.l7_eff = None
        self.mode = "IDLE"
        self.device_ids = []
        self.fb_lines = []

        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(JointState, "/joint_states", self._js_cb, qos)
        self.create_subscription(String, "/a3/control_mode", self._mode_cb, 10)
        self.create_subscription(String, "/a3/motor/device_id", self._dev_cb, 10)
        self.create_subscription(String, "/motor_feedback", self._fb_cb, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10)
        self.gains_pub = self.create_publisher(String, "/mit_gains_cmd", 10)

    # ---------- callbacks ----------
    def _js_cb(self, msg):
        if "L7_joint" in msg.name:
            i = msg.name.index("L7_joint")
            if i < len(msg.position) and math.isfinite(msg.position[i]):
                self.l7_pos = msg.position[i]
            if i < len(msg.velocity) and math.isfinite(msg.velocity[i]):
                self.l7_vel = msg.velocity[i]
            if i < len(msg.effort) and math.isfinite(msg.effort[i]):
                self.l7_eff = msg.effort[i]

    def _mode_cb(self, msg):
        self.mode = msg.data

    def _dev_cb(self, msg):
        self.device_ids.append(msg.data)

    def _fb_cb(self, msg):
        self.fb_lines.append(msg.data)
        if len(self.fb_lines) > 200:
            self.fb_lines = self.fb_lines[-200:]

    # ---------- helpers ----------
    def spin(self, dur):
        end = time.time() + dur
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def read_pos(self):
        return self.l7_pos

    def motor_cmd(self, command):
        req = MotorCommand.Request()
        req.motor_id = SL.MOTOR_ID
        req.command = command
        return call_service(self, MotorCommand, "/a3/motor/" +
                            {0: "get_device_id", 1: "enable", 2: "reset",
                             3: "set_zero", 4: "request_version"}[command], req)

    def trigger(self, name):
        return call_service(self, Trigger, name, Trigger.Request())

    def set_gains(self, kp, kd):
        msg = String()
        msg.data = "scope=rear kp=%.1f kd=%.1f" % (kp, kd)
        for _ in range(5):
            self.gains_pub.publish(msg)
            self.spin(0.1)

    def reset_gains(self):
        msg = String()
        msg.data = "reset=1"
        for _ in range(5):
            self.gains_pub.publish(msg)
            self.spin(0.1)

    # ---------- test steps ----------
    def run(self):
        rep = self.rep
        rep.info("目标: can1 电机 CAN_ID=%d (%s)，空载" % (SL.MOTOR_ID, SL.TARGET_JOINT))

        # 0. 反馈链路
        ok_topic = wait_for_topic(self, "/joint_states", JointState, timeout=8.0)
        self.spin(2.0)
        rep.check("反馈话题 /joint_states 存在", ok_topic)
        rep.check("L7_joint 反馈有效(非NaN)", self.l7_pos is not None,
                  "pos=%s" % ("%.4f" % self.l7_pos if self.l7_pos is not None else "None"))

        # 1. 扫描
        self.device_ids.clear()
        sreq = MotorScan.Request()
        sreq.id_min = 1
        sreq.id_max = 127
        sreq.bus = 1
        sres = call_service(self, MotorScan, "/a3/motor/scan", sreq)
        self.spin(2.5)
        found7 = any("motor=7" in s or "motor:7" in s for s in self.device_ids)
        rep.check("总线扫描 /a3/motor/scan 应答", sres is not None and sres.success,
                  sres.message if sres else "无应答")
        rep.check("扫描到 motor=7", found7,
                  "; ".join(self.device_ids[-3:]) if self.device_ids else "无 device_id 应答")

        # 2. 设零
        zr = self.motor_cmd(3)
        self.spin(1.5)
        p_after_zero = self.read_pos()
        rep.check("set_zero 服务成功", zr is not None and zr.success,
                  zr.message if zr else "无应答")
        rep.check("设零后 L7 角度≈0", p_after_zero is not None and abs(p_after_zero) < 0.08,
                  "pos=%.4f" % p_after_zero if p_after_zero is not None else "无反馈")

        # 3. 降低刚度（空载防过热/抖动）后使能
        self.set_gains(SL.MIT_KP_MAX, SL.MIT_KD_MAX)
        er = self.motor_cmd(1)
        self.spin(2.0)
        fb_tail = " | ".join(self.fb_lines[-2:])
        mode2 = any("mode=2" in s for s in self.fb_lines[-10:])
        rep.check("enable 服务成功", er is not None and er.success,
                  er.message if er else "无应答")
        rep.check("使能后 mode_status=2", mode2, fb_tail[-120:])

        # 4. 小角度多点轨迹（0 -> +0.25 -> 0 -> -0.25 -> 0）
        targets = [0.25, 0.0, -0.25, 0.0]
        traj = SL.build_safe_trajectory(targets, segment_s=SL.MIN_SEG_DURATION_S)
        self.traj_pub.publish(traj)
        total = SL.total_duration(targets)
        rep.info("下发温和轨迹(±0.25rad, 每段%.1fs)，跟踪中..." % SL.MIN_SEG_DURATION_S)
        samples = []
        end = time.time() + total
        while time.time() < end:
            self.spin(0.1)
            if self.l7_pos is not None:
                samples.append((time.time(), self.l7_pos))
        # 峰值角（相对零点）
        peak = max((abs(p) for _, p in samples), default=0.0)
        reached = peak > 0.12
        rep.check("轨迹运动 L7 跟随(峰值>0.12rad)", reached,
                  "peak=%.3f rad, vel=%s, eff=%s" %
                  (peak,
                   "%.3f" % self.l7_vel if self.l7_vel is not None else "NA",
                   "%.3f" % self.l7_eff if self.l7_eff is not None else "NA"))
        # 回到 0 位
        self.spin(1.5)
        p_back = self.read_pos()
        rep.check("轨迹结束回到 0 位(|pos|<0.10)", p_back is not None and abs(p_back) < 0.10,
                  "pos=%.4f" % p_back if p_back is not None else "无反馈")

        # 5. 零力矩 / 重力补偿（示教核心）
        #    客观断言：control_mode 切到 ZERO_TORQUE 且 stop 后恢复 IDLE。
        #    外力拖动需人工，自动化运行时无人转 -> 作为人工提示项，不判 FAIL。
        zt_start = self.trigger("/a3/zero_torque/start")
        self.spin(1.5)
        rep.check("zero_torque/start 服务成功",
                  zt_start is not None and zt_start.success,
                  zt_start.message if zt_start else "无应答")
        rep.check("control_mode 切到 ZERO_TORQUE", self.mode == "ZERO_TORQUE",
                  "mode=%s" % self.mode)
        rep.info("零力矩已开启：可用手轻转电机轴验证拖动手感（自动化不判此项）。")
        base = self.read_pos() or 0.0
        self.spin(3.0)
        drift = abs((self.read_pos() or 0.0) - base)
        if drift > 0.05:
            rep.info("检测到外力拖动 Δ=%.3f rad（零力矩拖动正常）" % drift)
        else:
            rep.info("未检测到外力拖动（自动化运行无人工干预，符合预期）")
        zt_stop = self.trigger("/a3/zero_torque/stop")
        self.spin(1.5)
        rep.check("zero_torque/stop 服务成功",
                  zt_stop is not None and zt_stop.success,
                  zt_stop.message if zt_stop else "无应答")
        rep.check("control_mode 恢复 IDLE", self.mode == "IDLE", "mode=%s" % self.mode)

        # 6. 超安全包络指令：发一个超出测试安全包络(0.30)但低于硬限位(1.5708)的目标，
        #    验证执行层平滑插值、无报警/冲击。硬限位裁剪(3.0->1.5708)已由首轮日志证实。
        over = SL.build_safe_trajectory([0.45], segment_s=3.5)
        over.points[1].positions[SL.TARGET_INDEX] = 0.45
        self.traj_pub.publish(over)
        self.spin(4.6)
        p_over = self.read_pos()
        no_fault = all("err=0" in s for s in self.fb_lines[-5:] if "motor=7" in s)
        rep.check("超包络指令平滑执行、无报警(|pos|≈0.45, err=0)",
                  p_over is not None and abs(p_over - 0.45) < 0.12 and no_fault,
                  "pos=%.4f" % p_over if p_over is not None else "无反馈")
        # 回到 0
        back0 = SL.build_safe_trajectory([0.0], segment_s=SL.MIN_SEG_DURATION_S)
        self.traj_pub.publish(back0)
        self.spin(SL.MIN_SEG_DURATION_S + 1.0)

        # 7. 失能
        self.reset_gains()
        rr = self.motor_cmd(2)
        self.spin(1.5)
        mode_off = not any("mode=2" in s for s in self.fb_lines[-10:])
        rep.check("reset(失能) 服务成功", rr is not None and rr.success,
                  rr.message if rr else "无应答")
        rep.check("失能后退出 mode=2", mode_off)

        return rep.summary()


def main():
    rclpy.init()
    node = HwMotorTest()
    try:
        ok = node.run()
    finally:
        # 安全兜底：尽量失能
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
