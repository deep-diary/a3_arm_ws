#!/usr/bin/env python3
"""F71 仿真验收：arm_monitor 标准诊断通道（diagnostic_updater → /diagnostics）。

全自动：脚本自行在独立 ROS_DOMAIN_ID 起 arm_monitor（无其他栈依赖），
用脚本化喂数制造健康态 → STALE_JS pending/触发 → 恢复全过程。
用法：python3 f71_monitor_diagnostics_acceptance.py
退出码 0 = 全部验收项通过。

验收项：
  1. 健康态：/diagnostics 两组件（Monitor / Tracking）level=OK；MonitorStatus OK
  2. js 停发 0~1 s（pending）：Monitor level=WARN
  3. js 停发 >1 s：MonitorStatus TRIGGERED/STALE_JS；Monitor level=ERROR fault 键正确
  4. js 恢复 >clear_hold：两组件回 OK；MonitorStatus OK
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from a3_msgs.msg import ArmStatus, MonitorStatus
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DOMAIN = "59"
JOINTS = [f"L{i}_joint" for i in range(1, 8)]
os.environ.setdefault("ROS_DOMAIN_ID", DOMAIN)


class Harness(Node):
    def __init__(self):
        super().__init__("f71_harness")
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.status_pub = self.create_publisher(ArmStatus, "/a3/arm_status", 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_group_effort_controller/joint_trajectory", 10)
        self.diag_msgs = []   # (mono, {name: (level, kv dict, message)})
        self.mon_msgs = []    # (mono, status, fault)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.create_subscription(
            MonitorStatus, "/a3/monitor/status", self._on_mon, 10)

    def _on_diag(self, msg: DiagnosticArray):
        now = time.monotonic()
        for s in msg.status:
            kv = {kv.key: kv.value for kv in s.values}
            self.diag_msgs.append((now, s.name, s.level, kv, s.message))

    def _on_mon(self, msg: MonitorStatus):
        self.mon_msgs.append((time.monotonic(), msg.status, msg.fault))

    def send_js(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = JOINTS
        m.position = [0.0] * 7
        self.js_pub.publish(m)

    def send_status(self, state="READY"):
        m = ArmStatus()
        m.state = state
        self.status_pub.publish(m)

    def send_home_traj(self):
        tr = JointTrajectory(joint_names=JOINTS)
        tr.points.append(JointTrajectoryPoint(
            positions=[0.0] * 7,
            time_from_start=rclpy.duration.Duration(seconds=0.5).to_msg()))
        self.traj_pub.publish(tr)

    def diag_since(self, t0):
        return [d for d in self.diag_msgs if d[0] >= t0]

    def mon_since(self, t0):
        return [m for m in self.mon_msgs if m[0] >= t0]


def spin_s(node, t):
    t0 = time.monotonic()
    while time.monotonic() - t0 < t:
        rclpy.spin_once(node, timeout_sec=0.05)


def lvl(x):
    # DiagnosticStatus.level 是 byte 字段，rclpy 反序列化为单元素 bytes
    return x[0] if isinstance(x, (bytes, bytearray)) else x


def levels(diags, name):
    return [lvl(d[2]) for d in diags if d[1] == name]


def latest(diags, name):
    rows = [d for d in diags if d[1] == name]
    return rows[-1] if rows else None


def main():
    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN, PYTHONNOUSERSITE="1")
    mon_proc = subprocess.Popen(
        ["ros2", "run", "a3_arm_controller", "arm_monitor",
         "--ros-args", "-p", "diagnostics_period_s:=0.25"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid, env=env)

    rclpy.init()
    node = Harness()
    results = []
    try:
        # ---- 阶段 A：健康态（5 s，跨过 startup_grace 3 s）----
        t_start = time.monotonic()
        node.send_home_traj()
        spin_s(node, 0.6)
        node.send_home_traj()  # 重发一次，跨过 monitor 数据未齐期
        while time.monotonic() - t_start < 5.0:
            node.send_js()
            node.send_status()
            spin_s(node, 0.05)
        spin_s(node, 0.6)  # 等诊断窗口外补发
        da = node.diag_since(t_start)
        ma = node.mon_since(t_start)
        mon_names = {d[1] for d in da}
        ok_names = {"a3_arm_monitor: Monitor", "a3_arm_monitor: Tracking"} <= mon_names
        results.append((ok_names, f"[两组件存在] {sorted(mon_names)}"))
        healthy_mon = levels(da, "a3_arm_monitor: Monitor")[-3:]
        healthy_trk = levels(da, "a3_arm_monitor: Tracking")[-3:]
        results.append((healthy_mon and all(l == 0 for l in healthy_mon),
                        f"[健康 Monitor level=0] 最近 {healthy_mon}"))
        last_trk_a = latest(da, "a3_arm_monitor: Tracking")
        results.append((healthy_trk and all(l == 0 for l in healthy_trk),
                        f"[健康 Tracking level=0] 最近 {healthy_trk} "
                        f"msg={last_trk_a[4] if last_trk_a else '无'}"))
        mon_ok = bool(ma) and ma[-1][1] == "OK"
        results.append((mon_ok, f"[MonitorStatus OK] 最近: {ma[-1][1] if ma else '无'}"))

        # ---- 阶段 B：停 js（status 照发），pending→TRIGGERED ----
        t0 = time.monotonic()
        while time.monotonic() - t0 < 4.0:
            node.send_status()
            spin_s(node, 0.05)
        db = node.diag_since(t0)
        mb = node.mon_since(t0)
        # pending：停发后 1 s 内应出现 WARN
        warn_seen = 1 in levels(db, "a3_arm_monitor: Monitor")
        results.append((warn_seen, "[pending WARN] Monitor 出现 level=1"))
        # 触发：MonitorStatus TRIGGERED + fault STALE_JS
        triggered = [m for m in mb if m[1] == "TRIGGERED" and m[2] == "STALE_JS"]
        results.append((bool(triggered),
                        f"[MonitorStatus TRIGGERED STALE_JS] {len(triggered)} 条"))
        # ERROR：诊断 Monitor level=2 且 fault 键值
        last = latest(db, "a3_arm_monitor: Monitor")
        err_ok = last is not None and lvl(last[2]) == 2 and last[3].get("fault") == "STALE_JS"
        results.append((err_ok,
                        f"[Monitor level=ERROR fault=STALE_JS] "
                        f"level={last[2] if last else '无'} "
                        f"fault={last[3].get('fault') if last else '无'}"))

        # ---- 阶段 C：恢复 js，>clear_hold(2 s) ----
        t0 = time.monotonic()
        while time.monotonic() - t0 < 4.0:
            node.send_js()
            node.send_status()
            spin_s(node, 0.05)
        spin_s(node, 0.6)
        dc = node.diag_since(t0)
        mc = node.mon_since(t0)
        last_mon = latest(dc, "a3_arm_monitor: Monitor")
        last_trk = latest(dc, "a3_arm_monitor: Tracking")
        rec_diag = (last_mon is not None and lvl(last_mon[2]) == 0
                    and last_trk is not None and lvl(last_trk[2]) == 0)
        results.append((rec_diag,
                        f"[诊断恢复 OK] Monitor={last_mon[2] if last_mon else '无'} "
                        f"Tracking={last_trk[2] if last_trk else '无'}"))
        rec_mon = bool(mc) and mc[-1][1] == "OK"
        results.append((rec_mon, f"[MonitorStatus 恢复 OK] {mc[-1][1] if mc else '无'}"))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(os.getpgid(mon_proc.pid), signal.SIGTERM)

    print("\n===== F71 验收结果 =====")
    all_ok = True
    for ok, msg in results:
        print(("PASS " if ok else "FAIL ") + msg)
        all_ok = all_ok and ok
    print("\n总体:", "ALL PASS" if all_ok else "HAS FAILURES")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
