#!/usr/bin/env python3
"""F50/F51 看门狗+编排层事故回归：示教退出不得假触发 HOLD_DRIFT（LL-039）。

2026-09-14 真机事故的第一环：示教拖动后退出，执行层 F38 把 MIT 目标重锚到拖动位姿
（正确），但看门狗的保持参照 `_last_goal` 仍停在上一条轨迹末点 → 1.2 s 后误判
HOLD_DRIFT → stop → reset → 臂失去保持 → 人工搬回 home → 使能甩断 L6。

本测试在仿真栈（sim_motor_node 冒充执行层 + 真 arm_controller/arm_monitor）上跑：

  M1 初始化+使能后看门狗 OK（无误报）；
  M2 正常轨迹（jog）不触发 FOLLOW_STUCK/HOLD_DRIFT；
  M3 示教拖动 → 退出：**不得**出现任何 TRIGGERED（修复前 1.4 s 即 HOLD_DRIFT→stop→reset）；
  M4 带外失能（直接调 /a3/motor/reset）**必须**报 UNEXPECTED_DISABLE 且编排层转
     DISABLED（闸门不能把真故障一起吞掉——防「修假阳性引入假阴性」）。

隔离：默认 ROS_DOMAIN_ID=57（真机栈在 domain 0）。
用法：
  source scripts/a3_shell_env.sh
  python3 -u scripts/a3_test/incident_monitor_regression_test.py
  python3 -u scripts/a3_test/incident_monitor_regression_test.py --keep-running  # 不自动收尾
"""

import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from a3_can_bridge.srv import MotorCommand, MotorMitCommand
from a3_msgs.msg import ArmStatus, MonitorStatus
from a3_msgs.srv import SetJointPositions
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

L2 = 1  # 事故关节：L2（索引 1）
JOG_TO = 0.8        # M2 轨迹末点
DRAG_TO = 1.70      # M3 手拖后位姿（Δ=0.9 rad ≫ hold_error_max_rad 0.30）
HOLD_DRIFT_WATCH_S = 4.0   # 修复前 1.2 s 即触发，留足余量

_T_START = time.monotonic()


def log(msg):
    print(f"[{time.monotonic() - _T_START:6.2f}s] {msg}", flush=True)


class Probe(Node):
    def __init__(self):
        super().__init__("f50_monitor_test")
        self.arm_state = ""
        self.arm_mode = ""
        self.mon_status = ""
        self.mon_fault = ""
        self.mon_action = ""
        self.mon_pending = []
        self.events = []          # (t, status, fault, action)
        self.js = {}
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_arm, 10)
        self.create_subscription(MonitorStatus, "/a3/monitor/status", self._on_mon, 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def t(self):
        return time.monotonic() - _T_START

    def _on_arm(self, msg):
        self.arm_state = msg.state
        self.arm_mode = msg.mode

    def _on_mon(self, msg):
        self.mon_status = msg.status
        self.mon_fault = msg.fault
        self.mon_action = msg.action
        self.mon_pending = list(msg.pending_faults)
        self.events.append((self.t(), msg.status, msg.fault, msg.action))

    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.js[n] = p

    def l2(self):
        return self.js.get("L2_joint", float("nan"))

    def triggers(self, t_from=0.0, fault=None):
        return [
            e for e in self.events
            if e[0] >= t_from and e[1] == "TRIGGERED" and (fault is None or e[2] == fault)
        ]


def call(client, req, timeout=20.0):
    fut = client.call_async(req)
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < timeout:
        time.sleep(0.01)
    return None if not fut.done() else fut.result()


def wait_until(cond, timeout, what):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.05)
    log(f"  ...等待超时: {what}")
    return False


def main():
    keep = "--keep-running" in sys.argv
    domain = "57"
    if "--domain" in sys.argv:
        domain = sys.argv[sys.argv.index("--domain") + 1]
    os.environ["ROS_DOMAIN_ID"] = domain
    log(f"→ 隔离域 ROS_DOMAIN_ID={domain}（真机栈在 domain 0，DDS 互不可见）")

    procs = []
    logs = []
    failures = []

    def spawn(name, cmd, logfile):
        f = open(logfile, "w")
        p = subprocess.Popen(
            cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        procs.append(p)
        logs.append(f)
        log(f"→ 拉起 {name}（日志 {logfile}）")

    rclpy.init()
    node = Probe()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    try:
        if node.get_publishers_info_by_topic("/a3/arm_status"):
            log("✗ /a3/arm_status 上已有发布者（残留仿真栈？）——先清理")
            return 2

        spawn("sim_motor_node",
              ["ros2", "run", "a3_bringup", "sim_motor_node", "--ros-args",
               "-r", "__node:=motor_protocol_node",
               "-p", "trajectory_interpolation_method:=auto"],
              "/tmp/f50_sim_motor.log")
        spawn("sim_power_sequence_node",
              ["ros2", "run", "a3_bringup", "sim_power_sequence_node", "--ros-args",
               "-r", "__node:=power_sequence_node"],
              "/tmp/f50_sim_power.log")
        spawn("arm_controller+arm_monitor",
              ["ros2", "launch", "a3_arm_controller", "arm_controller.launch.py"],
              "/tmp/f50_arm_controller.log")

        if not wait_until(
                lambda: node.arm_state and node.mon_status and node.js, 30.0,
                "仿真栈就绪（arm_status/monitor status/joint_states）"):
            log("✗ 仿真栈未就绪")
            return 2
        log(f"→ 仿真栈就绪: state={node.arm_state} monitor={node.mon_status}")

        cli = {
            "init": node.create_client(Trigger, "/a3/arm/init"),
            "enable": node.create_client(Trigger, "/a3/arm/enable"),
            "teach_start": node.create_client(Trigger, "/a3/arm/start_teach"),
            "teach_stop": node.create_client(Trigger, "/a3/arm/stop_teach"),
            "jog": node.create_client(SetJointPositions, "/a3/arm/set_joint_positions"),
            "mit": node.create_client(MotorMitCommand, "/a3/motor/mit_command"),
            "exec_reset": node.create_client(MotorCommand, "/a3/motor/reset"),
        }
        for name, c in cli.items():
            if not c.wait_for_service(timeout_sec=15.0):
                log(f"✗ 服务不可用: {name}")
                return 2
        log("→ 服务就绪")

        # ---- M1 初始化 + 使能 → READY 且看门狗不误报 ----
        r = call(cli["init"], Trigger.Request())
        if r is None or not r.success:
            failures.append(f"M1 init 失败: {getattr(r, 'message', None)}")
        if not wait_until(lambda: node.arm_state == "READY", 10.0, "init → READY"):
            failures.append(f"M1 init 后 state={node.arm_state}（期望 READY）")
        r = call(cli["enable"], Trigger.Request())
        if r is None or not r.success:
            failures.append(f"M1 enable 失败: {getattr(r, 'message', None)}")
        t_m1 = node.t()
        time.sleep(2.5)
        bad = node.triggers(t_from=t_m1)
        if bad:
            failures.append(f"M1 使能后看门狗误触发: {bad[:3]}")
        else:
            log(f"M1 初始化+使能后 {node.mon_status}，无误报 ✓（state={node.arm_state}）")

        # ---- M2 正常轨迹（jog）不触发运动类故障 ----
        t_m2 = node.t()
        q = [0.0] * 7
        q[L2] = JOG_TO
        r = call(cli["jog"], SetJointPositions.Request(positions=q, duration=0.6))
        if r is None or not r.success:
            failures.append(f"M2 jog 失败: {getattr(r, 'message', None)}")
        wait_until(lambda: node.arm_state == "READY", 8.0, "jog 后回 READY")
        time.sleep(2.0)
        bad = node.triggers(t_from=t_m2)
        if bad:
            failures.append(f"M2 正常轨迹期看门狗误触发: {bad[:3]}")
        else:
            log(f"M2 jog → L2={node.l2():.3f}（目标 {JOG_TO}），看门狗无误报 ✓")

        # ---- M3 示教拖动 → 退出：不得假触发 HOLD_DRIFT（事故根因） ----
        r = call(cli["teach_start"], Trigger.Request())
        if r is None or not r.success:
            failures.append(f"M3 进入示教失败: {getattr(r, 'message', None)}")
        wait_until(lambda: node.arm_mode == "ZERO_TORQUE", 5.0, "进入 ZERO_TORQUE")
        # 手拖：走 MIT 直驱改仿真臂实际位姿（看门狗只看到 /joint_states 变化，
        # 与真机「人手拖动」等价）
        call(cli["mit"], MotorMitCommand.Request(
            motor_id=L2 + 1, position_rad=DRAG_TO, hold_duration_s=0.0, hold_hz=50.0))
        wait_until(lambda: abs(node.l2() - DRAG_TO) < 0.1, 5.0, f"拖动到 {DRAG_TO}")
        log(f"M3 示教中拖动: L2={node.l2():.3f}（原 {JOG_TO}）")
        t_exit = node.t()
        r = call(cli["teach_stop"], Trigger.Request())
        if r is None or not r.success:
            failures.append(f"M3 退出示教失败: {getattr(r, 'message', None)}")
        wait_until(lambda: node.arm_state == "READY", 5.0, "示教退出 → READY")
        time.sleep(HOLD_DRIFT_WATCH_S)
        # 任何一类故障在这段窗口内成立都是假阳性（示教退出是合法的新位姿意图）
        bad = node.triggers(t_from=t_exit)
        if bad:
            failures.append(
                f"M3 F50 失败：示教退出后假触发 {sorted({(e[2], e[3]) for e in bad})}"
                f"（保持参照未重基准 → 真机事故同款 stop→reset 梯子）")
        else:
            log(f"M3 示教退出后 {HOLD_DRIFT_WATCH_S:.0f}s 内零触发 ✓"
                f"（L2={node.l2():.3f}，末次状态 {node.mon_status}"
                f"/{node.mon_fault or '-'}）")

        # ---- M4 带外失能必须被抓到（防假阴性） ----
        t_m4 = node.t()
        r = call(cli["exec_reset"], MotorCommand.Request(motor_id=0, command=2))
        got = wait_until(
            lambda: node.triggers(t_from=t_m4, fault="UNEXPECTED_DISABLE"), 5.0,
            "看门狗报 UNEXPECTED_DISABLE")
        if not got:
            failures.append("M4 F50 失败：带外失能未被看门狗确认（假阴性）")
        if not wait_until(lambda: node.arm_state == "DISABLED", 6.0,
                          "编排层转 DISABLED"):
            failures.append(
                f"M4 F51 失败：带外失能后编排层未转 DISABLED（state={node.arm_state}）")
        if got and node.arm_state == "DISABLED":
            ev = node.triggers(t_from=t_m4, fault="UNEXPECTED_DISABLE")[0]
            log(f"M4 带外失能 → 看门狗 TRIGGERED/{ev[2]}（action={ev[3]}）→ "
                f"编排层 DISABLED ✓")

    finally:
        if not keep:
            log("→ 收尾")
            for p in procs:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    p.terminate()
            for p in procs:
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except (ProcessLookupError, PermissionError):
                        p.kill()
            for f in logs:
                f.close()
        ex.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print()
    if failures:
        log("✗ F50/F51 看门狗事故回归测试失败：")
        for f in failures:
            log("  - " + f)
        return 1
    log("✓ F50/F51 看门狗事故回归测试通过（无误报 / 真故障不漏报）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
