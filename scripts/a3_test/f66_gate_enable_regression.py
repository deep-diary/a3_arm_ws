#!/usr/bin/env python3
"""F66/LL-070 事故路径回归（2026-09-22 真机再断 L6/L7 的精确复刻）。

与 incident_regression_test.py 的区别：F51 系列覆盖的是「ROS 服务使能」路径；
本次事故中电源序列 EnableInit 用裸 CAN 0x03 直接使能电机，完全绕过
/a3/motor/enable 服务——F51 重锚/软起步全部不执行，桥续发陈旧目标甩臂。

场景（motor_protocol_node 开 enable_power_sequence_gate，gate 由本脚本扮演
power_sequence_node 发布；mock 电机直接翻 mode=2 模拟「带外裸使能」）：
  S1 gate 开 + 服务使能 + 轨迹跑到旧位姿 Q_STALE（建立陈旧目标）；
  S2 X 硬急停：gate 关沿 + 带外裸失能（mode=0），随后人工把臂搬回 Q_HOME；
  S3 L3：电源序列带外裸使能（mode=2，不调任何 ROS 服务）→ gate 重开；
  A1 gate 关后执行层日志必须出现 F66 意图作废；
  A2 使能模式上升沿必须重锚——重开后所有 MIT 帧目标 ≈ Q_HOME，
     kp 从 0 软起步，mock 关节绝不被拉向 Q_STALE。

接线/隔离同 incident_regression_test.py（/can_tx_frames↔/can_rx_frames，
独立 ROS_DOMAIN_ID=57，不碰 SocketCAN）。
"""

import os
import sys
import threading
import time

import rclpy
from a3_can_bridge.srv import MotorCommand
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from incident_regression_test import (
    Harness, JOINTS7, MOTORS, Q_HOME, call, kill_exec_node, log,
    spawn_exec_node, wait_for_publisher)

Q_STALE = 1.09   # 事故日志中失能前 L2 陈旧目标 ≈1.0839 rad
TOL = 0.10
EXEC_LOG = "/tmp/f66_gate_regression_exec.log"


def main():
    domain = "57"
    if "--domain" in sys.argv:
        domain = sys.argv[sys.argv.index("--domain") + 1]
    os.environ["ROS_DOMAIN_ID"] = domain
    log(f"[  0.00s] → 隔离域 ROS_DOMAIN_ID={domain}")

    rclpy.init()
    node = Harness()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()

    proc = None
    fh = None
    failures = []
    try:
        log("→ 拉起执行层（enable_power_sequence_gate:=true）")
        proc, fh = spawn_exec_node(
            log_path=EXEC_LOG,
            extra_args=("-p", "enable_power_sequence_gate:=true"))
        if not wait_for_publisher(node, "/can_tx_frames", 10.0):
            log("✗ 执行层未就绪")
            return 2
        log("→ 执行层就绪")

        enable_cli = node.create_client(MotorCommand, "/a3/motor/enable")
        if not enable_cli.wait_for_service(timeout_sec=5.0):
            log("✗ /a3/motor/enable 不可用")
            return 2

        gate_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        gate_pub = node.create_publisher(Bool, "/power_sequence/gate_open", gate_qos)
        t_sub = time.monotonic()
        while gate_pub.get_subscription_count() < 1 and time.monotonic() - t_sub < 5.0:
            time.sleep(0.1)

        traj_pub = node.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10)
        t_sub = time.monotonic()
        while traj_pub.get_subscription_count() < 1 and time.monotonic() - t_sub < 5.0:
            time.sleep(0.1)

        def set_gate(open_):
            msg = Bool()
            msg.data = bool(open_)
            for _ in range(3):
                gate_pub.publish(msg)
                time.sleep(0.05)

        def publish_traj(positions):
            msg = JointTrajectory()
            msg.joint_names = list(JOINTS7)
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in positions]
            msg.points = [pt]
            for _ in range(3):
                traj_pub.publish(msg)
                time.sleep(0.05)

        for m in MOTORS:
            node.set_pose(m, Q_HOME)

        # ---- S1：gate 关的 EnableInit 阶段服务使能 → SoftStand gate 开 → 轨迹到 Q_STALE ----
        set_gate(False)
        time.sleep(0.3)
        r = call(node, enable_cli, MotorCommand.Request(motor_id=0))
        if r is None or not r.success:
            log(f"✗ S1 使能失败: {getattr(r, 'message', None)}")
            return 2
        time.sleep(0.5)
        set_gate(True)
        time.sleep(0.5)
        publish_traj([Q_STALE] * 7)
        t_wait = time.monotonic()
        while time.monotonic() - t_wait < 8.0:
            fr = node.mit_frames(2)
            if fr and abs(fr[-1][3] - Q_STALE) <= 0.05:
                break
            time.sleep(0.2)
        time.sleep(0.5)
        pre = node.mit_frames(2)
        if not pre or abs(pre[-1][3] - Q_STALE) > 0.10:
            failures.append(f"S1 前置失败：L2 未建立陈旧目标 {Q_STALE}: {pre[-1:]}")
        else:
            log(f"S1 陈旧目标已建立：L2 命令/实测 ≈{node.pos[2]:.3f}")

        # ---- S2：X 硬急停——gate 关沿 + 电源序列带外裸失能；人工搬回 home ----
        set_gate(False)
        with node._lock:
            for m in MOTORS:
                node.mode[m] = 0
                node.cmd_kp[m] = 0.0
        time.sleep(0.3)
        for m in MOTORS:
            node.set_pose(m, Q_HOME)
        time.sleep(0.5)

        # ---- S3：L3——电源序列带外裸使能（不碰 ROS 服务），0.3s 后 gate 重开 ----
        t_raw_enable = node.t()
        with node._lock:
            for m in MOTORS:
                node.mode[m] = 2
        time.sleep(0.3)   # EnableInit：桥在此窗口看到模式上升沿并完成重锚
        set_gate(True)
        time.sleep(1.6)

        # ---- A1：日志证据 ----
        try:
            with open(EXEC_LOG, "r", encoding="utf-8", errors="replace") as f:
                elog = f.read()
        except OSError:
            elog = ""
        if "F66 motion intent invalidated" not in elog:
            failures.append("A1 执行层日志缺少 F66 意图作废记录（gate 关沿未清缓存）")
        edge_cnt = elog.count("F66 enable rising edge")
        if edge_cnt < 7:
            failures.append(f"A1 使能上升沿重锚只覆盖 {edge_cnt}/7 电机（应 7）")
        else:
            log(f"A1 gate 关意图作废 + {edge_cnt}/7 电机使能沿重锚日志齐全 ✓")

        # ---- A2：重开后绝不能把关节拉向 Q_STALE ----
        fr = node.mit_frames(2, t_from=t_raw_enable)
        if not fr:
            failures.append("A2 gate 重开后 L2 无 MIT 帧（保持流断）")
        else:
            worst = max(abs(f[3] - Q_HOME) for f in fr)
            max_kp = max(f[4] for f in fr)
            first_kp = fr[0][4]
            log(f"A2 重开后 {len(fr)} 帧：目标最大偏离 home {worst:.4f} rad，"
                f"kp {first_kp:.1f}→{max_kp:.1f}")
            if worst > TOL:
                failures.append(
                    f"A2 F66 失败：重开后目标被拉向陈旧位（最大偏离 home {worst:.3f} rad"
                    f"，首帧目标 {fr[0][3]:.3f}）——会再次甩臂")
            if max_kp < 40.0:
                failures.append(f"A2 kp 未升到额定（max {max_kp:.1f}）")
            if first_kp > 0.75 * max_kp:
                failures.append(f"A2 缺少 kp 软起步（首帧 kp={first_kp:.1f}）")
        q_end = node.pos[2]
        if abs(q_end - Q_HOME) > 0.05:
            failures.append(
                f"A2 mock L2 实际被甩到 {q_end:.3f}（应停在 {Q_HOME}，陈旧位 {Q_STALE}）")
        else:
            log(f"A2 L2 实际保持 {q_end:.4f}，未被甩向 {Q_STALE} ✓")

    finally:
        log("→ 收尾")
        kill_exec_node(proc, fh)
        ex.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print()
    if failures:
        log("✗ F66 gate/裸使能事故回归失败：")
        for f in failures:
            log("  - " + f)
        return 1
    log("✓ F66 事故回归通过（gate 关意图作废 / 带外裸使能沿重锚 / 软起步 / 无甩动）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
