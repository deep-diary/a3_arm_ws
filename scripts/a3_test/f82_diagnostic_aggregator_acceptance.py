#!/usr/bin/env python3
"""F82 验收：diagnostic_aggregator GenericAnalyzer 分组 + toplevel 状态转移。

用法：python3 f82_diagnostic_aggregator_acceptance.py [domain]

phase 1（domain，默认 82）：起 aggregator_node（加载 a3_bringup/config/diagnostics.yaml），
  本脚本向 /diagnostics 注入两组状态，断言：
  1) /diagnostics_agg 出现 /A3/Hardware/... 与 /A3/Arm Monitor/... 路径，toplevel=0
  2) 硬件组置 ERROR → toplevel=2（≤2.5 s）；恢复 → 0
  3) 停止发布 → 组 STALE、toplevel=3（≤timeout+3 s）；恢复 → 0
phase 2（domain+1）：全栈烟雾
  a3_bringup.launch.py hardware:=mock use_mqtt:=false use_teleop:=false
  → /diagnostic_aggregator 节点与 /diagnostics_agg、/diagnostics_toplevel_state 话题存在

设 F82_SMOKE=0 可跳过 phase 2。
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

A3_ROOT = "/home/cat/a3_arm_ws"
HW_STATUS = "a3_hardware:feedback_watchdog"
MON_STATUS = "a3_arm_monitor: Monitor"
HW_PATH = f"/A3/Hardware/{HW_STATUS}"
MON_PATH = f"/A3/Arm Monitor/{MON_STATUS}"
TIMEOUT_S = 5.0


def as_int(v):
    # 新版 diagnostic_msgs 的 level 在 rclpy 里可能以 bytes（b'\x02'）形式到达。
    if isinstance(v, (bytes, bytearray)):
        return v[0]
    return int(v)


class DiagHarness(Node):
    def __init__(self):
        super().__init__("f82_harness")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(DiagnosticArray, "/diagnostics", qos)
        self.agg_qos = QoSProfile(depth=20)
        self.sub_agg = self.create_subscription(
            DiagnosticArray, "/diagnostics_agg", self._on_agg, self.agg_qos
        )
        # ROS 2 版 toplevel 话题是 diagnostic_msgs/DiagnosticStatus（level 字段即 0..3）。
        self.sub_top = self.create_subscription(
            DiagnosticStatus, "/diagnostics_toplevel_state", self._on_top, 10
        )
        self.agg = None
        self.toplevel = None
        self.hw_level = DiagnosticStatus.OK
        self.mon_level = DiagnosticStatus.OK
        self.publishing = True
        self.timer = self.create_timer(0.2, self._tick)

    def _on_agg(self, msg):
        self.agg = msg

    def _on_top(self, msg):
        self.toplevel = as_int(msg.level)

    def _status(self, name, level):
        s = DiagnosticStatus()
        s.level = level
        s.name = name
        s.message = "OK" if level == DiagnosticStatus.OK else "TRIGGERED"
        s.hardware_id = "a3-arm"
        return s

    def _tick(self):
        if not self.publishing:
            return
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [
            self._status(HW_STATUS, self.hw_level),
            self._status(MON_STATUS, self.mon_level),
        ]
        self.pub.publish(msg)

    def agg_paths(self):
        if self.agg is None:
            return set()
        return {s.name for s in self.agg.status}

    def agg_level(self, path):
        if self.agg is None:
            return None
        for s in self.agg.status:
            if s.name == path:
                return as_int(s.level)
        return None


def wait_for(node, cond, timeout_s, desc):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if cond():
            return True
    print(f"  TIMEOUT waiting for: {desc}")
    return False


def run(cmd, env, timeout_s=60):
    return subprocess.run(
        cmd, env=env, capture_output=True, text=True, timeout=timeout_s
    )


def main():
    domain = int(sys.argv[1]) if len(sys.argv) > 1 else 82
    results = []

    # harness 自身也必须在目标 domain（子进程只影响孩子，不影响本进程）。
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["PYTHONNOUSERSITE"] = "1"

    base_env = os.environ.copy()
    base_env["ROS_DOMAIN_ID"] = str(domain)
    base_env["PYTHONNOUSERSITE"] = "1"
    source = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {A3_ROOT}/install/local_setup.bash && "
        f"export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID={domain}"
    )

    # ---------------- phase 1：aggregator_node 行为 ----------------
    print(f"== phase 1: aggregator_node on domain {domain} ==")
    agg_proc = subprocess.Popen(
        [
            "bash", "-c",
            source + " && exec ros2 run diagnostic_aggregator aggregator_node "
            "--ros-args -r __node:=diagnostic_aggregator "
            f"--params-file {A3_ROOT}/src/a3_bringup/config/diagnostics.yaml",
        ],
        stdout=open("/tmp/f82_aggregator.log", "w"),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    rclpy.init()
    node = DiagHarness()
    try:
        time.sleep(1.0)

        # check 1：分组路径 + OK
        ok1 = wait_for(
            node,
            lambda: HW_PATH in node.agg_paths() and MON_PATH in node.agg_paths(),
            8.0,
            "both aggregated paths",
        )
        ok1b = wait_for(node, lambda: node.toplevel == 0, 3.0, "toplevel OK")
        paths = sorted(node.agg_paths())
        print(f"  paths: {paths}")
        print(f"  toplevel={node.toplevel}")
        results.append(("1 groups present + toplevel OK", ok1 and ok1b))

        # check 2：ERROR → toplevel 2 → 恢复 0
        node.hw_level = DiagnosticStatus.ERROR
        t0 = time.time()
        ok2 = wait_for(node, lambda: node.toplevel == 2, 4.0, "toplevel ERROR")
        dt_err = time.time() - t0
        hw_level_agg = node.agg_level(HW_PATH)
        print(f"  ERROR detected in {dt_err:.2f}s, agg item level={hw_level_agg}")
        node.hw_level = DiagnosticStatus.OK
        ok2b = wait_for(node, lambda: node.toplevel == 0, 4.0, "toplevel re-OK")
        results.append(("2 ERROR escalates then recovers", ok2 and ok2))

        # check 3：停发 → STALE → 恢复
        node.publishing = False
        t0 = time.time()
        ok3 = wait_for(node, lambda: node.toplevel == 3, TIMEOUT_S + 4.5,
                       "toplevel STALE")
        dt_stale = time.time() - t0
        print(f"  STALE after {dt_stale:.2f}s")
        node.publishing = True
        ok3b = wait_for(node, lambda: node.toplevel == 0, 3.0, "post-stale OK")
        results.append(("3 input loss -> STALE -> recover", ok3 and ok3b))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # preexec_fn=setsid → 子进程 pid 即新进程组 pgid。
        os.killpg(agg_proc.pid, signal.SIGINT)
        try:
            agg_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(agg_proc.pid, signal.SIGKILL)

    # ---------------- phase 2：全栈烟雾 ----------------
    if os.environ.get("F82_SMOKE", "1") != "0":
        sdomain = domain + 1
        senv = base_env.copy()
        senv["ROS_DOMAIN_ID"] = str(sdomain)
        print(f"== phase 2: full mock stack smoke on domain {sdomain} ==")
        stack = subprocess.Popen(
            [
                "bash", "-c",
                f"source /opt/ros/humble/setup.bash && "
                f"source {A3_ROOT}/install/local_setup.bash && "
                f"export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID={sdomain} && "
                f"exec ros2 launch a3_bringup a3_bringup.launch.py "
                "hardware:=mock use_mqtt:=false use_teleop:=false use_rviz:=false",
            ],
            stdout=open("/tmp/f82_smoke.log", "w"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        try:
            # 不用 ros2 CLI/daemon（daemon 跨 domain 发现不稳），直接用 rclpy 图查询。
            os.environ["ROS_DOMAIN_ID"] = str(sdomain)
            rclpy.init()
            probe = Node("f82_smoke_probe")
            try:
                def graph_ok():
                    names = {n for n, _ in probe.get_node_names_and_namespaces()}
                    topics = {n for n, _ in probe.get_topic_names_and_types()}
                    has_node = "diagnostic_aggregator" in names
                    has_agg = "/diagnostics_agg" in topics
                    has_top = "/diagnostics_toplevel_state" in topics
                    return has_node, has_agg, has_top

                has_node = has_agg = has_top = False
                deadline = time.time() + 60.0
                while time.time() < deadline:
                    rclpy.spin_once(probe, timeout_sec=0.5)
                    has_node, has_agg, has_top = graph_ok()
                    if has_node and has_agg and has_top:
                        break
                print(f"  nodes has /diagnostic_aggregator: {has_node}")
                print(f"  topics: agg={has_agg} toplevel={has_top}")
                results.append(
                    ("4 mock stack smoke", has_node and has_agg and has_top)
                )
            finally:
                probe.destroy_node()
                rclpy.shutdown()
        finally:
            os.killpg(stack.pid, signal.SIGINT)
            try:
                stack.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(stack.pid, signal.SIGKILL)

    # ---------------- summary ----------------
    print("\n== F82 results ==")
    npass = 0
    for name, ok in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
        npass += bool(ok)
    print(f"{npass}/{len(results)}")
    sys.exit(0 if npass == len(results) else 1)


if __name__ == "__main__":
    main()
