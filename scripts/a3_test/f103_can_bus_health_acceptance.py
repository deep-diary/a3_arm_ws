#!/usr/bin/env python3
"""F103 acceptance: CAN bus physical-layer health on standard diagnostics.

A vcan interface (vcan103) stands in for the real SocketCAN link. The product
can_bus_monitor samples `ip -s -d link` and reports task
"a3_can_bus: CAN link <iface>"; the product aggregator (shipped
diagnostics.yaml) places it under /A3/Hardware.

Acceptance (docs/edge/REQUIREMENTS.md F103):
  1. vcan103 UP: diagnostic OK + aggregator item OK within 15 s.
  2. vcan103 set down: diagnostic ERROR + aggregator ERROR within 10 s.
  3. vcan103 set up: both recover OK within 10 s.
  4. interface:=can99 (missing): diagnostic ERROR within 10 s, node stays alive.

Simulation only — the real arm stays powered off. Exit 0 = all passed.
Requires: sudo for `ip link add/set`; vcan kernel module.
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "103"
IFACE = "vcan103"
MISSING_IFACE = "can99"
SUDO_PW = "temppwd"
MON_LOG = "/tmp/f103_monitor.log"
MON99_LOG = "/tmp/f103_monitor99.log"
AGG_LOG = "/tmp/f103_aggregator.log"
DIAG_YAML = f"{WS}/src/a3_bringup/config/diagnostics.yaml"
TASK_NAME = f"a3_can_bus: CAN link {IFACE}"
AGG_NAME = f"/A3/Hardware/a3_can_bus: CAN link {IFACE}"
TASK99_NAME = f"a3_can_bus: CAN link {MISSING_IFACE}"

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def _level(x):
    # LL-107：本机 diagnostic_msgs 4.9.1 把 octet（字段与常量）投递为 bytes
    return x[0] if isinstance(x, (bytes, bytearray)) else x


LEVEL_OK = _level(DiagnosticStatus.OK)
LEVEL_ERROR = _level(DiagnosticStatus.ERROR)


def make_env():
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = DOMAIN
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    env["PYTHONNOUSERSITE"] = "1"
    return env


def spin(node, t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def sudo_ip(*args):
    proc = subprocess.run(
        ["sudo", "-S", "ip", *args],
        input=f"{SUDO_PW}\n", text=True, capture_output=True)
    return proc.returncode, proc.stderr


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f103_acceptance")
        self.levels = {}
        self.agg_levels = {}
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.create_subscription(
            DiagnosticArray, "/diagnostics_agg", self._on_agg, 10)

    def _on_diag(self, msg):
        for s in msg.status:
            self.levels[s.name] = _level(s.level)

    def _on_agg(self, msg):
        for s in msg.status:
            self.agg_levels[s.name] = _level(s.level)

    def wait_until(self, pred, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            spin(self, 0.1)
            if pred():
                return time.monotonic() - t0
        return None


def ensure_vcan_up():
    exists = subprocess.run(
        ["ip", "link", "show", IFACE], capture_output=True).returncode == 0
    if not exists:
        rc, err = sudo_ip("link", "add", "dev", IFACE, "type", "vcan")
        if rc != 0:
            raise RuntimeError(f"cannot add {IFACE}: {err.strip()}")
    rc, err = sudo_ip("link", "set", IFACE, "up")
    if rc != 0:
        raise RuntimeError(f"cannot set {IFACE} up: {err.strip()}")


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["PYTHONNOUSERSITE"] = "1"

    ensure_vcan_up()

    procs = []

    def kill_all():
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    monitor = subprocess.Popen(
        ["ros2", "run", "a3_bringup", "can_bus_monitor",
         "--ros-args", "-p", f"interface:={IFACE}"],
        stdout=open(MON_LOG, "wb"), stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL, start_new_session=True, env=make_env())
    procs.append(monitor)

    aggregator = subprocess.Popen(
        ["ros2", "run", "diagnostic_aggregator", "aggregator_node",
         "--ros-args", "-r", "__node:=diagnostic_aggregator",
         "--params-file", DIAG_YAML],
        stdout=open(AGG_LOG, "wb"), stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL, start_new_session=True, env=make_env())
    procs.append(aggregator)

    rclpy.init()
    node = HarnessNode()
    try:
        t = node.wait_until(
            lambda: node.levels.get(TASK_NAME) == LEVEL_OK
            and node.agg_levels.get(AGG_NAME) == LEVEL_OK, 15)
        check("1 link UP: diagnostic + aggregator OK within 15 s",
              t is not None,
              f"after {t:.1f}s" if t is not None
              else f"diag={node.levels.get(TASK_NAME)} "
                   f"agg={node.agg_levels.get(AGG_NAME)}")

        sudo_ip("link", "set", IFACE, "down")
        t = node.wait_until(
            lambda: node.levels.get(TASK_NAME) == LEVEL_ERROR
            and node.agg_levels.get(AGG_NAME) == LEVEL_ERROR, 10)
        check("2 link down: diagnostic + aggregator ERROR within 10 s",
              t is not None,
              f"after {t:.1f}s" if t is not None
              else f"diag={node.levels.get(TASK_NAME)} "
                   f"agg={node.agg_levels.get(AGG_NAME)}")

        sudo_ip("link", "set", IFACE, "up")
        t = node.wait_until(
            lambda: node.levels.get(TASK_NAME) == LEVEL_OK
            and node.agg_levels.get(AGG_NAME) == LEVEL_OK, 10)
        check("3 link up: diagnostic + aggregator recover OK within 10 s",
              t is not None,
              f"after {t:.1f}s" if t is not None
              else f"diag={node.levels.get(TASK_NAME)} "
                   f"agg={node.agg_levels.get(AGG_NAME)}")

        # 任务名前缀取自节点名（须为 a3_can_bus:），先停掉 vcan103 实例避免争名
        try:
            os.killpg(os.getpgid(monitor.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass

        # can99 不存在；节点名保持 a3_can_bus，诊断任务名符合契约
        monitor99 = subprocess.Popen(
            ["ros2", "run", "a3_bringup", "can_bus_monitor",
             "--ros-args", "-p", f"interface:={MISSING_IFACE}"],
            stdout=open(MON99_LOG, "wb"), stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True, env=make_env())
        procs.append(monitor99)

        t = node.wait_until(
            lambda: node.levels.get(TASK99_NAME) == LEVEL_ERROR, 10)
        time.sleep(1.0)
        alive = monitor99.poll() is None
        check("4 missing interface: ERROR within 10 s, node stays alive",
              t is not None and alive,
              f"ERROR after {t:.1f}s, alive={alive}" if t is not None
              else f"level={node.levels.get(TASK99_NAME)}, alive={alive}")
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        kill_all()
        sudo_ip("link", "set", IFACE, "up")
        node.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F103 acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
