#!/usr/bin/env python3
"""F104 acceptance: critical-topic frequency health on standard diagnostics.

A local publisher emits /f104_probe (std_msgs/Empty) at a controlled rate.
The product topic_rate_monitor is pointed at that topic (band 40-60 Hz); the
product aggregator exercises the new Topic Rates group.

Acceptance (docs/edge/REQUIREMENTS.md F104):
  1. 50 Hz: diagnostic + aggregator OK within 15 s.
  2. 10 Hz: both WARN within 15 s.
  3. stop publishing: both ERROR within 15 s.
  4. resume 50 Hz: both recover OK within 15 s.

Simulation only — the real arm stays powered off. Exit 0 = all passed.
"""

import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from std_msgs.msg import Empty

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "104"
PROBE = "/f104_probe"
MON_LOG = "/tmp/f104_monitor.log"
AGG_LOG = "/tmp/f104_aggregator.log"
DIAG_YAML = f"{WS}/src/a3_bringup/config/diagnostics.yaml"

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def _level(x):
    # LL-107：本机 diagnostic_msgs 4.9.1 把 octet（字段与常量）投递为 bytes
    return x[0] if isinstance(x, (bytes, bytearray)) else x


LEVEL_OK = _level(DiagnosticStatus.OK)
LEVEL_WARN = _level(DiagnosticStatus.WARN)
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


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f104_acceptance")
        self.diag_level = None
        self.agg_level = None
        self.agg_msgs = 0
        self.agg_names = set()
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.create_subscription(
            DiagnosticArray, "/diagnostics_agg", self._on_agg, 10)

    def _on_diag(self, msg):
        for s in msg.status:
            if s.name.startswith("a3_topic_rate:") and PROBE in s.name:
                self.diag_level = _level(s.level)

    def _on_agg(self, msg):
        self.agg_msgs += 1
        for s in msg.status:
            self.agg_names.add(s.name)
            # aggregator 会剥掉条目名的前导斜杠：
            # /A3/Topic Rates/a3_topic_rate:  f104_probe topic status
            if ("/Topic Rates/a3_topic_rate:" in s.name
                    and PROBE.lstrip("/") in s.name):
                self.agg_level = _level(s.level)

    def wait_until(self, pred, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            spin(self, 0.1)
            if pred():
                return time.monotonic() - t0
        return None


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["PYTHONNOUSERSITE"] = "1"

    rclpy.init()
    node = HarnessNode()
    pub = node.create_publisher(Empty, PROBE, 10)

    rate_hz = [50.0]
    stop = [False]

    def publish_loop():
        while not stop[0]:
            hz = rate_hz[0]
            if hz > 0:
                pub.publish(Empty())
                time.sleep(1.0 / hz)
            else:
                time.sleep(0.1)

    threading.Thread(target=publish_loop, daemon=True).start()
    time.sleep(0.5)

    procs = []

    def kill_all():
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    monitor = subprocess.Popen(
        ["ros2", "run", "a3_bringup", "topic_rate_monitor",
         "--ros-args",
         "-p", f"topics:=[{PROBE}]",
         "-p", "min_freq:=[40.0]",
         "-p", "max_freq:=[60.0]"],
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

    try:
        t = node.wait_until(
            lambda: node.diag_level == LEVEL_OK
            and node.agg_level == LEVEL_OK, 15)
        check("1 50 Hz: diagnostic + aggregator OK within 15 s",
              t is not None,
              f"after {t:.1f}s" if t is not None
              else f"diag={node.diag_level} agg={node.agg_level}")

        rate_hz[0] = 10.0
        t = node.wait_until(
            lambda: node.diag_level == LEVEL_WARN
            and node.agg_level == LEVEL_WARN, 15)
        check("2 10 Hz: diagnostic + aggregator WARN within 15 s",
              t is not None,
              f"after {t:.1f}s" if t is not None
              else f"diag={node.diag_level} agg={node.agg_level}")

        rate_hz[0] = 0.0
        t = node.wait_until(
            lambda: node.diag_level == LEVEL_ERROR
            and node.agg_level == LEVEL_ERROR, 15)
        check("3 stop: diagnostic + aggregator ERROR within 15 s",
              t is not None,
              f"after {t:.1f}s" if t is not None
              else f"diag={node.diag_level} agg={node.agg_level}")

        rate_hz[0] = 50.0
        t = node.wait_until(
            lambda: node.diag_level == LEVEL_OK
            and node.agg_level == LEVEL_OK, 15)
        check("4 resume 50 Hz: diagnostic + aggregator recover OK",
              t is not None,
              f"after {t:.1f}s" if t is not None
              else f"diag={node.diag_level} agg={node.agg_level}")
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        stop[0] = True
        kill_all()
        if sum(1 for _, ok in RESULTS if ok) != 4:
            print(f"DEBUG agg_msgs={node.agg_msgs}", flush=True)
            print(f"DEBUG agg_names={sorted(node.agg_names)}", flush=True)
        node.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F104 acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
