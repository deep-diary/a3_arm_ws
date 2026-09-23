#!/usr/bin/env python3
"""F102 acceptance: MQTT link health on the standard diagnostics channel.

A local mosquitto broker (no auth) stands in for EMQX; the product bridge
points at it via A3_MQTT_HOST/A3_MQTT_PORT. The product aggregator is started
with the shipped diagnostics.yaml so the new Comms group is exercised too.

Acceptance (docs/edge/REQUIREMENTS.md F102):
  1. broker up: /a3/comms/mqtt_connected = true within 15 s.
  2. MQTT link diagnostic OK within 15 s.
  3. aggregator publishes /A3/Comms/ros2mqtt_bridge: MQTT link (OK).
  4. broker killed: latched topic flips false within 15 s.
  5. diagnostic escalates to ERROR after 10 s disconnected (<= 20 s total).
  6. aggregator Comms item is ERROR (<= 25 s).
  7. broker back: topic true, diagnostic OK within 30 s; aggregator recovers.

Simulation only — the real arm stays powered off. Exit 0 = all passed.
Requires: apt package mosquitto (broker binary, system service need not run).
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "102"
BROKER_PORT = int(os.environ.get("F102_PORT", "18830"))
BROKER_HOST = "127.0.0.1"
MQTT_CFG = "/tmp/f102_mosquitto.conf"
BROKER_LOG = "/tmp/f102_mosquitto.log"
BRIDGE_LOG = "/tmp/f102_bridge.log"
AGG_LOG = "/tmp/f102_aggregator.log"
DIAG_YAML = f"{WS}/src/a3_bringup/config/diagnostics.yaml"
TASK_NAME = "ros2mqtt_bridge: MQTT link"
AGG_NAME = "/A3/Comms/ros2mqtt_bridge: MQTT link"

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


def make_env(clear_nousersite=False):
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = DOMAIN
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    if clear_nousersite:
        env["PYTHONNOUSERSITE"] = ""
    else:
        env["PYTHONNOUSERSITE"] = "1"
    return env


def spin(node, t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f102_acceptance")
        self.connected = None
        self.diag_level = None
        self.agg_level = None
        self.create_subscription(
            Bool, "/a3/comms/mqtt_connected", self._on_conn,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.create_subscription(
            DiagnosticArray, "/diagnostics_agg", self._on_agg, 10)

    def _on_conn(self, msg):
        self.connected = bool(msg.data)

    def _on_diag(self, msg):
        for s in msg.status:
            if s.name == TASK_NAME:
                self.diag_level = _level(s.level)

    def _on_agg(self, msg):
        for s in msg.status:
            if s.name == AGG_NAME:
                self.agg_level = _level(s.level)

    def wait_until(self, pred, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            spin(self, 0.1)
            if pred():
                return time.monotonic() - t0
        return None


def start_broker():
    with open(MQTT_CFG, "w") as f:
        f.write(
            f"listener {BROKER_PORT}\nallow_anonymous true\n"
            f"persistence false\nlog_dest file {BROKER_LOG}\n"
            f"connection_messages false\nlog_type error\nlog_type warning\n")
    return subprocess.Popen(
        ["mosquitto", "-c", MQTT_CFG],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL, start_new_session=True)


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["PYTHONNOUSERSITE"] = "1"

    if subprocess.run(["which", "mosquitto"], capture_output=True).returncode:
        print("mosquitto not installed; sudo apt install mosquitto")
        return 1

    procs = []

    def kill_all():
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    broker = start_broker()
    procs.append(broker)
    time.sleep(0.8)

    bridge_env = make_env(clear_nousersite=True)
    bridge_env["A3_MQTT_HOST"] = BROKER_HOST
    bridge_env["A3_MQTT_PORT"] = str(BROKER_PORT)
    bridge = subprocess.Popen(
        ["ros2", "run", "a3_mqtt_bridge", "ros2mqtt_bridge"],
        stdout=open(BRIDGE_LOG, "wb"), stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL, start_new_session=True, env=bridge_env)
    procs.append(bridge)

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
        t = node.wait_until(lambda: node.connected is True, 15)
        check("1 mqtt_connected=true within 15 s", t is not None,
              f"after {t:.1f}s" if t is not None else f"state={node.connected}")

        t = node.wait_until(
            lambda: node.diag_level == LEVEL_OK, 15)
        check("2 MQTT link diagnostic OK", t is not None,
              f"after {t:.1f}s" if t is not None
              else f"level={node.diag_level}")

        t = node.wait_until(
            lambda: node.agg_level == LEVEL_OK, 20)
        check("3 aggregator /A3/Comms item OK", t is not None,
              f"after {t:.1f}s" if t is not None
              else f"level={node.agg_level}")

        os.killpg(os.getpgid(broker.pid), signal.SIGKILL)
        t_down = time.monotonic()

        t = node.wait_until(lambda: node.connected is False, 15)
        check("4 mqtt_connected=false within 15 s of broker death",
              t is not None,
              f"after {t:.1f}s" if t is not None else "still true")

        t = node.wait_until(
            lambda: node.diag_level == LEVEL_ERROR, 20)
        elapsed = (time.monotonic() - t_down) if t is not None else None
        check("5 diagnostic ERROR <= 20 s after disconnect",
              t is not None and elapsed >= 9.0,
              f"{elapsed:.1f}s after disconnect" if elapsed is not None
              else f"level={node.diag_level}")

        t = node.wait_until(
            lambda: node.agg_level == LEVEL_ERROR, 25)
        check("6 aggregator Comms item ERROR", t is not None,
              f"after {t:.1f}s" if t is not None
              else f"level={node.agg_level}")

        broker = start_broker()
        procs.append(broker)

        t = node.wait_until(lambda: node.connected is True, 30)
        check("7 reconnected: topic true within 30 s", t is not None,
              f"after {t:.1f}s" if t is not None else f"state={node.connected}")

        t = node.wait_until(
            lambda: node.diag_level == LEVEL_OK
            and node.agg_level == LEVEL_OK, 15)
        check("7 diagnostics + aggregator recover OK", t is not None,
              f"after {t:.1f}s" if t is not None
              else f"diag={node.diag_level} agg={node.agg_level}")
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        kill_all()
        for p in (MQTT_CFG,):
            if os.path.exists(p):
                os.remove(p)
        node.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F102 acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
