"""测试套件公共辅助：结果报告、ROS 等待工具、MQTT 配置。"""

import json
import os
import time

# ---- MQTT / 设备契约（与 src/a3_mqtt_bridge/config/bridge.yaml 对齐）----
MQTT_HOST = os.environ.get("A3_MQTT_HOST", "bluemac.local")
MQTT_PORT = int(os.environ.get("A3_MQTT_PORT", "1883"))
TOPIC_PREFIX = "deep-trace/HOME-DEMO/RK3588"
TOPIC_TELEMETRY = f"{TOPIC_PREFIX}/telemetry"
TOPIC_CMD = f"{TOPIC_PREFIX}/cmd"
TOPIC_CMD_RESULT = f"{TOPIC_PREFIX}/cmd_result"

JOINT_NAMES = [
    "L1_joint", "L2_joint", "L3_joint", "L4_joint",
    "L5_joint", "L6_joint", "L7_joint",
]


class Reporter:
    """收集 PASS/FAIL 并打印汇总。"""

    def __init__(self, title):
        self.title = title
        self.results = []   # (name, ok, detail)

    def check(self, name, ok, detail=""):
        ok = bool(ok)
        self.results.append((name, ok, detail))
        tag = "PASS" if ok else "FAIL"
        line = f"[{tag}] {name}"
        if detail:
            line += f"  -- {detail}"
        print(line, flush=True)
        return ok

    def info(self, msg):
        print(f"[INFO] {msg}", flush=True)

    def warn(self, msg):
        print(f"[WARN] {msg}", flush=True)

    def summary(self):
        n_pass = sum(1 for _, ok, _ in self.results if ok)
        n_fail = len(self.results) - n_pass
        print("\n" + "=" * 60, flush=True)
        print(f"{self.title}: {n_pass} PASS / {n_fail} FAIL "
              f"(共 {len(self.results)} 项)", flush=True)
        print("=" * 60, flush=True)
        for name, ok, detail in self.results:
            if not ok:
                print(f"  FAIL: {name} -- {detail}", flush=True)
        return n_fail == 0


def wait_for_topic(node, topic, msg_type, timeout=10.0):
    """等待某个 topic 出现发布者，返回是否就绪。"""
    import rclpy
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        names = [n for n, _ in node.get_topic_names_and_types()]
        if topic in names:
            return True
    return False


def call_service(node, srv_type, name, request, timeout=10.0):
    """同步调用 ROS 服务，返回 response 或 None。"""
    import rclpy
    cli = node.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=timeout):
        node.destroy_client(cli)
        return None
    fut = cli.call_async(request)
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if fut.done():
            try:
                return fut.result()
            except Exception:
                return None
    node.destroy_client(cli)
    return None


def read_l7_position(node, timeout=3.0):
    """从 /joint_states 读取一次 L7_joint 位置（rad），返回 float 或 None。"""
    import math
    import rclpy
    from sensor_msgs.msg import JointState

    box = {"value": None, "stamp": None}

    def cb(msg):
        if "L7_joint" in msg.name:
            i = msg.name.index("L7_joint")
            if i < len(msg.position):
                v = msg.position[i]
                if math.isfinite(v):
                    box["value"] = v
                    box["stamp"] = time.time()

    sub = node.create_subscription(JointState, "/joint_states", cb, 10)
    deadline = time.time() + timeout
    while time.time() < deadline and box["value"] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return box["value"]


def mqtt_connect(client_id, timeout=8.0):
    """连接 EMQX 并返回 paho client（已 loop_start）。失败抛异常。"""
    import paho.mqtt.client as mqtt

    cli = mqtt.Client(client_id=client_id, clean_session=True)
    cli.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
    cli.loop_start()
    time.sleep(0.6)
    return cli
