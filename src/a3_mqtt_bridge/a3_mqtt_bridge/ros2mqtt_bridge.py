#!/usr/bin/env python3
"""ROS2 → MQTT telemetry bridge for A3 arm (rk3588).

Subscribes to a configurable whitelist of ROS topics, flattens each message
into a ``points`` dict, and publishes:

- ``<prefix>/device/info``   (retained)  board system info + node/topic/signal catalog
- ``<prefix>/device/status`` (~1 Hz)     heartbeat + cpu/mem/temp + running nodes
- ``<prefix>/telemetry``     (per msg)   flattened ``points``

Also subscribes ``<prefix>/cmd`` as a stub for future bidirectional control.

See docs/edge/REQUIREMENTS.md F18 and config/bridge.yaml.
"""

import importlib
import json
import os
import queue
import socket
import time
import uuid
from datetime import datetime, timezone

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from a3_msgs.srv import (
    GotoNamedPose,
    GripperCommand,
    GripperSetConfig,
    PlaybackTrajectory,
    SaveTrajectory,
)


def _load_msg_class(type_str: str):
    """'sensor_msgs/msg/JointState' -> the message class."""
    module_path, cls_name = type_str.rsplit("/", 1)
    module = importlib.import_module(module_path.replace("/", "."))
    return getattr(module, cls_name)


def _short_joint(name: str) -> str:
    return name[:-6] if name.endswith("_joint") else name


def _now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def _first_ipv4() -> str:
    try:
        import psutil

        for _, addrs in psutil.net_if_addrs().items():
            for a in addrs:
                if a.family == socket.AF_INET and not a.address.startswith("127."):
                    return a.address
    except Exception:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return ""


def _first_mac() -> str:
    try:
        for iface in sorted(os.listdir("/sys/class/net")):
            if iface == "lo":
                continue
            with open(f"/sys/class/net/{iface}/address", encoding="utf-8") as f:
                addr = f.read().strip()
            if addr and addr != "00:00:00:00:00:00":
                return addr
    except Exception:
        pass
    node = uuid.getnode()
    if node:
        return ":".join(f"{(node >> 8 * i) & 0xFF:02x}" for i in reversed(range(6)))
    return ""


def _cpu_model() -> str:
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if line.startswith(("model name", "Hardware", "Processor")):
                    return line.split(":", 1)[1].strip()
    except Exception:
        pass
    import platform

    return platform.machine()


def _cpu_temp() -> float:
    try:
        import psutil

        temps = psutil.sensors_temperatures()
        for name, entries in temps.items():
            if entries:
                return entries[0].current
    except Exception:
        pass
    try:
        for zone in sorted(os.listdir("/sys/class/thermal")):
            if zone.startswith("thermal_zone"):
                with open(f"/sys/class/thermal/{zone}/temp") as f:
                    raw = f.read().strip()
                if raw.isdigit():
                    return int(raw) / 1000.0
    except Exception:
        pass
    return 0.0


class Ros2MqttBridge(Node):
    def __init__(self, config: dict):
        super().__init__("ros2mqtt_bridge")

        self._cfg = config
        mqtt_cfg = config.get("mqtt", {})
        device_cfg = config.get("device", {})

        self.host = mqtt_cfg.get("host", "192.168.3.73")
        self.port = int(mqtt_cfg.get("port", 1883))
        self.path = mqtt_cfg.get("path", "/mqtt")
        self.topic_prefix = (mqtt_cfg.get("topic_prefix") or "deep-trace/HOME-DEMO/RK3588").rstrip("/")
        client_id_prefix = mqtt_cfg.get("client_id_prefix", "a3-rk3588")
        self.client_id = f"{client_id_prefix}-{socket.gethostname()}-{uuid.uuid4().hex[:6]}"

        self.home_code = device_cfg.get("home_code", "HOME-DEMO")
        self.work_unit_code = device_cfg.get("work_unit_code", "RK3588")
        self.program = device_cfg.get("program", "rk3588")
        self.device_name = device_cfg.get("name", "RK3588 机械臂")

        self.topic_info = f"{self.topic_prefix}/device/info"
        self.topic_status = f"{self.topic_prefix}/device/status"
        self.topic_telemetry = f"{self.topic_prefix}/telemetry"
        self.topic_cmd = f"{self.topic_prefix}/cmd"
        self.topic_cmd_result = f"{self.topic_prefix}/cmd_result"

        self.topics_rules = config.get("topics", [])
        status_interval = float(config.get("status_interval_sec", 1.0))
        info_interval = float(config.get("info_interval_sec", 10.0))

        self._mqtt = None
        self._mqtt_connected = False
        self._catalog = self._build_catalog()

        self._cmd_queue: "queue.Queue[dict]" = queue.Queue()
        self._cmd_clients = self._setup_cmd_clients()

        self._setup_mqtt()
        self._setup_subscriptions()

        self.create_timer(status_interval, self._publish_status)
        self.create_timer(info_interval, self._publish_info)
        self.create_timer(0.05, self._drain_cmds)

        self.get_logger().info(
            f"bridge up: {self.topic_prefix} -> {self.host}:{self.port} "
            f"({len(self.topics_rules)} ROS topics)"
        )

    # ------------------------------------------------------------------ MQTT

    def _setup_mqtt(self):
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            self.get_logger().error("paho-mqtt not installed: pip install paho-mqtt")
            return

        try:
            from paho.mqtt.client import CallbackAPIVersion

            self._mqtt = mqtt.Client(
                callback_api_version=CallbackAPIVersion.VERSION2,
                client_id=self.client_id,
            )
        except ImportError:
            self._mqtt = mqtt.Client(client_id=self.client_id)

        if self._cfg.get("mqtt", {}).get("username"):
            self._mqtt.username_pw_set(
                self._cfg["mqtt"].get("username"),
                self._cfg["mqtt"].get("password", "") or "",
            )

        self._mqtt.will_set(self.topic_status, json.dumps({"online": False}), qos=0, retain=True)

        def on_connect(client, userdata, flags, rc, properties=None):
            code = int(getattr(rc, "value", rc))
            if code == 0:
                self._mqtt_connected = True
                self._mqtt.subscribe(self.topic_cmd, qos=0)
                self.get_logger().info(f"mqtt connected: {self.host}:{self.port}")
                self._publish_info()
            else:
                self._mqtt_connected = False
                self.get_logger().warning(f"mqtt connect failed rc={rc}")

        def on_disconnect(client, userdata, rc, properties=None):
            self._mqtt_connected = False
            self.get_logger().warning(f"mqtt disconnected rc={rc}")

        def on_message(client, userdata, msg):
            topic = getattr(msg, "topic", "")
            if topic == self.topic_cmd:
                self._enqueue_cmd(msg.payload)

        self._mqtt.on_connect = on_connect
        self._mqtt.on_disconnect = on_disconnect
        self._mqtt.on_message = on_message
        self._mqtt.reconnect_delay_set(min_delay=1, max_delay=30)

        try:
            self._mqtt.connect(self.host, self.port, keepalive=30)
            self._mqtt.loop_start()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"mqtt connect error: {exc}")

    def _publish(self, topic: str, payload, retain: bool = False):
        if not self._mqtt or not self._mqtt_connected:
            return False
        try:
            body = payload if isinstance(payload, str) else json.dumps(payload)
            self._mqtt.publish(topic, body, qos=0, retain=retain)
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"publish failed {topic}: {exc}")
            return False

    # ------------------------------------------------------------- downlink

    def _setup_cmd_clients(self) -> dict:
        """op -> (client, request_factory)。白名单仅限编排层服务。"""
        return {
            "init": (self.create_client(Trigger, "/a3/arm/init"), Trigger.Request),
            "enable": (self.create_client(Trigger, "/a3/arm/enable"), Trigger.Request),
            "disable": (self.create_client(Trigger, "/a3/arm/disable"), Trigger.Request),
            "goto": (
                self.create_client(GotoNamedPose, "/a3/arm/goto_named_pose"),
                GotoNamedPose.Request,
            ),
            "teach_start": (
                self.create_client(Trigger, "/a3/arm/start_teach"),
                Trigger.Request,
            ),
            "teach_stop": (self.create_client(Trigger, "/a3/arm/stop_teach"), Trigger.Request),
            "save": (
                self.create_client(SaveTrajectory, "/a3/arm/save_trajectory"),
                SaveTrajectory.Request,
            ),
            "playback": (
                self.create_client(PlaybackTrajectory, "/a3/arm/playback"),
                PlaybackTrajectory.Request,
            ),
            "enter_ai": (self.create_client(Trigger, "/a3/arm/enter_ai"), Trigger.Request),
            "exit_ai": (self.create_client(Trigger, "/a3/arm/exit_ai"), Trigger.Request),
            # 夹爪力控（F26）：command 服务被 grasp/release/stop 三个 op 共用
            "gripper_grasp": (
                self.create_client(GripperCommand, "/a3/gripper/command"),
                GripperCommand.Request,
            ),
            "gripper_release": (
                self.create_client(GripperCommand, "/a3/gripper/command"),
                GripperCommand.Request,
            ),
            "gripper_stop": (
                self.create_client(GripperCommand, "/a3/gripper/command"),
                GripperCommand.Request,
            ),
            "gripper_set_max_torque": (
                self.create_client(GripperSetConfig, "/a3/gripper/set_config"),
                GripperSetConfig.Request,
            ),
        }

    def _enqueue_cmd(self, payload) -> None:
        try:
            data = json.loads(payload.decode("utf-8"))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"cmd ignored (invalid json): {exc}")
            return
        if not isinstance(data, dict) or "op" not in data:
            self.get_logger().warning("cmd ignored (missing op)")
            return
        op = str(data["op"]).strip()
        if op not in self._cmd_clients:
            self.get_logger().warning(f"cmd ignored (unknown op {op!r})")
            self._publish_cmd_result(op, False, f"unknown op {op!r}")
            return
        self._cmd_queue.put({"op": op, "args": data.get("args") or {}})

    def _drain_cmds(self) -> None:
        while True:
            try:
                cmd = self._cmd_queue.get_nowait()
            except queue.Empty:
                return
            self._dispatch_cmd(cmd["op"], cmd["args"])

    def _dispatch_cmd(self, op: str, args: dict) -> None:
        client, req_factory = self._cmd_clients[op]
        req = req_factory()
        if op == "goto":
            req.pose_name = str(args.get("pose") or args.get("pose_name") or "")
        elif op in ("save", "playback"):
            req.name = str(args.get("name") or "")
        elif op == "gripper_grasp":
            req.mode = "force"
            if "torque" in args:
                try:
                    req.torque_nm = float(args.get("torque"))
                except (TypeError, ValueError):
                    self._publish_cmd_result(op, False, "invalid torque value")
                    return
            req.preset = str(args.get("preset") or "")
            if "timeout" in args:
                try:
                    req.timeout_s = float(args.get("timeout"))
                except (TypeError, ValueError):
                    pass
        elif op == "gripper_release":
            req.mode = "release"
        elif op == "gripper_stop":
            req.mode = "stop"
        elif op == "gripper_set_max_torque":
            req.key = "max_torque_nm"
            try:
                req.value = float(args.get("value"))
            except (TypeError, ValueError):
                self._publish_cmd_result(op, False, "invalid torque value")
                return
        if not client.service_is_ready():
            self._publish_cmd_result(op, False, f"{op} service unavailable")
            return
        future = client.call_async(req)
        if future is None:
            self._publish_cmd_result(op, False, f"{op} rejected")
            return
        future.add_done_callback(lambda f, o=op: self._on_cmd_done(f, o))

    def _on_cmd_done(self, future, op: str) -> None:
        ok = False
        message = "error"
        try:
            resp = future.result()
            ok = bool(resp.success)
            message = resp.message
        except Exception as exc:  # noqa: BLE001
            message = str(exc)
        self._publish_cmd_result(op, ok, message)

    def _publish_cmd_result(self, op: str, ok: bool, message: str) -> None:
        self._publish(
            self.topic_cmd_result,
            {"op": op, "ok": ok, "message": message, "ts": _now_iso()},
        )

    # ------------------------------------------------------------- catalog

    def _build_catalog(self):
        """node -> topics -> signals (for device/info.nodes and frontend cards)."""
        nodes: dict[str, list[dict]] = {}
        for rule in self.topics_rules:
            node_name = rule.get("node", "ros_node")
            signals = self._signals_for(rule)
            entry = {"topic": rule.get("topic"), "signals": signals}
            nodes.setdefault(node_name, []).append(entry)
        return [{"node": n, "topics": t} for n, t in nodes.items()]

    def _signals_for(self, rule) -> list[str]:
        if rule.get("flatten") == "joint_state":
            prefixes = rule.get("prefixes") or rule.get("fields", [])
            joints = rule.get("joints") or []
            shorts = [_short_joint(j) for j in joints]
            out = []
            for p in prefixes:
                out.extend(f"{p}_{s}" for s in shorts)
            return out
        return list(rule.get("names") or rule.get("fields", []))

    # -------------------------------------------------------------- system

    def _collect_system_info(self) -> dict:
        import platform

        uptime = 0.0
        try:
            import psutil

            uptime = round(time.time() - psutil.boot_time(), 1)
        except Exception:
            pass
        return {
            "hostname": socket.gethostname(),
            "ip": _first_ipv4(),
            "mac": _first_mac(),
            "cpu_model": _cpu_model(),
            "os": platform.platform(),
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "uptime_sec": uptime,
            "device_name": self.device_name,
        }

    def _publish_info(self):
        info = self._collect_system_info()
        info["home_code"] = self.home_code
        info["work_unit_code"] = self.work_unit_code
        info["program"] = self.program
        info["nodes"] = self._catalog
        info["ts"] = _now_iso()
        self._publish(self.topic_info, info, retain=True)

    def _publish_status(self):
        import psutil

        running = {name for name, _ in self.get_node_names_and_namespaces()}
        status = {
            "online": True,
            "ts": _now_iso(),
            "cpu_percent": psutil.cpu_percent(interval=None),
            "mem_percent": psutil.virtual_memory().percent,
            "temp_c": _cpu_temp(),
            "nodes": sorted(running),
        }
        self._publish(self.topic_status, status, retain=True)

    # ------------------------------------------------------------ ROS subs

    def _qos_for(self, rule) -> QoSProfile:
        qos = str(rule.get("qos") or "reliable").lower()
        if qos in ("best_effort", "best", "sensor_data", "sensor"):
            return QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

    def _setup_subscriptions(self):
        for rule in self.topics_rules:
            topic = rule.get("topic")
            type_str = rule.get("type")
            if not topic or not type_str:
                continue
            try:
                msg_cls = _load_msg_class(type_str)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"cannot load {type_str} for {topic}: {exc}")
                continue
            cb = self._make_callback(rule)
            self.create_subscription(msg_cls, topic, cb, self._qos_for(rule))
            self.get_logger().info(f"subscribed {topic} ({type_str})")

    def _make_callback(self, rule):
        flatten = rule.get("flatten", "scalar")
        fields = rule.get("fields", [])

        if flatten == "joint_state":
            prefixes = rule.get("prefixes") or fields
            joints = rule.get("joints") or []
            shorts = [_short_joint(j) for j in joints]

            def cb(msg):
                points = {}
                for fi, field in enumerate(fields):
                    prefix = prefixes[fi] if fi < len(prefixes) else field
                    arr = getattr(msg, field, None)
                    if arr is None:
                        continue
                    for idx, short in enumerate(shorts):
                        if idx >= len(arr):
                            break
                        points[f"{prefix}_{short}"] = arr[idx]
                self._publish_telemetry(points)

            return cb

        names = rule.get("names") or fields

        def cb(msg):
            points = {}
            for fi, field in enumerate(fields):
                name = names[fi] if fi < len(names) else field
                points[name] = getattr(msg, field, None)
            self._publish_telemetry(points)

        return cb

    def _publish_telemetry(self, points: dict):
        payload = {
            "home_code": self.home_code,
            "work_unit_code": self.work_unit_code,
            "device_program": self.program,
            "ts": _now_iso(),
            "points": points,
        }
        self._publish(self.topic_telemetry, payload)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        from ament_index_python.packages import get_package_share_directory

        import yaml

        share = get_package_share_directory("a3_mqtt_bridge")
        cfg_path = os.path.join(share, "config", "bridge.yaml")
        with open(cfg_path, encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
    except Exception as exc:  # noqa: BLE001
        print(f"[a3_mqtt_bridge] failed to load config: {exc}")
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return 1

    try:
        node = Ros2MqttBridge(config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
