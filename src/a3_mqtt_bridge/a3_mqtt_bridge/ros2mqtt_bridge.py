#!/usr/bin/env python3
"""ROS2 → MQTT telemetry bridge for A3 arm (rk3588).

Subscribes to a configurable whitelist of ROS topics, flattens each message
into a ``points`` dict, and publishes:

- ``<prefix>/device/info``   (retained)  board system info + node/topic/signal catalog
- ``<prefix>/device/status`` (~1 Hz)     heartbeat + cpu/mem/temp + running nodes
- ``<prefix>/telemetry``     (≤5 Hz, F35) flattened ``points``, latest-wins throttled

Also subscribes ``<prefix>/cmd`` as a stub for future bidirectional control.

See docs/edge/REQUIREMENTS.md F18 and config/bridge.yaml.
"""

import importlib
import json
import math
import os
import queue
import socket
import threading
import time
import uuid
from datetime import datetime, timezone

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:  # Humble: RCLError 只在私有编译模块暴露（SIGINT 竞态兜底，见 main()）
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:  # pragma: no cover - 其他发行版可能没有该符号
    RCLError = None
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from a3_msgs.srv import (
    GotoNamedPose,
    GripperCommand,
    GripperSetConfig,
    PlaybackTrajectory,
    SaveTrajectory,
    SetJointPositions,
)

# 电机调试（F32）：a3_can_bridge 服务
from a3_can_bridge.srv import (
    MotorCommand,
    MotorMitCommand,
    MotorScanCollect,
    MotorSetMode,
    MotorStop,
    SetMotorParam,
)


def _load_msg_class(type_str: str):
    """'sensor_msgs/msg/JointState' -> the message class."""
    module_path, cls_name = type_str.rsplit("/", 1)
    module = importlib.import_module(module_path.replace("/", "."))
    return getattr(module, cls_name)


def _short_joint(name: str) -> str:
    return name[:-6] if name.endswith("_joint") else name


def _motor_id_arg(args: dict):
    """motor 必须 int 1..127（F32 安全约束）；非法返回 None。"""
    try:
        motor = int(args.get("motor"))
    except (TypeError, ValueError):
        return None
    return motor if 1 <= motor <= 127 else None


def _motor_int_arg(args: dict, key: str, default: int) -> int:
    try:
        return int(args.get(key, default))
    except (TypeError, ValueError):
        return default


def _motor_float_arg(args: dict, key: str, default: float) -> float:
    try:
        val = float(args.get(key, default))
        return val if math.isfinite(val) else default
    except (TypeError, ValueError):
        return default


def _motor_hex_arg(args: dict, key: str):
    """index 支持十进制 int 或 '0x7005' 十六进制字符串；非法返回 None。"""
    raw = args.get(key)
    try:
        if isinstance(raw, str) and raw.strip().lower().startswith("0x"):
            return int(raw.strip(), 16)
        val = int(raw)
        return val if 0 <= val <= 0xFFFF else None
    except (TypeError, ValueError):
        return None


def _now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def _sanitize_nonfinite(obj):
    """递归把 NaN/±Inf 浮点替换为 None，保证 allow_nan=False 序列化不抛（LL-011）。"""
    if isinstance(obj, float):
        if obj != obj or obj in (float("inf"), float("-inf")):
            return None
        return obj
    if isinstance(obj, dict):
        return {k: _sanitize_nonfinite(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize_nonfinite(v) for v in obj]
    return obj


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
        # F35：遥测全局最小发布间隔（最新值胜出）；cmd/cmd_result/status/info 不受限
        self._telemetry_min_interval = float(
            config.get("telemetry_min_interval_sec", 0.2)
        )

        self._mqtt = None
        self._mqtt_connected = False
        # 信号缓存：各话题回调只更新自己的信号，telemetry 始终发布完整聚合 points，
        # 避免不同话题各自发一条 points 不完整的 telemetry，导致前端信号时有时无。
        self._points_cache: dict = {}
        self._telemetry_dirty = False
        self._last_telemetry_pub = 0.0
        self._catalog = self._build_catalog()

        self._cmd_queue: "queue.Queue[dict]" = queue.Queue()
        self._cmd_clients = self._setup_cmd_clients()

        # F35：发布解耦——所有 MQTT 发布经有界队列交独立线程发送，慢 socket / JSON
        # 串行化不再阻塞单线程 executor（遥测回调、cmd 分发、服务回执都在其上）。
        # telemetry 可丢（最新值胜出），cmd_result/status/info 永不丢。
        self._pub_queue: "queue.Queue" = queue.Queue(maxsize=256)
        self._closing = False  # destroy_node 置位后发布线程不再触碰日志器
        self._pub_thread = threading.Thread(
            target=self._publish_loop, name="a3-mqtt-publish", daemon=True
        )
        self._pub_thread.start()

        self._setup_mqtt()
        self._setup_subscriptions()

        self.create_timer(status_interval, self._publish_status)
        self.create_timer(info_interval, self._publish_info)
        self.create_timer(0.05, self._drain_cmds)
        self.create_timer(0.05, self._flush_telemetry)

        self.get_logger().info(
            f"bridge up: {self.topic_prefix} -> {self.host}:{self.port} "
            f"({len(self.topics_rules)} ROS topics)"
        )

    # ------------------------------------------------------------------ MQTT

    def destroy_node(self) -> bool:
        # 发哨兵停止发布线程，避免销毁后仍触碰 ROS 日志器
        self._closing = True
        try:
            self._pub_queue.put_nowait(None)
        except queue.Full:
            pass
        try:
            self._mqtt.disconnect()  # 让慢 publish 尽快返回，join 不必等满 2 s
        except Exception:
            pass
        self._pub_thread.join(timeout=2.0)
        return super().destroy_node()

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

        def on_disconnect(client, userdata, disconnect_flags=None, rc=None, properties=None):
            # paho 2.x (CallbackAPIVersion.VERSION2) 调用签名为 5 参数：
            # (client, userdata, disconnect_flags, reason_code, properties)；
            # 旧版 1.x 为 3 参数。缺 disconnect_flags 会 TypeError 导致断线后无法重连。
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

    def _publish(self, topic: str, payload, retain: bool = False, droppable: bool = False):
        """入队给发布线程；任何线程（executor / paho 回调）调用都安全（F35）。

        droppable=True（仅遥测）：队列满直接丢新值（最新值胜出，下个 flush 会补）。
        其余（cmd_result/status/info）：队列满挤掉最旧一条重试一次，保证不丢。
        """
        item = (topic, payload, retain)
        try:
            self._pub_queue.put_nowait(item)
            return True
        except queue.Full:
            if droppable:
                return False
            try:
                self._pub_queue.get_nowait()  # 挤掉最旧
                self._pub_queue.put_nowait(item)
                return True
            except queue.Full:
                self.get_logger().warning(f"publish queue full, dropped {topic}")
                return False

    def _publish_loop(self) -> None:
        """发布消费线程：JSON 串行化 + paho publish 全部在此执行（F35）。"""
        while True:
            item = self._pub_queue.get()
            if item is None:  # 关闭哨兵
                return
            topic, payload, retain = item
            if not self._mqtt or not self._mqtt_connected:
                continue
            try:
                # allow_nan=False 双保险：非有限浮点必须先清洗（LL-011）。漏网时抛异常落日志，
                # 而不是把非法 JSON（"vel_L1": NaN）污染给所有订阅者（浏览器 JSON.parse 整包丢弃）。
                body = payload if isinstance(payload, str) else json.dumps(
                    _sanitize_nonfinite(payload), allow_nan=False
                )
                self._mqtt.publish(topic, body, qos=0, retain=retain)
            except Exception as exc:  # noqa: BLE001
                if not self._closing:  # 销毁后日志器失效，避免守护线程再触碰
                    self.get_logger().warning(f"publish failed {topic}: {exc}")

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
            "set_joints": (
                self.create_client(SetJointPositions, "/a3/arm/set_joint_positions"),
                SetJointPositions.Request,
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
            # 夹爪（F26/F31）：command 服务被 grasp/release/stop/set_position 四个 op 共用
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
            "gripper_set_position": (
                self.create_client(GripperCommand, "/a3/gripper/command"),
                GripperCommand.Request,
            ),
            "gripper_set_max_torque": (
                self.create_client(GripperSetConfig, "/a3/gripper/set_config"),
                GripperSetConfig.Request,
            ),
            # 电机调试（F32）：扫描/使能/复位/设零/MIT/保持/停止/模式/参数
            "motor_scan": (
                self.create_client(MotorScanCollect, "/a3/motor/scan_and_collect"),
                MotorScanCollect.Request,
            ),
            "motor_enable": (
                self.create_client(MotorCommand, "/a3/motor/enable"),
                MotorCommand.Request,
            ),
            "motor_reset": (
                self.create_client(MotorCommand, "/a3/motor/reset"),
                MotorCommand.Request,
            ),
            "motor_set_zero": (
                self.create_client(MotorCommand, "/a3/motor/set_zero"),
                MotorCommand.Request,
            ),
            "motor_mit": (
                self.create_client(MotorMitCommand, "/a3/motor/mit_command"),
                MotorMitCommand.Request,
            ),
            "motor_hold": (
                self.create_client(MotorMitCommand, "/a3/motor/mit_command"),
                MotorMitCommand.Request,
            ),
            "motor_stop": (
                self.create_client(MotorStop, "/a3/motor/stop"),
                MotorStop.Request,
            ),
            "motor_set_mode": (
                self.create_client(MotorSetMode, "/a3/motor/set_mode"),
                MotorSetMode.Request,
            ),
            "motor_set_param": (
                self.create_client(SetMotorParam, "/a3/motor/set_param"),
                SetMotorParam.Request,
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
        elif op == "gripper_set_position":
            # 位置模式直驱（F31）：0..1 归一化开合，越界/非有限明确拒绝而非 clamp（可测）
            try:
                p = float(args.get("position"))
            except (TypeError, ValueError):
                self._publish_cmd_result(op, False, "position must be in [0, 1]")
                return
            if not math.isfinite(p) or not 0.0 <= p <= 1.0:
                self._publish_cmd_result(op, False, "position must be in [0, 1]")
                return
            req.mode = "position"
            req.position = p
        elif op == "gripper_set_max_torque":
            req.key = "max_torque_nm"
            try:
                req.value = float(args.get("value"))
            except (TypeError, ValueError):
                self._publish_cmd_result(op, False, "invalid torque value")
                return
        elif op == "set_joints":
            positions = args.get("positions")
            if not isinstance(positions, (list, tuple)) or len(positions) != 7:
                self._publish_cmd_result(op, False, "set_joints needs positions[7]")
                return
            try:
                req.positions = [float(v) for v in positions]
            except (TypeError, ValueError):
                self._publish_cmd_result(op, False, "invalid positions value")
                return
            try:
                req.duration = float(args.get("duration") or 0.3)
            except (TypeError, ValueError):
                req.duration = 0.3
        elif op == "motor_scan":
            req.id_min = _motor_int_arg(args, "id_min", 1)
            req.id_max = _motor_int_arg(args, "id_max", 127)
            req.bus = _motor_int_arg(args, "bus", 1)
            req.timeout_s = _motor_float_arg(args, "timeout_s", 1.5)
        elif op in ("motor_enable", "motor_reset", "motor_set_zero"):
            motor = _motor_id_arg(args)
            if motor is None:
                self._publish_cmd_result(op, False, "motor must be int in [1, 127]")
                return
            req.motor_id = motor
            req.command = {"motor_enable": 1, "motor_reset": 2, "motor_set_zero": 3}[op]
        elif op in ("motor_mit", "motor_hold"):
            motor = _motor_id_arg(args)
            if motor is None:
                self._publish_cmd_result(op, False, "motor must be int in [1, 127]")
                return
            req.motor_id = motor
            req.position_rad = _motor_float_arg(args, "p", 0.0)
            req.velocity_rad_s = _motor_float_arg(args, "v", 0.0)
            req.kp = _motor_float_arg(args, "kp", 20.0)
            req.kd = _motor_float_arg(args, "kd", 1.0)
            req.torque_ff_nm = _motor_float_arg(args, "t", 0.0)
            if op == "motor_hold":
                req.hold_duration_s = _motor_float_arg(args, "duration_s", 0.0)
                req.hold_hz = _motor_float_arg(args, "hz", 0.0)
        elif op == "motor_stop":
            req.motor_id = _motor_int_arg(args, "motor", 0)
            if not 0 <= req.motor_id <= 127:
                self._publish_cmd_result(op, False, "motor must be int in [0, 127]")
                return
        elif op == "motor_set_mode":
            motor = _motor_id_arg(args)
            if motor is None:
                self._publish_cmd_result(op, False, "motor must be int in [1, 127]")
                return
            mode = str(args.get("mode") or "").strip().lower()
            if mode not in ("mit", "position", "speed"):
                self._publish_cmd_result(op, False, "mode must be mit|position|speed")
                return
            req.motor_id = motor
            req.mode = mode
            req.position_rad = _motor_float_arg(args, "position", 0.0)
            req.limit_speed_rad_s = _motor_float_arg(args, "limit_spd", 5.0)
            req.speed_rad_s = _motor_float_arg(args, "speed", 0.0)
        elif op == "motor_set_param":
            motor = _motor_id_arg(args)
            if motor is None:
                self._publish_cmd_result(op, False, "motor must be int in [1, 127]")
                return
            index = _motor_hex_arg(args, "index")
            if index is None:
                self._publish_cmd_result(op, False, "index must be int or hex string")
                return
            req.motor_id = motor
            req.param_id = index
            req.value = _motor_float_arg(args, "value", 0.0)
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
            if op == "motor_scan" and ok:
                # 扫描结果重编码为 JSON 电机列表（uid 大端十六进制串）
                motors = []
                ids = getattr(resp, "ids", [])
                uids = getattr(resp, "uids", [])
                for i, mid in enumerate(ids):
                    uid = uids[i] if i < len(uids) else 0
                    motors.append({"id": int(mid), "uid": f"{uid:016X}"})
                message = json.dumps({"motors": motors}, allow_nan=False)
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
        if rule.get("flatten") == "motor_state":
            # F32：6 前缀 × L1..L7 = 42 个信号（key 与 _make_callback 一致）
            prefixes = rule.get("prefixes") or []
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
                self._update_telemetry(points)

            return cb

        if flatten == "motor_state":
            # F32：/a3/motor/states 逐电机展平。points key = prefix_L{n}，n=motor_id（1..7）
            prefixes = rule.get("prefixes") or []
            sig_map = {
                "temp": lambda st: float(st.temperature_c),
                "err": lambda st: int(st.fault_mask),
                "mode": lambda st: int(st.mode_status),
                "online": lambda st: 1 if st.fresh else 0,
                "mtq": lambda st: float(st.torque_nm),
                "mp": lambda st: float(st.position_rad),
            }

            def cb(msg):
                points = {}
                for st in getattr(msg, "states", []):
                    n = int(st.motor_id)
                    for p in prefixes:
                        fn = sig_map.get(p)
                        if fn is not None:
                            points[f"{p}_L{n}"] = fn(st)
                self._update_telemetry(points)

            return cb

        names = rule.get("names") or fields

        def cb(msg):
            points = {}
            for fi, field in enumerate(fields):
                name = names[fi] if fi < len(names) else field
                points[name] = getattr(msg, field, None)
            self._update_telemetry(points)

        return cb

    def _update_telemetry(self, points: dict):
        """合并信号到缓存并置脏；由 _flush_telemetry 定时统一发布（F35 降频）。"""
        self._points_cache.update(points)
        self._telemetry_dirty = True

    def _flush_telemetry(self) -> None:
        """F35：脏且距上次发布 ≥ 最小间隔 → 发布完整 points（最新值胜出）。"""
        if not self._telemetry_dirty:
            return
        now = time.time()
        if now - self._last_telemetry_pub < self._telemetry_min_interval:
            return
        self._telemetry_dirty = False
        self._last_telemetry_pub = now
        self._publish_telemetry(dict(self._points_cache))

    def _publish_telemetry(self, points: dict):
        payload = {
            "home_code": self.home_code,
            "work_unit_code": self.work_unit_code,
            "device_program": self.program,
            "ts": _now_iso(),
            "points": points,
        }
        self._publish(self.topic_telemetry, payload, droppable=True)


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
    except Exception as exc:
        # SIGINT 与 WaitSet 创建竞态（Humble）：信号处理已调用 rclpy.shutdown()，
        # executor 在 context 失效后创建 WaitSet 会抛 RCLError —— 视为正常退出。
        if RCLError is not None and isinstance(exc, RCLError):
            pass
        else:
            raise
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
