#!/usr/bin/env python3
"""F86 acceptance: motor-side CAN timeout (0x7028 Type-18) over vcan.

Self-contained: launches vcan_motor_sim and the product stack
(a3_bringup.launch.py hardware:=can, motor_can_timeout_enabled:=true),
runs criteria 1-5, hard-kills the stack (emulating host death — SIGTERM
would trigger on_deactivate, which deliberately disarms), restarts with
motor_can_timeout_enabled:=false, runs criterion 6. Arm stays powered off.

  python3 scripts/a3_test/f86_can_timeout_acceptance.py [DOMAIN_ID]

Acceptance (docs/edge/REQUIREMENTS.md F86):
  1. 7 Type-18 0x7028 frames: data[0..1]=28 70, data[2..3]=00 00,
     data[4..8] LE uint32 ≈ 4000 +-10%; per motor ordered after the
     0x7005 RUN_MODE write and before the Type-3 enable frame
  2. Type-17 readback of 0x7028 ≈ 4000
  3. no trips during healthy operation; after SIGKILL of the stack all
     7 motors trip with trip_delay in [0.18, 0.7] s
  4. armed stack: quintic FJT error_code=0
  5. (label order per REQUIREMENTS) restart armed:=false: 0x7028 written
     as 0; after SIGKILL no motor trips within 1.2 s

Exit code 0 = all checks passed.
"""

import json
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "86"
IFACE = os.environ.get("F86_CAN_IF", "vcan6")
STATE_FILE = os.environ.get("F86_STATE_FILE", "/tmp/f86_timeout_state.json")
STACK_LOG = "/tmp/f86_stack.log"
STACK2_LOG = "/tmp/f86_stack_disarmed.log"
SIM_LOG = "/tmp/f86_sim.log"
RESULTS = []

CAN_FORMAT = "=IB3x8s"
CAN_EFF_FLAG = 0x80000000
CMD_ENABLE = 0x03
CMD_GET_PARAM = 0x11
CMD_SET_PARAM = 0x12
MASTER_ID = 0xFD
PARAM_RUN_MODE = 0x7005
PARAM_CAN_TIMEOUT = 0x7028
EXPECTED_COUNTS = 4000


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


class ProcessGroup:
    def __init__(self, argv, log_path, env=None):
        self.log = open(log_path, "wb")
        self.proc = subprocess.Popen(
            argv, stdout=self.log, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True, env=env)

    def terminate(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)

    def hard_kill(self):
        # Emulate sudden host death: no on_deactivate / disarm frames.
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc.wait(timeout=10)
        self.log.close()


def make_env(domain):
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["PYTHONNOUSERSITE"] = "1"
    return env


def ensure_vcan(interface):
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "add", "dev", interface,
         "type", "vcan"], input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "set", interface, "up"],
        input=b"temppwd\n", capture_output=True)


def spin(node, t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(node, cli, request, timeout=30.0):
    fut = cli.call_async(request)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    raise RuntimeError(f"service timeout: {cli.srv_name}")


def pack_frame(can_id, data):
    return struct.pack(CAN_FORMAT, can_id | CAN_EFF_FLAG, 8, bytes(data))


def read_state():
    try:
        with open(STATE_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


class ParamRecorder(threading.Thread):
    """Records Type-18 parameter writes and Type-3 enable frames per motor."""

    def __init__(self, interface):
        super().__init__(daemon=True)
        self.interface = interface
        self.stop_ev = threading.Event()
        self.lock = threading.Lock()
        self.events = {i: [] for i in range(1, 8)}
        self.sock = None

    def run(self):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((self.interface,))
        self.sock.settimeout(0.2)
        while not self.stop_ev.is_set():
            try:
                frame = self.sock.recv(72)
            except socket.timeout:
                continue
            except OSError:
                break
            can_id, _dlc, data = struct.unpack(CAN_FORMAT, frame)
            can_id &= 0x1FFFFFFF
            cmd_type = (can_id >> 24) & 0x1F
            motor_id = can_id & 0xFF
            if not (1 <= motor_id <= 7):
                continue
            if cmd_type == CMD_SET_PARAM:
                param = data[0] | (data[1] << 8)
                if param in (PARAM_RUN_MODE, PARAM_CAN_TIMEOUT):
                    value = data[4] | (data[5] << 8) | (data[6] << 16) | \
                        (data[7] << 24)
                    rec = (time.monotonic(), "set", param, value, bytes(data))
                else:
                    continue
            elif cmd_type == CMD_ENABLE:
                rec = (time.monotonic(), "enable", None, None, None)
            else:
                continue
            with self.lock:
                self.events[motor_id].append(rec)

    def motor_events(self, motor_id, since=0.0):
        with self.lock:
            return [e for e in self.events[motor_id] if e[0] >= since]

    def mark(self):
        with self.lock:
            for lst in self.events.values():
                lst.clear()

    def stop(self):
        self.stop_ev.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass


def readback_param(interface, motor_id):
    """Send a raw Type-17 request, wait for the motor reply, return value."""
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((interface,))
        sock.settimeout(0.5)
        can_id = (CMD_GET_PARAM << 24) | (MASTER_ID << 8) | motor_id
        data = [PARAM_CAN_TIMEOUT & 0xFF, (PARAM_CAN_TIMEOUT >> 8) & 0xFF,
                0, 0, 0, 0, 0, 0]
        sock.send(pack_frame(can_id, data))
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            try:
                frame = sock.recv(72)
            except socket.timeout:
                continue
            rid, _dlc, rdata = struct.unpack(CAN_FORMAT, frame)
            rid &= 0x1FFFFFFF
            if (rid >> 24) & 0x1F != CMD_GET_PARAM:
                continue
            if (rid >> 8) & 0xFF != motor_id or (rid & 0xFF) != MASTER_ID:
                continue
            param = rdata[0] | (rdata[1] << 8)
            if param != PARAM_CAN_TIMEOUT:
                continue
            return rdata[4] | (rdata[5] << 8) | (rdata[6] << 16) | \
                (rdata[7] << 24)
        return None
    finally:
        sock.close()


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f86_acceptance")
        self.status = {"state": None}
        self.create_subscription(JointState, "/joint_states",
                                 lambda msg: None, 10)
        from a3_msgs.msg import ArmStatus
        self.create_subscription(
            ArmStatus, "/a3/arm_status",
            lambda m: self.status.__setitem__("state", m.state), 10)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")

    def enable_to_ready(self):
        assert self.enable_cli.wait_for_service(timeout_sec=20), \
            "enable service absent"
        resp = call(self, self.enable_cli, Trigger.Request(), timeout=30)
        if not resp.success:
            raise RuntimeError(f"enable rejected: {resp.message}")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30:
            spin(self, 0.1)
            if self.status["state"] == "READY":
                return
        raise RuntimeError(f"not READY: {self.status['state']}")


def wait_js(node, timeout_s=25):
    from rclpy.qos import qos_profile_sensor_data
    got = {"v": False}
    node.create_subscription(
        JointState, "/joint_states", lambda msg: got.__setitem__("v", True),
        qos_profile_sensor_data)
    t0 = time.monotonic()
    while not got["v"] and time.monotonic() - t0 < timeout_s:
        spin(node, 0.1)
    return got["v"]


def start_stack(env, armed, log_path):
    args = ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
            "hardware:=can", f"can_interface:={IFACE}",
            f"motor_can_timeout_enabled:={'true' if armed else 'false'}",
            "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"]
    return ProcessGroup(args, log_path, env=env)


def check_armed_frames(rec, since):
    """Criterion 1: value + byte layout + ordering per motor."""
    all_ok = True
    for motor in range(1, 8):
        evs = rec.motor_events(motor, since=since)
        set05 = next((e for e in evs
                      if e[1] == "set" and e[2] == PARAM_RUN_MODE), None)
        set28 = next((e for e in evs
                      if e[1] == "set" and e[2] == PARAM_CAN_TIMEOUT), None)
        enable = next((e for e in evs if e[1] == "enable"), None)
        if set28 is None:
            check(f"1 motor {motor} 0x7028 frame present", False)
            all_ok = False
            continue
        _, _, _, value, raw = set28
        bytes_ok = raw[0] == 0x28 and raw[1] == 0x70 and \
            raw[2] == 0x00 and raw[3] == 0x00
        value_ok = abs(value - EXPECTED_COUNTS) / EXPECTED_COUNTS <= 0.10
        order_ok = set05 is not None and enable is not None and \
            set05[0] < set28[0] < enable[0]
        check(f"1 motor {motor} 0x7028 layout", bytes_ok, raw[:4].hex())
        check(f"1 motor {motor} counts ≈{EXPECTED_COUNTS} +-10%", value_ok,
              f"counts={value}")
        check(f"1 motor {motor} order 0x7005<0x7028<enable", order_ok)
        if not (bytes_ok and value_ok and order_ok):
            all_ok = False
    check("1 all 7 motors armed correctly", all_ok)


def wait_trips(expected, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        st = read_state()
        if st is not None:
            tripped = [int(k) for k, v in st["motors"].items()
                       if v["tripped"]]
            if len(tripped) >= expected:
                return tripped, time.monotonic() - t0
        time.sleep(0.02)
    st = read_state()
    tripped = [int(k) for k, v in (st["motors"].items() if st else [])
               if v["tripped"]]
    return tripped, timeout


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    ensure_vcan(IFACE)
    try:
        os.remove(STATE_FILE)
    except OSError:
        pass
    env = make_env(DOMAIN)

    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", IFACE, "--state-file", STATE_FILE],
        SIM_LOG, env=env)
    time.sleep(0.8)
    stack = start_stack(env, True, STACK_LOG)

    rclpy.init()
    node = HarnessNode()
    rec = ParamRecorder(IFACE)
    rec.start()

    fatal = False
    try:
        if not wait_js(node):
            raise RuntimeError("no /joint_states in 25 s")
        t_activate = time.monotonic()
        node.enable_to_ready()
        spin(node, 1.0)

        check_armed_frames(rec, t_activate)

        values = []
        for motor in range(1, 8):
            values.append(readback_param(IFACE, motor))
        ok = all(v is not None and abs(v - EXPECTED_COUNTS) /
                 EXPECTED_COUNTS <= 0.10 for v in values)
        check("2 Type-17 readback ≈4000 for all motors", ok,
              f"{values}")

        # No spurious trips during healthy 200 Hz operation.
        time.sleep(1.0)
        st = read_state()
        early = [int(k) for k, v in st["motors"].items() if v["tripped"]] \
            if st else []
        check("3a no trips while host healthy", not early, f"{early}")

        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from f83_enable_choreography_acceptance import move_small
        err_arm, err_grip = move_small(node)
        check("4 quintic FJT error_code=0",
              err_arm == 0 and err_grip == 0, f"err={err_arm}/{err_grip}")

        # Host death: hard kill, motors must self-trip.
        stack.hard_kill()
        tripped, dt = wait_trips(7, 1.5)
        st = read_state()
        delays = {int(k): v["trip_delay"]
                  for k, v in st["motors"].items()} if st else {}
        window_ok = all(d is not None and 0.18 <= d <= 0.7
                        for d in delays.values())
        check("3b all 7 motors trip after host death", len(tripped) == 7,
              f"{sorted(tripped)} in {dt:.2f}s")
        check("3c trip_delay in [0.18,0.7] s", window_ok,
              f"{ {k: round(d, 3) if d else None for k, d in delays.items()} }")

    except Exception:
        import traceback
        traceback.print_exc()
        fatal = True
    finally:
        if stack.proc.poll() is None:
            stack.hard_kill()

    # ---- disarmed-config restart ----
    if not fatal:
        stack2 = start_stack(env, False, STACK2_LOG)
        try:
            if not wait_js(node):
                raise RuntimeError("disarmed stack: no /joint_states")
            rec.mark()
            t_activate = time.monotonic()
            node.enable_to_ready()
            spin(node, 1.0)

            all_zero = True
            for motor in range(1, 8):
                evs = rec.motor_events(motor, since=t_activate)
                set28 = next((e for e in evs
                              if e[1] == "set" and e[2] == PARAM_CAN_TIMEOUT),
                             None)
                if set28 is None or set28[3] != 0:
                    all_zero = False
            check("5a 0x7028 written as 0 (disarmed)", all_zero)

            stack2.hard_kill()
            time.sleep(1.2)
            st = read_state()
            tripped = [int(k) for k, v in st["motors"].items()
                       if v["tripped"]] if st else []
            check("5b no trips within 1.2 s after host death (disarmed)",
                  not tripped, f"{tripped}")
        except Exception:
            import traceback
            traceback.print_exc()
            fatal = True
        finally:
            if stack2.proc.poll() is None:
                stack2.hard_kill()

    sim.terminate()
    rec.stop()
    node.destroy_node()
    rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F86 acceptance: {passed}/{total} ====", flush=True)
    if fatal:
        return 2
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
