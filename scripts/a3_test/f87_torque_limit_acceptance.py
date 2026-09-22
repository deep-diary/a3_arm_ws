#!/usr/bin/env python3
"""F87 acceptance: firmware torque-limit arming (0x700B Type-18 float) over vcan.

Self-contained: launches vcan_motor_sim and the product stack
(a3_bringup.launch.py hardware:=can), runs criteria 1-5, hard-kills the
stack. Arm stays powered off.

  python3 scripts/a3_test/f87_torque_limit_acceptance.py [DOMAIN_ID]

Acceptance (docs/edge/REQUIREMENTS.md F87):
  1. 7 Type-18 0x700B frames: data[0..1]=0b 70, data[2..3]=00 00,
     data[4..8] LE IEEE754 float: motors 1-3 = 14.0 Nm, motors 4-7 =
     6.0 Nm (+-1%); exactly one frame per motor
  2. per-motor order: 0x7005 RUN_MODE < 0x700B < 0x7028 < Type-3 enable
  3. Type-17 readback of 0x700B is bit-exact vs the float written
  4. choreography regression: per motor one discovery reset (Type 4
     data[1]=0xC0) then one clear-fault (Type 4 data[0]=1) before
     RUN_MODE; 0x7028=4000; frame counts/order per F83/F86
  5. small quintic FJT error_code=0

Exit code 0 = all checks passed.
"""

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
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "87"
IFACE = os.environ.get("F87_CAN_IF", "vcan7")
STATE_FILE = os.environ.get("F87_STATE_FILE", "/tmp/f87_timeout_state.json")
STACK_LOG = "/tmp/f87_stack.log"
SIM_LOG = "/tmp/f87_sim.log"
RESULTS = []

CAN_FORMAT = "=IB3x8s"
CAN_EFF_FLAG = 0x80000000
CMD_RESET = 0x04
CMD_ENABLE = 0x03
CMD_GET_PARAM = 0x11
CMD_SET_PARAM = 0x12
MASTER_ID = 0xFD
PARAM_RUN_MODE = 0x7005
PARAM_TORQUE_LIMIT = 0x700B
PARAM_CAN_TIMEOUT = 0x7028
EXPECTED_NM = {i: (14.0 if i <= 3 else 6.0) for i in range(1, 8)}
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


class ParamRecorder(threading.Thread):
    """Records per motor: Type-4 resets, Type-18 writes (3 params), enables."""

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
                if param not in (PARAM_RUN_MODE, PARAM_TORQUE_LIMIT,
                                 PARAM_CAN_TIMEOUT):
                    continue
                value = data[4] | (data[5] << 8) | (data[6] << 16) | \
                    (data[7] << 24)
                rec = (time.monotonic(), "set", param, value, bytes(data))
            elif cmd_type == CMD_ENABLE:
                rec = (time.monotonic(), "enable", None, None, None)
            elif cmd_type == CMD_RESET:
                rec = (time.monotonic(), "reset", None, data[0], bytes(data))
            else:
                continue
            with self.lock:
                self.events[motor_id].append(rec)

    def motor_events(self, motor_id, since=0.0):
        with self.lock:
            return [e for e in self.events[motor_id] if e[0] >= since]

    def stop(self):
        self.stop_ev.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass


def readback_param(interface, motor_id, param_id):
    """Send a raw Type-17 request, wait for the motor reply, return raw u32."""
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((interface,))
        sock.settimeout(0.5)
        can_id = (CMD_GET_PARAM << 24) | (MASTER_ID << 8) | motor_id
        data = [param_id & 0xFF, (param_id >> 8) & 0xFF,
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
            if param != param_id:
                continue
            return rdata[4] | (rdata[5] << 8) | (rdata[6] << 16) | \
                (rdata[7] << 24)
        return None
    finally:
        sock.close()


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f87_acceptance")
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


def sets_of(evs, kind, param=None):
    return [e for e in evs if e[1] == kind and (param is None or e[2] == param)]


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
    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={IFACE}",
         "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"],
        STACK_LOG, env=env)

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

        # Criterion 1: layout, float value, exactly one frame per motor.
        all_ok = True
        for motor in range(1, 8):
            evs = rec.motor_events(motor, since=t_activate)
            sets = sets_of(evs, "set", PARAM_TORQUE_LIMIT)
            if len(sets) != 1:
                check(f"1 motor {motor} exactly one 0x700B frame", False,
                      f"count={len(sets)}")
                all_ok = False
                continue
            _, _, _, value, raw = sets[0]
            nm = struct.unpack("<f", struct.pack("<I", value))[0]
            expected = EXPECTED_NM[motor]
            bytes_ok = raw[0] == 0x0B and raw[1] == 0x70 and \
                raw[2] == 0x00 and raw[3] == 0x00
            value_ok = abs(nm - expected) / expected <= 0.01
            check(f"1 motor {motor} 0x700B layout", bytes_ok, raw[:4].hex())
            check(f"1 motor {motor} float = {expected} Nm +-1%", value_ok,
                  f"nm={nm}")
            if not (bytes_ok and value_ok):
                all_ok = False
        check("1 all 7 torque-limit frames correct", all_ok)

        # Criterion 2: ordering 0x7005 < 0x700B < 0x7028 < enable.
        order_all = True
        for motor in range(1, 8):
            evs = rec.motor_events(motor, since=t_activate)
            set05 = next((e for e in evs
                          if e[1] == "set" and e[2] == PARAM_RUN_MODE), None)
            set0b = next((e for e in evs
                          if e[1] == "set" and e[2] == PARAM_TORQUE_LIMIT),
                         None)
            set28 = next((e for e in evs
                          if e[1] == "set" and e[2] == PARAM_CAN_TIMEOUT), None)
            enable = next((e for e in evs if e[1] == "enable"), None)
            order_ok = all(x is not None for x in (set05, set0b, set28,
                                                   enable)) and \
                set05[0] < set0b[0] < set28[0] < enable[0]
            check(f"2 motor {motor} order 0x7005<0x700B<0x7028<enable",
                  order_ok)
            order_all = order_all and order_ok
        check("2 ordering correct for all motors", order_all)

        # Criterion 3: Type-17 readback bit-exact.
        readback_all = True
        for motor in range(1, 8):
            raw = readback_param(IFACE, motor, PARAM_TORQUE_LIMIT)
            want = struct.unpack(
                "<I", struct.pack("<f", EXPECTED_NM[motor]))[0]
            ok = raw == want
            check(f"3 motor {motor} readback bit-exact", ok,
                  f"raw={raw} want={want}")
            readback_all = readback_all and ok
        check("3 readback bit-exact for all motors", readback_all)

        # Criterion 4: choreography regression. Per motor two Type-4 frames:
        # the all-motors discovery reset (data[1]=0xC0) and the in-sequence
        # clear-fault (data[0]=1) — see F83.
        ch_all = True
        for motor in range(1, 8):
            evs = rec.motor_events(motor, since=t_activate)
            resets = sets_of(evs, "reset")
            clearf = [e for e in resets if e[3] == 0x01]
            plain = [e for e in resets if e[3] == 0x00 and e[4][1] == 0xC0]
            set05s = sets_of(evs, "set", PARAM_RUN_MODE)
            set28s = sets_of(evs, "set", PARAM_CAN_TIMEOUT)
            enables = sets_of(evs, "enable")
            counts_ok = len(plain) == 1 and len(clearf) == 1 and \
                len(set05s) == 1 and len(set28s) == 1 and len(enables) == 1
            data_ok = set28s and set28s[0][3] == EXPECTED_COUNTS
            order_ok = counts_ok and plain[0][0] < clearf[0][0] < \
                set05s[0][0] < set28s[0][0] < enables[0][0]
            check(f"4 motor {motor} choreography counts/order",
                  counts_ok and order_ok,
                  f"plain/clear/run/timeout/enable="
                  f"{len(plain)}/{len(clearf)}/{len(set05s)}/"
                  f"{len(set28s)}/{len(enables)}")
            check(f"4 motor {motor} 0x7028=4000", data_ok)
            ch_all = ch_all and counts_ok and data_ok and order_ok
        check("4 choreography regression all motors", ch_all)

        # Criterion 5: small quintic FJT.
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from f83_enable_choreography_acceptance import move_small
        err_arm, err_grip = move_small(node)
        check("5 quintic FJT error_code=0",
              err_arm == 0 and err_grip == 0, f"err={err_arm}/{err_grip}")

    except Exception:
        import traceback
        traceback.print_exc()
        fatal = True
    finally:
        if stack.proc.poll() is None:
            stack.hard_kill()
        sim.terminate()
        rec.stop()
        node.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F87 acceptance: {passed}/{total} ====", flush=True)
    if fatal:
        return 2
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
