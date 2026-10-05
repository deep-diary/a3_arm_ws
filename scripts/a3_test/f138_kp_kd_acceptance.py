#!/usr/bin/env python3
"""F138 acceptance: runtime per-joint kp/kd on the unified ros2_control stack.

Verifies the F138 P0 change — that /a3_hardware_health now accepts
kp_<joint>/kd_<joint> set_parameters and the A3MITHardwareInterface write()
emits the updated gains on the CAN control frame. Runs self-contained on vcan
(no real arm): vcan_motor_sim + a3_bringup.launch.py hardware:=can.

  python3 scripts/a3_test/f138_kp_kd_acceptance.py [DOMAIN_ID] [VCAN_IFACE]

Acceptance (docs/edge/REQUIREMENTS.md F138):
  1. per-joint default = 80/2 for all L1-L6 (xacro dead-param activation is a
     no-op: wrist 100/4 was normalized to 80/2)
  2. set kp_L1_joint=60 -> L1 control frame kp decodes ~60 (other joints 80)
  3. set kd_L1_joint=3 -> L1 kd decodes ~3
  4. restore nominal -> L1 kp/kd back to ~80/~2

Exit code 0 = all checks passed.

Requires sudo to create the vcan interface (modprobe vcan + ip link add).
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
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "138"
IFACE = sys.argv[2] if len(sys.argv) > 2 else "vcan138"
STACK_LOG = "/tmp/f138_stack.log"
SIM_LOG = "/tmp/f138_sim.log"
HEALTH_NODE = "/a3_hardware_health"

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
KP_NOM = 80.0
KD_NOM = 2.0
KP_TOL = 3.0
KD_TOL = 0.3
KP_TEST = 60.0
KD_TEST = 3.0

# 开发机 sudo 密码（与 f85/f87/f90 等既有 vcan 验收脚本一致，仅本机 vcan 建接口用）。
SUDO_PW = b"temppwd\n"

CAN_FORMAT = "=IB3x8s"
CMD_CONTROL = 0x01
P_RANGE = 12.57
TORQUE_MAX = {i: (14.0 if i <= 3 else 6.0) for i in range(1, 8)}
SPEED_MAX = {i: (33.0 if i <= 3 else 50.0) for i in range(1, 8)}

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def u16_to_float(raw, lo, hi):
    return lo + (raw / 65535.0) * (hi - lo)


class FrameRecorder(threading.Thread):
    def __init__(self, interface):
        super().__init__(daemon=True)
        self.interface = interface
        self.stop_ev = threading.Event()
        self.lock = threading.Lock()
        self.hist = {i: [] for i in range(1, 8)}
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
            if cmd_type != CMD_CONTROL or not (1 <= motor_id <= 7):
                continue
            vmax = SPEED_MAX[motor_id]
            tmax = TORQUE_MAX[motor_id]
            rec = (
                time.monotonic(),
                u16_to_float((data[0] << 8) | data[1], -P_RANGE, P_RANGE),
                u16_to_float((data[2] << 8) | data[3], -vmax, vmax),
                u16_to_float((data[4] << 8) | data[5], 0.0, 500.0),  # kp
                u16_to_float((data[6] << 8) | data[7], 0.0, 5.0),    # kd
                u16_to_float((can_id >> 8) & 0xFFFF, -tmax, tmax),
            )
            with self.lock:
                self.hist[motor_id].append(rec)
                if len(self.hist[motor_id]) > 2000:
                    del self.hist[motor_id][:-2000]

    def latest(self, motor_id):
        with self.lock:
            return self.hist[motor_id][-1] if self.hist[motor_id] else None

    def stop(self):
        self.stop_ev.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass


class Harness(Node):
    def __init__(self):
        super().__init__("f138_acceptance")
        self.status = {"state": None}
        from a3_msgs.msg import ArmStatus
        self.create_subscription(
            ArmStatus, "/a3/arm_status",
            lambda m: self.status.__setitem__("state", m.state), 10)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")
        self.set_cli = self.create_client(
            SetParameters, HEALTH_NODE + "/set_parameters")


def spin_until(fut, node, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    return fut.done()


def set_gain(node, joint, kp, kd):
    node.set_cli.wait_for_service(timeout_sec=5.0)
    req = SetParameters.Request()
    req.parameters = [
        Parameter(name=f"kp_{joint}",
                  value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                       double_value=float(kp))),
        Parameter(name=f"kd_{joint}",
                  value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                       double_value=float(kd))),
    ]
    fut = node.set_cli.call_async(req)
    if not spin_until(fut, node, 5.0):
        return False
    res = fut.result()
    return bool(res and all(r.successful for r in res.results))


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    env = dict(os.environ)
    env["PYTHONNOUSERSITE"] = "1"

    # vcan + sim + stack。
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=SUDO_PW, capture_output=True)
    subprocess.run(["sudo", "-S", "ip", "link", "add", "dev", IFACE,
                    "type", "vcan"], input=SUDO_PW, capture_output=True)
    subprocess.run(["sudo", "-S", "ip", "link", "set", IFACE, "up"],
                   input=SUDO_PW, capture_output=True)

    sim = subprocess.Popen(
        [sys.executable, f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", IFACE],
        stdout=open(SIM_LOG, "wb"), stderr=subprocess.STDOUT,
        start_new_session=True, env=env)
    stack = subprocess.Popen(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={IFACE}"],
        stdout=open(STACK_LOG, "wb"), stderr=subprocess.STDOUT,
        start_new_session=True, env=env)

    rec = FrameRecorder(IFACE)
    rec.start()

    rclpy.init()
    node = Harness()
    try:
        # enable -> READY.
        assert node.enable_cli.wait_for_service(timeout_sec=30), "enable svc absent"
        fut = node.enable_cli.call_async(Trigger.Request())
        assert spin_until(fut, node, 40.0), "enable timeout"
        assert fut.result().success, f"enable rejected: {fut.result().message}"

        end = time.monotonic() + 30.0
        while time.monotonic() < end and node.status["state"] != "READY":
            rclpy.spin_once(node, timeout_sec=0.1)
        assert node.status["state"] == "READY", f"not READY: {node.status['state']}"

        # 1) default gains all 80/2.
        time.sleep(1.0)
        ok_default = True
        for m in range(1, 7):
            f = rec.latest(m)
            if f is None:
                ok_default = False
                break
            if abs(f[3] - KP_NOM) > KP_TOL or abs(f[4] - KD_NOM) > KD_TOL:
                ok_default = False
        check("default kp/kd 80/2 for L1-L6", ok_default)

        # 2) kp_L1_joint=60.
        assert set_gain(node, "L1_joint", KP_TEST, KD_NOM), "set kp failed"
        time.sleep(1.0)
        f = rec.latest(1)
        check("kp_L1_joint=60 on wire",
              f is not None and abs(f[3] - KP_TEST) <= KP_TOL,
              f"kp={f[3]:.1f}" if f else "no frame")

        # 3) kd_L1_joint=3.
        assert set_gain(node, "L1_joint", KP_TEST, KD_TEST), "set kd failed"
        time.sleep(1.0)
        f = rec.latest(1)
        check("kd_L1_joint=3 on wire",
              f is not None and abs(f[4] - KD_TEST) <= KD_TOL,
              f"kd={f[4]:.2f}" if f else "no frame")

        # 4) restore nominal.
        assert set_gain(node, "L1_joint", KP_NOM, KD_NOM), "restore failed"
        time.sleep(1.0)
        f = rec.latest(1)
        check("restore 80/2 on wire",
              f is not None and abs(f[3] - KP_NOM) <= KP_TOL
              and abs(f[4] - KD_NOM) <= KD_TOL,
              f"kp={f[3]:.1f} kd={f[4]:.2f}" if f else "no frame")

    finally:
        rec.stop()
        node.destroy_node()
        rclpy.shutdown()
        for p in (stack, sim):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except (ProcessLookupError, OSError):
                pass
        print(f"\n==== F138 RESULT: "
              f"{sum(1 for _, ok in RESULTS if ok)} PASS / "
              f"{sum(1 for _, ok in RESULTS if not ok)} FAIL", flush=True)
        sys.exit(0 if all(ok for _, ok in RESULTS) else 1)


if __name__ == "__main__":
    main()
