#!/usr/bin/env python3
"""F85 acceptance: velocity-adaptive Kd in free-drive (EFFORT CAN path).

Self-contained: launches vcan_motor_sim (external-force injection) and the
full product stack (a3_bringup.launch.py hardware:=can,
adaptive_kd_enabled:=true), runs criteria 1-5, tears the stack down,
restarts with adaptive_kd_enabled:=false, runs criterion 6. The arm stays
powered off; everything runs over vcan.

  python3 scripts/a3_test/f85_adaptive_kd_acceptance.py [DOMAIN_ID]

Acceptance (docs/edge/REQUIREMENTS.md F85):
  1. rest free-drive frames: kp~=0, pos=current, vel=0; L1-L3 kd in
     [0.12,0.16], L4-L6 at per-joint kd_max (0.10/0.05/0.05 +-0.02)
  2. motor 3 pushed to |v|>=1.5 rad/s: kd in [0.001,0.05]; unpushed joints
     hold their rest bands
  3. EMA: adjacent 200 Hz frame kd jumps <= 0.03
  4. after release, kd recovers to >= 0.10 within 3 s
  5. switch back: kp~=80, kd~=2, small quintic FJT error_code=0
  6. adaptive_kd_enabled:=false: kd constant 0.3+-0.05, even while pushed

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
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "85"
IFACE = os.environ.get("F85_CAN_IF", "vcan5")
STACK_LOG = "/tmp/f85_stack.log"
SIM_LOG = "/tmp/f85_sim.log"
STACK2_LOG = "/tmp/f85_stack_fixed.log"
EXT_FILE = os.environ.get("F85_EXT_FILE", "/tmp/f85_ext.json")
RESULTS = []

CAN_FORMAT = "=IB3x8s"
CMD_CONTROL = 0x01
CMD_FEEDBACK = 0x02
P_RANGE = 12.57
TORQUE_MAX = {i: (14.0 if i <= 3 else 6.0) for i in range(1, 8)}
SPEED_MAX = {i: (33.0 if i <= 3 else 50.0) for i in range(1, 8)}

DIRECTION = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0]
ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]

KD_REST_BAND = {1: (0.12, 0.16), 2: (0.12, 0.16), 3: (0.12, 0.16),
                4: (0.08, 0.12), 5: (0.03, 0.07), 6: (0.03, 0.07)}
KD_FIXED = 0.3
KP_POSITION = 80.0
KD_POSITION = 2.0
MAX_HISTORY = 6000


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


class ProcessGroup:
    def __init__(self, argv, log_path, env=None):
        self.log = open(log_path, "wb")
        self.proc = subprocess.Popen(
            argv, stdout=self.log, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True, env=env)

    def kill(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
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


def write_ext(payload):
    tmp = EXT_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(payload)
    os.replace(tmp, EXT_FILE)
    time.sleep(0.12)  # sim polls mtime every 50 ms


def u16_to_float(raw, lo, hi):
    return lo + (raw / 65535.0) * (hi - lo)


class FrameRecorder(threading.Thread):
    """Raw CAN_RAW socket; keeps a timestamped history of control frames."""

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
                u16_to_float((data[4] << 8) | data[5], 0.0, 500.0),
                u16_to_float((data[6] << 8) | data[7], 0.0, 5.0),
                u16_to_float((can_id >> 8) & 0xFFFF, -tmax, tmax),
            )
            with self.lock:
                lst = self.hist[motor_id]
                lst.append(rec)
                if len(lst) > MAX_HISTORY:
                    del lst[:-MAX_HISTORY]

    def frames(self, motor_id, t0=None, t1=None):
        with self.lock:
            return [r for r in self.hist[motor_id]
                    if (t0 is None or r[0] >= t0)
                    and (t1 is None or r[0] <= t1)]

    def latest(self, motor_id):
        with self.lock:
            return self.hist[motor_id][-1] if self.hist[motor_id] else None

    def mark(self):
        with self.lock:
            for lst in self.hist.values():
                lst.clear()

    def stop(self):
        self.stop_ev.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f85_acceptance")
        self.js = {"pos": {}, "vel": {}, "stamp": None, "n": 0}
        self.status = {"state": None}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        from a3_msgs.msg import ArmStatus
        self.create_subscription(ArmStatus, "/a3/arm_status",
                                 lambda m: self.status.__setitem__("state", m.state), 10)
        self.switch_cli = self.create_client(
            SwitchController, "/controller_manager/switch_controller")
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")

    def _on_js(self, msg):
        for i, name in enumerate(msg.name):
            self.js["pos"][name] = msg.position[i]
            if i < len(msg.velocity):
                self.js["vel"][name] = msg.velocity[i]
        self.js["stamp"] = time.monotonic()
        self.js["n"] += 1

    def switch(self, activate, deactivate):
        if not self.switch_cli.wait_for_service(timeout_sec=5.0):
            return False, "switch_controller service absent"
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = False
        req.timeout = rclpy.duration.Duration(seconds=3.0).to_msg()
        fut = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        res = fut.result()
        if res is None:
            return False, "switch timeout"
        return res.ok, "ok"


def wait_ready(node, timeout_s=30):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        spin(node, 0.1)
        if node.status["state"] == "READY":
            return True
    return False


def wait_js(node, timeout_s=25):
    t0 = time.monotonic()
    while node.js["n"] == 0 and time.monotonic() - t0 < timeout_s:
        spin(node, 0.1)
    return node.js["n"] > 0


def enable_to_ready(node):
    assert node.enable_cli.wait_for_service(timeout_sec=20), "enable service absent"
    resp = call(node, node.enable_cli, Trigger.Request(), timeout=30)
    if not resp.success:
        raise RuntimeError(f"enable rejected: {resp.message}")
    if not wait_ready(node):
        raise RuntimeError(f"not READY: {node.status['state']}")


def rest_frame_checks(node, rec, label, fixed=False):
    """Criterion 1 / fixed-mode rest: kp, vel, pos, kd bands."""
    spin(node, 0.8)
    max_kp = max_vel = max_pos_err = 0.0
    kd_vals = {}
    for motor in range(1, 7):
        f = rec.latest(motor)
        if f is None:
            check(f"{label} motor {motor} frame present", False)
            continue
        _, pos, vel, kp, kd, _ = f
        kd_vals[motor] = kd
        max_kp = max(max_kp, kp)
        max_vel = max(max_vel, abs(vel))
        joint_pos = node.js["pos"].get(ARM_JOINTS[motor - 1])
        if joint_pos is not None:
            max_pos_err = max(max_pos_err,
                              abs(pos - DIRECTION[motor - 1] * joint_pos))

    check(f"{label} kp~=0", max_kp <= 0.05, f"max kp={max_kp:.3f}")
    check(f"{label} vel=0", max_vel <= 0.01, f"max |vel|={max_vel:.4f}")
    check(f"{label} pos=current measured", max_pos_err <= 0.02,
          f"max err={max_pos_err:.4f}")

    if not fixed:
        for motor, (lo, hi) in KD_REST_BAND.items():
            v = kd_vals.get(motor, -1.0)
            check(f"{label} L{motor} kd in [{lo},{hi}]", lo <= v <= hi,
                  f"kd={v:.3f}")
    else:
        all_fixed = all(abs(kd_vals.get(m, -1.0) - KD_FIXED) <= 0.05
                        for m in range(1, 7))
        check(f"{label} kd constant {KD_FIXED}+-0.05", all_fixed,
              f"{ {m: round(v, 3) for m, v in kd_vals.items()} }")


def push_phase_adaptive(node, rec):
    """Criteria 2-4: push motor 3, high-speed kd / EMA / recovery."""
    t_push = time.monotonic()
    write_ext('{"motor": 3, "torque": 0.6}')

    t_fast = None
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 4.0:
            spin(node, 0.05)
            v = abs(node.js["vel"].get("L3_joint", 0.0))
            if v >= 1.5:
                t_fast = time.monotonic()
                break
        check("2a motor 3 reaches |v|>=1.5 rad/s", t_fast is not None,
              f"v={v:.2f}" if t_fast is None else "")

        if t_fast is None:
            return

        # Hold the push briefly to collect high-speed frames.
        spin(node, 0.4)
    finally:
        t_clear = time.monotonic()
        write_ext("{}")

    fast = rec.frames(3, t0=t_fast, t1=t_clear)
    pushed_kd = [f[4] for f in fast]
    check("2b pushed motor 3 high-speed kd in [0.001,0.05]",
          len(pushed_kd) >= 5 and all(0.001 <= k <= 0.05 for k in pushed_kd),
          f"n={len(pushed_kd)} min={min(pushed_kd, default=-1):.3f} "
          f"max={max(pushed_kd, default=-1):.3f}")

    # Unpushed joints hold rest bands (latest frame during the fast window).
    for motor, (lo, hi) in ((1, KD_REST_BAND[1]), (2, KD_REST_BAND[2]),
                            (4, KD_REST_BAND[4]), (5, KD_REST_BAND[5]),
                            (6, KD_REST_BAND[6])):
        windowed = rec.frames(motor, t0=t_push, t1=t_clear)
        kd = windowed[-1][4] if windowed else -1.0
        check(f"2c unpushed L{motor} holds rest kd band", lo <= kd <= hi,
              f"kd={kd:.3f}")

    # Criterion 3: EMA adjacent-frame jumps.
    max_jump = max(abs(b[4] - a[4]) for a, b in zip(fast, fast[1:])) \
        if len(fast) >= 2 else 9.9
    check("3 EMA adjacent frame dkd <= 0.03", max_jump <= 0.03,
          f"max dkd={max_jump:.4f} n={len(fast)}")

    # Criterion 4: recovery to kd >= 0.10 within 3 s.
    recovered = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        spin(node, 0.05)
        f = rec.latest(3)
        if f is not None and f[0] >= t_clear and f[4] >= 0.10:
            recovered = True
            break
    dt = time.monotonic() - t0
    check("4 kd recovers to >=0.10 within 3 s", recovered,
          f"in {dt:.2f} s")


def start_stack(env, adaptive, log_path):
    args = ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
            "hardware:=can", f"can_interface:={IFACE}",
            f"adaptive_kd_enabled:={'true' if adaptive else 'false'}",
            "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"]
    return ProcessGroup(args, log_path, env=env)


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    ensure_vcan(IFACE)
    write_ext("{}")
    env = make_env(DOMAIN)

    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", IFACE, "--ext-file", EXT_FILE],
        SIM_LOG, env=env)
    time.sleep(0.8)
    stack = start_stack(env, True, STACK_LOG)

    rclpy.init()
    node = HarnessNode()
    rec = FrameRecorder(IFACE)
    try:
        rec.start()
    except OSError as e:
        print(f"vcan capture socket failed: {e}")
        return 1

    fatal = False
    try:
        if not wait_js(node):
            raise RuntimeError("no /joint_states in 25 s")
        spin(node, 0.5)
        enable_to_ready(node)

        ok, msg = node.switch(activate=["zero_torque_controller"],
                              deactivate=["arm_controller"])
        if not ok:
            raise RuntimeError(f"switch to free-drive failed: {msg}")

        rest_frame_checks(node, rec, "rest-adaptive")
        push_phase_adaptive(node, rec)

        # Criterion 5: switch back to position mode + small FJT.
        ok, msg = node.switch(activate=["arm_controller"],
                              deactivate=["zero_torque_controller"])
        check("5a switch back to arm_controller", ok, msg)
        if ok:
            spin(node, 0.6)
            kp_ok = all(abs((rec.latest(m) or [None] * 4)[3] - KP_POSITION) <= 0.5
                        for m in range(1, 7))
            kd_ok = all(abs((rec.latest(m) or [None] * 5)[4] - KD_POSITION) <= 0.05
                        for m in range(1, 7))
            check("5b position frames kp~=80", kp_ok)
            check("5c position frames kd~=2", kd_ok)

            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from f83_enable_choreography_acceptance import move_small
            err_arm, err_grip = move_small(node)
            check("5d quintic FJT error_code=0",
                  err_arm == 0 and err_grip == 0, f"err={err_arm}/{err_grip}")

    except Exception:
        import traceback
        traceback.print_exc()
        fatal = True
    finally:
        write_ext("{}")
        stack.kill()
        spin(node, 2.0)

    # ---- criterion 6: fixed-damping fallback restart ----
    if not fatal:
        stack2 = start_stack(env, False, STACK2_LOG)
        try:
            node.js["n"] = 0
            if not wait_js(node):
                raise RuntimeError("fixed stack: no /joint_states")
            spin(node, 0.5)
            enable_to_ready(node)
            ok, msg = node.switch(activate=["zero_torque_controller"],
                                  deactivate=["arm_controller"])
            check("6a switch to free-drive (fixed)", ok, msg)
            if ok:
                rest_frame_checks(node, rec, "rest-fixed", fixed=True)

                # Fixed damping must not change even at speed.
                write_ext('{"motor": 3, "torque": 0.6}')
                try:
                    moving = False
                    t0 = time.monotonic()
                    while time.monotonic() - t0 < 4.0:
                        spin(node, 0.05)
                        if abs(node.js["vel"].get("L3_joint", 0.0)) >= 1.0:
                            moving = True
                            break
                    spin(node, 0.5)
                    frames = rec.frames(3, t0=t0)
                    kd_vals = [f[4] for f in frames]
                    check("6b motor 3 reaches |v|>=1.0 (fixed push)", moving)
                    check("6c kd stays 0.3+-0.05 while pushed",
                          len(kd_vals) >= 5
                          and all(abs(k - KD_FIXED) <= 0.05 for k in kd_vals),
                          f"n={len(kd_vals)} "
                          f"range=[{min(kd_vals, default=0):.3f},"
                          f"{max(kd_vals, default=0):.3f}]")
                finally:
                    write_ext("{}")
                spin(node, 1.0)
        except Exception:
            import traceback
            traceback.print_exc()
            fatal = True
        finally:
            stack2.kill()

    sim.kill()
    rec.stop()
    node.destroy_node()
    rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F85 acceptance: {passed}/{total} ====", flush=True)
    if fatal:
        return 2
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
