#!/usr/bin/env python3
"""F81 acceptance: per-motor feedback-staleness watchdog.

Fault injection: scripts/a3_test/vcan_motor_sim.py --silence-file lets one
motor stop sending type-2 feedback while still executing control frames,
emulating a dead feedback TX channel.

This harness is self-contained: it launches the vcan sim and the full CAN
stack (a3_bringup.launch.py hardware:=can can_interface:=vcan0) itself, runs
the acceptance checks, and tears everything down.

  python3 scripts/a3_test/f81_feedback_stale_acceptance.py [DOMAIN_ID]

Acceptance (docs/edge/REQUIREMENTS.md F81):
  1. silence motor 4 -> plugin ERROR log naming motor 4 within 0.2+0.5 s
  2. while silenced, commanded motion moves the other joints but motor 4's
     CAN command position stays frozen (write() freeze-hold)
  3. silence cleared -> feedback resumes, latch clears; disable/enable cycle
     moves all 7 joints; disable tears the stack down cleanly
  4. a motor missing feedback from startup is blocked by on_activate
     (arm_controller never activates; "re-anchored 6/7" in log)

Exit code 0 = all checks passed.
"""

import math
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from collections import deque

import rclpy
import rclpy.action
from rclpy.node import Node

from a3_msgs.msg import ArmStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "65"
IFACE = os.environ.get("F81_CAN_IF", "vcan0")
SILENCE = os.environ.get("F81_SILENCE_FILE", "/tmp/f81_silence.json")
SIM_LOG = "/tmp/f81_sim_harness.log"
# Attach mode: run checks against an already-booted full stack (started
# manually in another terminal); harness then does not launch or kill it.
ATTACH = os.environ.get("F81_ATTACH", "") == "1"
STACK_LOG = os.environ.get(
    "F81_STACK_LOG", "/tmp/f81_stack.log" if ATTACH else "/tmp/f81_phaseA.log")
# Phase B: minimal controller-only stack on a second bus/domain.
IFACE_B = os.environ.get("F81_CAN_IF_B", "vcan2")
SILENCE_B = os.environ.get("F81_SILENCE_FILE_B", "/tmp/f81_silence_b.json")
MINIMAL_LAUNCH = os.environ.get(
    "F81_MINIMAL_LAUNCH", "/tmp/f81_minimal.launch.py")
JOINTS = [f"L{i}_joint" for i in range(1, 8)]

CAN_EFF_FLAG = 0x80000000
CAN_FORMAT = "=IB3x8s"
CMD_CONTROL = 0x01
CMD_FEEDBACK = 0x02
P_RANGE = 12.57
TORQUE_MAX = {i: (14.0 if i <= 3 else 6.0) for i in range(1, 8)}
SPEED_MAX = {i: (33.0 if i <= 3 else 50.0) for i in range(1, 8)}

FB_TIMEOUT_S = 0.2
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def u16_to_float(raw, lo, hi):
    return lo + (raw / 65535.0) * (hi - lo)


class ProcessGroup:
    def __init__(self, argv, log_path, env=None):
        self.log = open(log_path, "wb")
        self.proc = subprocess.Popen(
            argv, stdout=self.log, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True, env=env,
        )

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


class CanSniffer(threading.Thread):
    def __init__(self, interface="vcan0"):
        super().__init__(daemon=True)
        self.interface = interface
        self.stop_ev = threading.Event()
        self.lock = threading.Lock()
        self.cmd = {}   # motor -> deque[(t, pos)]
        self.fb_t = {}  # motor -> latest feedback t
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
            motor_id = ((can_id >> 8) & 0xFF
                        if cmd_type == CMD_FEEDBACK else can_id & 0xFF)
            if not (1 <= motor_id <= 7):
                continue
            now = time.monotonic()
            with self.lock:
                if cmd_type == CMD_CONTROL:
                    pos = u16_to_float(
                        (data[0] << 8) | data[1], -P_RANGE, P_RANGE)
                    self.cmd.setdefault(motor_id, deque(maxlen=2000)).append(
                        (now, pos))
                elif cmd_type == CMD_FEEDBACK:
                    self.fb_t[motor_id] = now

    def fb_age(self, motor):
        with self.lock:
            t = self.fb_t.get(motor)
        return None if t is None else time.monotonic() - t

    def cmd_span(self, motor, since):
        with self.lock:
            vals = [p for t, p in self.cmd.get(motor, ()) if t >= since]
        return (max(vals) - min(vals)) if vals else 0.0, len(vals)

    def stop(self):
        self.stop_ev.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass


class Recorder(Node):
    def __init__(self):
        super().__init__("f81_acceptance")
        self.samples = deque(maxlen=2000)
        self.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg):
        snap = {n: p for n, p in zip(msg.name, msg.position)}
        if all(j in snap for j in JOINTS):
            self.samples.append((time.monotonic(), snap))

    def current(self):
        if len(self.samples) < 3:
            return None
        tail = [s for _, s in list(self.samples)[-9:]]
        return {j: sorted(s[j] for s in tail)[len(tail) // 2]
                for j in JOINTS}


def spin(node, t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(node, cli, request, timeout=8.0):
    fut = cli.call_async(request)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    raise RuntimeError(f"service timeout: {cli.srv_name}")


def wait_jsb(node, list_cli, deadline_s=60):
    t0 = time.monotonic()
    while time.monotonic() - t0 < deadline_s:
        spin(node, 0.5)
        states = {c.name: c.state for c in call(
            node, list_cli, ListControllers.Request(), timeout=5.0).controller}
        if states.get("joint_state_broadcaster") == "active":
            return states
    return None


def quintic_goal(joint_names, q0, q1, duration_s=3.0):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(joint_names)
    n = 31
    for k in range(n + 1):
        s = k / n
        s = s * s * s * (10 * s * s - 15 * s + 6)
        pt = JointTrajectoryPoint()
        pt.positions = [a + s * (b - a) for a, b in zip(q0, q1)]
        pt.time_from_start.sec = int(duration_s * k / n)
        pt.time_from_start.nanosec = int(
            (duration_s * k / n % 1.0) * 1e9)
        goal.trajectory.points.append(pt)
    return goal


def move(node, q0, q1, duration_s=3.0, result_wait_s=None):
    """Send a quintic 7-joint move, split across the two standard JTCs.

    L1-L6 -> /arm_controller, L7 -> /gripper_controller (their claimed joint
    sets differ). Returns (arm_error, gripper_error); (None, None) on infra
    failure; ("pending", "pending") if results have not arrived by result_wait_s
    (used while the hardware is in freeze-hold: JTC holds the goal past its end
    because open-loop still gates success on actual state vs goal tolerance).
    """
    arm_names = JOINTS[:6]
    grip_names = JOINTS[6:]
    ac_arm = rclpy.action.ActionClient(
        node, FollowJointTrajectory,
        "/arm_controller/follow_joint_trajectory")
    ac_grip = rclpy.action.ActionClient(
        node, FollowJointTrajectory,
        "/gripper_controller/follow_joint_trajectory")
    if (not ac_arm.wait_for_server(timeout_sec=8)
            or not ac_grip.wait_for_server(timeout_sec=8)):
        return None, None

    gh1 = ac_arm.send_goal_async(quintic_goal(
        arm_names, [q0[j] for j in arm_names], [q1[j] for j in arm_names],
        duration_s))
    gh2 = ac_grip.send_goal_async(quintic_goal(
        grip_names, [q0[j] for j in grip_names], [q1[j] for j in grip_names],
        duration_s))
    end = time.monotonic() + 10
    while (not gh1.done() or not gh2.done()) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not gh1.done() or not gh2.done():
        return None, None
    if not (gh1.result().accepted and gh2.result().accepted):
        return None, None

    r1 = gh1.result().get_result_async()
    r2 = gh2.result().get_result_async()
    if result_wait_s is None:
        result_wait_s = duration_s + 8
    end = time.monotonic() + result_wait_s
    while (not r1.done() or not r2.done()) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not r1.done() or not r2.done():
        return "pending", "pending"
    return r1.result().result.error_code, r2.result().result.error_code


def clamp_target(q):
    q = dict(q)
    q["L1_joint"] = max(-2.7, min(2.7, q["L1_joint"]))
    q["L2_joint"] = max(0.05, min(3.5, q["L2_joint"]))
    q["L3_joint"] = max(-3.9, min(-0.05, q["L3_joint"]))
    for name in ("L4_joint", "L5_joint", "L6_joint", "L7_joint"):
        q[name] = max(-1.45, min(1.45, q[name]))
    return q


def list_remote(domain):
    """list_controllers via a fresh CLI process on another ROS domain."""
    try:
        proc = subprocess.run(
            ["ros2", "service", "call",
             "/controller_manager/list_controllers",
             "controller_manager_msgs/srv/ListControllers"],
            capture_output=True, text=True, timeout=12,
            env=make_env(domain))
    except subprocess.TimeoutExpired:
        return None
    return proc.stdout if proc.returncode == 0 else None


def make_env(domain):
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["PYTHONNOUSERSITE"] = "1"
    return env


def start_stack(domain, log_path):
    env = make_env(domain)
    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", IFACE, "--silence-file", SILENCE],
        SIM_LOG, env=env)
    time.sleep(0.8)
    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={IFACE}",
         "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"],
        log_path, env=env)
    return sim, stack


def main():
    rclpy.init()
    node = Recorder()

    list_cli = node.create_client(
        ListControllers, "/controller_manager/list_controllers")
    enable_cli = node.create_client(Trigger, "/a3/arm/enable")
    disable_cli = node.create_client(Trigger, "/a3/arm/disable")

    sim = stack = None
    if not ATTACH:
        if os.path.exists(SILENCE):
            os.remove(SILENCE)
        sim, stack = start_stack(DOMAIN, STACK_LOG)

    sniffer = CanSniffer(IFACE)
    sniffer.start()
    fatal = False
    try:
        if not list_cli.wait_for_service(timeout_sec=30):
            raise RuntimeError("controller_manager absent in 30s")
        if wait_jsb(node, list_cli, deadline_s=90) is None:
            raise RuntimeError("JSB not active in 90s")

        call(node, enable_cli, Trigger.Request(), timeout=15)
        t0 = time.monotonic()
        state = {}
        fsm_box = {"v": ""}
        node.create_subscription(
            ArmStatus, "/a3/arm_status",
            lambda m: fsm_box.update(v=m.state), 10)
        while time.monotonic() - t0 < 15:
            spin(node, 0.1)
            state = {c.name: c.state for c in call(
                node, list_cli, ListControllers.Request(),
                timeout=5.0).controller}
            if (fsm_box["v"] == "READY"
                    and state.get("arm_controller") == "active"
                    and state.get("gripper_controller") == "active"):
                break
        check("1a enable -> READY + arm/gripper controllers active",
              fsm_box["v"] == "READY"
              and state.get("arm_controller") == "active"
              and state.get("gripper_controller") == "active",
              f"fsm={fsm_box['v']} states={state}")

        spin(node, 0.5)
        q0 = node.current()
        age4 = sniffer.fb_age(4)
        check("1b 7 feedback channels healthy (motor 4 fresh)",
              q0 is not None and age4 is not None and age4 < 0.1,
              f"age4={age4}")

        # ---- check 1: silence motor 4 -> stale ERROR within 0.2+0.5 s ----
        with open(SILENCE, "w") as f:
            f.write('{"motor": 4}')
        t_silence = time.monotonic()
        detected = None
        ages_grow = 0
        prev_age = 0.0
        while time.monotonic() - t_silence < FB_TIMEOUT_S + 0.5:
            spin(node, 0.05)
            age = sniffer.fb_age(4)
            if age is not None and age > prev_age + 0.1:
                ages_grow += 1
                prev_age = age
            with open(STACK_LOG, errors="ignore") as lf:
                if "feedback stale: motor=4" in lf.read():
                    detected = time.monotonic() - t_silence
                    break
        check("1 motor-4 silence -> stale ERROR within 0.7s",
              detected is not None and ages_grow >= 1,
              f"detected={detected} age_grows={ages_grow}")

        # ---- check 2: whole-arm protective freeze-hold during silence ----
        q1 = node.current()
        delta = [0.12, 0.15, -0.18, 0.25, 0.12, 0.12, 0.15]
        target = dict(q1)
        for i, d in enumerate(delta):
            target[JOINTS[i]] += d
        target = clamp_target(target)

        t_cmd = time.monotonic()
        # Trajectory (2 s) ends well inside the 4 s wait while the arm is still
        # blind: JTC must hold the goals un-finished — open-loop still gates
        # success on actual state converging inside goal tolerance, and the
        # freeze-hold guarantees it cannot.
        err_arm, err_grip = move(node, q1, target,
                                 duration_s=2.0, result_wait_s=4.0)
        spin(node, 0.3)
        spans = {}
        for motor in range(1, 8):
            spans[motor] = sniffer.cmd_span(motor, t_cmd)
        max_span = max(s for s, _ in spans.values())
        min_frames = min(n for _, n in spans.values())
        q_after = node.current()
        max_drift = max(abs(q_after[j] - q1[j]) for j in JOINTS)
        check("2 protective freeze: 7 hold, zero displacement, goals stay pending",
              err_arm == "pending" and err_grip == "pending"
              and min_frames > 100 and max_span < 0.02 and max_drift < 0.03,
              f"err={err_arm}/{err_grip} max_span={max_span:.4f} "
              f"min_frames={min_frames} max_drift={max_drift:.3f}")

        # ---- check 3: recovery + disable/enable + motion + clean disable ----
        with open(SILENCE, "w") as f:
            f.write("{}")
        t0 = time.monotonic()
        recovered = False
        while time.monotonic() - t0 < 2.0:
            spin(node, 0.05)
            age = sniffer.fb_age(4)
            if age is not None and age < 0.1:
                recovered = True
                break
        spin(node, 0.3)
        check("3a feedback resumes, latch clears", recovered,
              f"age4={sniffer.fb_age(4)}")

        call(node, disable_cli, Trigger.Request(), timeout=15)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20:
            spin(node, 0.1)
            st = {c.name: c.state for c in call(
                node, list_cli, ListControllers.Request(),
                timeout=5.0).controller}
            if (fsm_box["v"] == "DISABLED"
                    and st.get("arm_controller") == "inactive"):
                break
        check("3b disable -> DISABLED",
              fsm_box["v"] == "DISABLED"
              and st.get("arm_controller") == "inactive",
              f"fsm={fsm_box['v']} arm={st.get('arm_controller')}")

        call(node, enable_cli, Trigger.Request(), timeout=15)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15:
            spin(node, 0.1)
            if fsm_box["v"] == "READY":
                break
        spin(node, 0.5)
        q2 = node.current()
        q_back = dict(q2)
        for i, d in enumerate(delta):
            q_back[JOINTS[i]] = q2[JOINTS[i]] - d
        q_back = clamp_target(q_back)
        t_cmd = time.monotonic()
        err_arm, err_grip = move(node, q2, q_back)
        spin(node, 0.3)
        span4, n4 = sniffer.cmd_span(4, t_cmd)
        span7, n7 = sniffer.cmd_span(7, t_cmd)
        check("3c re-enable -> all 7 joints (incl. motors 4 and 7) move",
              err_arm == 0 and err_grip == 0
              and n4 > 10 and span4 > 0.05 and span7 > 0.05,
              f"err={err_arm}/{err_grip} span4={span4:.3f} ({n4}) "
              f"span7={span7:.3f} ({n7})")

        call(node, disable_cli, Trigger.Request(), timeout=15)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20:
            spin(node, 0.1)
            if fsm_box["v"] == "DISABLED":
                break
        check("3d disable clean teardown", fsm_box["v"] == "DISABLED",
              f"fsm={fsm_box['v']}")
    except Exception:
        import traceback
        traceback.print_exc()
        fatal = True
    finally:
        sniffer.stop()
        if not ATTACH and stack is not None:
            stack.kill()
            sim.kill()

    # ---- check 4: motor missing from startup blocked by on_activate ----
    if not fatal:
        with open(SILENCE_B, "w") as f:
            f.write('{"motor": 4}')
        domain_b = str(int(DOMAIN) + 3)
        sim2 = ProcessGroup(
            ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
             "--interface", IFACE_B, "--silence-file", SILENCE_B],
            "/tmp/f81_sim_b.log", env=make_env(domain_b))
        time.sleep(0.8)
        env_b = make_env(domain_b)
        env_b["F81_CAN_IF"] = IFACE_B
        log_b = "/tmp/f81_phaseB.log"
        stack2 = ProcessGroup(
            ["ros2", "launch", MINIMAL_LAUNCH], log_b, env=env_b)
        ever_active = False
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < 22:
                spin(node, 0.5)
                out = list_remote(domain_b)
                if out and "name='arm_controller'" in out:
                    # parse just the arm_controller state
                    head = out.split("name='arm_controller'")[1].split(")")[0]
                    if "state='active'" in head:
                        ever_active = True
                        break
                with open(log_b, errors="ignore") as lf:
                    text = lf.read()
                if (not ever_active
                        and "no enable frames sent" in text
                        and ("Failed to 'activate' hardware" in text
                             or "process has died" in text
                             or "Could not contact service" in text)):
                    break
            with open(log_b, errors="ignore") as lf:
                text = lf.read()
            evidence = "no enable frames sent" in text
            spawner_failed = (
                "Failed to 'activate' hardware" in text
                or "process has died" in text
                or "Could not contact service" in text)
            check("4 missing-at-startup motor blocks activation, no motors enabled",
                  not ever_active and evidence and spawner_failed,
                  f"ever_active={ever_active} log_no_enable={evidence} "
                  f"spawner_failed={spawner_failed}")
        finally:
            stack2.kill()
            sim2.kill()
            if os.path.exists(SILENCE_B):
                os.remove(SILENCE_B)

    node.destroy_node()
    rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F81 acceptance: {passed}/{total} ====", flush=True)
    if fatal:
        return 2
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
