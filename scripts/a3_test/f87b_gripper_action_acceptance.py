#!/usr/bin/env python3
"""F87（第二步）acceptance: L7 on the standard GripperActionController.

Self-contained vcan harness: launches vcan_motor_sim (per-motor dynamics
overrides enabled) plus the product stack a3_bringup.launch.py
hardware:=can, runs the five requirement criteria, hard-kills the stack.
The real arm stays powered off.

  python3 scripts/a3_test/f87b_gripper_action_acceptance.py [DOMAIN_ID]

Acceptance (docs/edge/REQUIREMENTS.md F87 第二步):
  1. gripper_controller loads as effort_controllers/GripperActionController;
     /gripper_controller/gripper_cmd (control_msgs/GripperCommand) is
     connectable; no /gripper_controller/follow_joint_trajectory
  2. position goals (open/close) converge within goal_tolerance with
     reached_goal=true; travel matches calibration (open 0.0, close ~1.79)
  3. contact goals: after sim injects a hard stop, raw Type-1 frames decode
     to steady effort within +-5% of the goal max_effort (two values);
     stall detection fires per stall_velocity_threshold/stall_timeout and
     the force is maintained
  4. double-layer limit: firmware 0x700B is still 6 Nm; a goal with an
     excessive max_effort cannot exceed the firmware/plugin clamp
  5. default launch omits the Python a3_gripper_controller node; nothing in
     the stack targets /a3/motor/set_param or /mit_gains_cmd; FSM
     goto/park/disable L7 segments go through the standard action and the
     full chain regresses green

Exit code 0 = all checks passed.
"""

import json
import os
import re
import signal
import socket
import statistics
import struct
import subprocess
import sys
import threading
import time

import rclpy
import rclpy.action
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from a3_msgs.msg import ArmStatus
from a3_msgs.srv import GotoNamedPose

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "87"
IFACE = os.environ.get("F87B_CAN_IF", "vcan7")
STATE_FILE = os.environ.get("F87B_STATE_FILE", "/tmp/f87_timeout_state.json")
DYN_FILE = os.environ.get("F87B_DYN_FILE", "/tmp/f87_dynamics.json")
STACK_LOG = "/tmp/f87b_stack.log"
SIM_LOG = "/tmp/f87b_sim.log"
RESULTS = []

CAN_FORMAT = "=IB3x8s"
CAN_EFF_FLAG = 0x80000000
CMD_CONTROL = 0x01
CMD_GET_PARAM = 0x11
MASTER_ID = 0xFD
PARAM_TORQUE_LIMIT = 0x700B
TORQUE_MAX = 6.0  # L7 = EL05
ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 1.79
CONTACT_AT = 0.8


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
        ["sudo", "-S", "ip", "link", "add", "dev", interface, "type", "vcan"],
        input=b"temppwd\n", capture_output=True)
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


def write_dynamics(cfg):
    with open(DYN_FILE, "w") as f:
        json.dump(cfg, f)
    # sim polls mtime at 50 ms cadence
    time.sleep(0.15)


class TorqueRecorder(threading.Thread):
    """Timestamped decode of motor-7 Type-1 torque_ff fields."""

    def __init__(self, interface):
        super().__init__(daemon=True)
        self.interface = interface
        self.stop_ev = threading.Event()
        self.lock = threading.Lock()
        self.samples = []
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
            can_id, _dlc, _data = struct.unpack(CAN_FORMAT, frame)
            can_id &= 0x1FFFFFFF
            if (can_id >> 24) & 0x1F != CMD_CONTROL:
                continue
            if can_id & 0xFF != 7:
                continue
            raw_t = (can_id >> 8) & 0xFFFF
            torque = -TORQUE_MAX + (raw_t / 65535) * (2.0 * TORQUE_MAX)
            with self.lock:
                self.samples.append((time.monotonic(), torque))

    def window(self, seconds):
        cutoff = time.monotonic() - seconds
        with self.lock:
            return [v for t, v in self.samples if t >= cutoff]

    def stop(self):
        self.stop_ev.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass


def readback_param(interface, motor_id, param_id):
    """Send a raw Type-17 request, wait for the reply, return raw u32."""
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((interface,))
        sock.settimeout(0.5)
        can_id = (CMD_GET_PARAM << 24) | (MASTER_ID << 8) | motor_id
        data = [param_id & 0xFF, (param_id >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
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
        super().__init__("f87b_acceptance")
        self.status = {"state": None, "l7": None}
        self.create_subscription(JointState, "/joint_states",
                                 self._on_js, 10)
        self.create_subscription(
            ArmStatus, "/a3/arm_status",
            lambda m: self.status.__setitem__("state", m.state), 10)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")
        self.disable_cli = self.create_client(Trigger, "/a3/arm/disable")
        self.goto_cli = self.create_client(
            GotoNamedPose, "/a3/arm/goto_named_pose")

    def _on_js(self, msg):
        if "L7_joint" in msg.name:
            self.status["l7"] = float(
                msg.position[msg.name.index("L7_joint")])

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


def list_controllers(env):
    proc = subprocess.run(
        ["ros2", "control", "list_controllers",
         "--controller-manager", "/controller_manager"],
        env=env, capture_output=True, text=True, timeout=20)
    found = {}
    for line in proc.stdout.splitlines():
        clean = re.sub(r"\x1b\[[0-9;]*m", "", line).split()
        if len(clean) >= 3:
            found[clean[0]] = (clean[1], clean[2])
    return found


def gripper_goal(node, position, max_effort, timeout=25.0):
    ac = rclpy.action.ActionClient(
        node, GripperCommand, "/gripper_controller/gripper_cmd")
    if not ac.wait_for_server(timeout_sec=10):
        raise RuntimeError("gripper_cmd action absent")
    goal = GripperCommand.Goal()
    goal.command.position = float(position)
    goal.command.max_effort = float(max_effort)
    gh_fut = ac.send_goal_async(goal)
    end = time.monotonic() + timeout
    while not gh_fut.done() and time.monotonic() < end:
        spin(node, 0.05)
    if not gh_fut.done():
        raise RuntimeError("gripper goal handle timeout")
    handle = gh_fut.result()
    if not handle.accepted:
        raise RuntimeError("gripper goal rejected")
    res_fut = handle.get_result_async()
    end = time.monotonic() + timeout
    while not res_fut.done() and time.monotonic() < end:
        spin(node, 0.05)
    if not res_fut.done():
        raise RuntimeError("gripper result timeout")
    return res_fut.result().result


def quintic_goal(delta, duration_s=3.0):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(ARM_JOINTS)
    n = 31
    for k in range(n + 1):
        s = k / n
        s = s * s * s * (10 * s * s - 15 * s + 6)
        pt = JointTrajectoryPoint()
        pt.positions = [s * d for d in delta]
        t = duration_s * k / n
        pt.time_from_start.sec = int(t)
        pt.time_from_start.nanosec = int((t % 1.0) * 1e9)
        goal.trajectory.points.append(pt)
    return goal


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    ensure_vcan(IFACE)
    for path in (STATE_FILE, DYN_FILE):
        try:
            os.remove(path)
        except OSError:
            pass
    env = make_env(DOMAIN)

    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", IFACE, "--state-file", STATE_FILE,
         "--dynamics-file", DYN_FILE],
        SIM_LOG, env=env)
    time.sleep(0.8)
    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={IFACE}",
         "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"],
        STACK_LOG, env=env)

    rclpy.init()
    node = HarnessNode()
    rec = TorqueRecorder(IFACE)
    rec.start()

    fatal = False
    try:
        if not wait_js(node):
            raise RuntimeError("no /joint_states in 25 s")
        spin(node, 1.0)

        # ---- Criterion 1 (pre-enable): controller type/inactive state ----
        ctrls = list_controllers(env)
        g = ctrls.get("gripper_controller")
        check("1 gripper_controller = effort GAC, inactive before enable",
              g == ("effort_controllers/GripperActionController", "inactive"),
              str(g))
        a = ctrls.get("arm_controller")
        check("1 arm_controller = JTC, inactive before enable",
              a == ("joint_trajectory_controller/JointTrajectoryController",
                    "inactive"), str(a))

        # ---- Criterion 5 (graph evidence): no Python gripper node / deps ----
        names_ns = node.get_node_names_and_namespaces()
        node_names = {n for n, _ns in names_ns}
        check("5 Python gripper_controller_node absent",
              "gripper_controller_node" not in node_names)
        services = dict(node.get_service_names_and_types())
        check("5 /a3/gripper/command service absent",
              "/a3/gripper/command" not in services)
        check("5 /a3/gripper/set_config service absent",
              "/a3/gripper/set_config" not in services)
        check("5 /a3/motor/set_param service absent",
              "/a3/motor/set_param" not in services)
        no_gains = not node.get_publishers_info_by_topic(
            "/mit_gains_cmd") and not node.get_subscriptions_info_by_topic(
            "/mit_gains_cmd")
        check("5 /mit_gains_cmd has no endpoints", no_gains)

        # ---- Enable (standard activation choreography) ----
        t_enable = time.monotonic()
        node.enable_to_ready()
        spin(node, 1.0)
        ctrls = list_controllers(env)
        check("1 controllers active after enable",
              ctrls.get("gripper_controller", ("", ""))[1] == "active"
              and ctrls.get("arm_controller", ("", ""))[1] == "active",
              str(ctrls))

        ac_g = rclpy.action.ActionClient(
            node, GripperCommand, "/gripper_controller/gripper_cmd")
        check("1 /gripper_controller/gripper_cmd connectable",
              ac_g.wait_for_server(timeout_sec=10))
        ac_old = rclpy.action.ActionClient(
            node, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory")
        check("1 /gripper_controller/follow_joint_trajectory absent",
              not ac_old.wait_for_server(timeout_sec=2.0))

        # ---- Criterion 2: free-space position goals ----
        write_dynamics({"7": {"gravity_nm": 0.0}})
        r = gripper_goal(node, GRIPPER_CLOSE, 1.0)
        close_ok = r.reached_goal and abs(r.position - GRIPPER_CLOSE) <= 0.03
        check("2 close goal reaches ~1.79, reached_goal=true",
              close_ok, f"pos={r.position:.3f} stalled={r.stalled}")
        r = gripper_goal(node, GRIPPER_OPEN, 1.0)
        open_ok = r.reached_goal and abs(r.position - GRIPPER_OPEN) <= 0.03
        check("2 open goal reaches 0.0, reached_goal=true",
              open_ok, f"pos={r.position:.3f} stalled={r.stalled}")

        # ---- Criterion 3: contact, steady effort, stall ----
        write_dynamics(
            {"7": {"gravity_nm": 0.0, "stop_at": CONTACT_AT}})
        # First back off to open (negative motion not pinned).
        r = gripper_goal(node, GRIPPER_OPEN, 1.0)
        check("3 back-off to open before contact", r.reached_goal,
              f"pos={r.position:.3f}")

        contact_all = True
        for want_eff in (0.5, 1.0):
            r = gripper_goal(node, GRIPPER_CLOSE, want_eff)
            # GAC stall contract (gripper_action_controller_impl.hpp:181):
            # stalled=true, reached_goal=false; allow_stalling -> succeeded.
            stall_ok = r.stalled and not r.reached_goal
            pos_ok = abs(r.position - CONTACT_AT) <= 0.03
            check(f"3 close @ {want_eff} Nm: stall succeeded, pinned at 0.8",
                  stall_ok and pos_ok,
                  f"stalled={r.stalled} reached={r.reached_goal} "
                  f"pos={r.position:.3f}")
            # Command outlives the finished goal; sample steady frames.
            time.sleep(0.6)
            vals = rec.window(0.5)
            steady = statistics.median(vals) if vals else float("nan")
            eff_ok = len(vals) >= 40 and abs(steady - want_eff) / want_eff \
                <= 0.05 and steady > 0
            check(f"3 steady Type-1 effort = {want_eff} Nm +-5%",
                  eff_ok, f"n={len(vals)} median={steady:.3f}")
            contact_all = contact_all and stall_ok and pos_ok and eff_ok
        check("3 contact force control at both effort levels", contact_all)

        # ---- Criterion 4: double-layer limit ----
        raw = readback_param(IFACE, 7, PARAM_TORQUE_LIMIT)
        nm = struct.unpack("<f", struct.pack("<I", raw or 0))[0]
        check("4 firmware 0x700B readback = 6.0 Nm",
              raw is not None and abs(nm - 6.0) <= 0.01, f"nm={nm}")
        r = gripper_goal(node, GRIPPER_CLOSE, 20.0)
        time.sleep(0.6)
        vals = rec.window(0.5)
        steady = statistics.median(vals) if vals else float("nan")
        clamp_ok = len(vals) >= 40 and abs(steady - 6.0) / 6.0 <= 0.05 \
            and steady > 0
        check("4 max_effort=20 clamps at firmware/plugin 6 Nm", clamp_ok,
              f"n={len(vals)} median={steady:.3f}")

        # ---- Criterion 5: FSM regression (mixed mode, goto, park/disable) ----
        write_dynamics({"7": {"gravity_nm": 0.0}})
        r = gripper_goal(node, GRIPPER_OPEN, 1.0)
        assert r.reached_goal

        arm_fut = None
        ac_arm = rclpy.action.ActionClient(
            node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        assert ac_arm.wait_for_server(timeout_sec=10)
        arm_handle_fut = ac_arm.send_goal_async(quintic_goal([0.10] * 6))
        end = time.monotonic() + 5
        while not arm_handle_fut.done() and time.monotonic() < end:
            spin(node, 0.05)
        arm_handle = arm_handle_fut.result()
        rg = gripper_goal(node, 0.5, 1.0)  # concurrently with arm move
        ar_fut = arm_handle.get_result_async()
        end = time.monotonic() + 15
        while not ar_fut.done() and time.monotonic() < end:
            spin(node, 0.05)
        arm_err = ar_fut.result().result.error_code
        check("5 mixed-mode: arm FJT error_code=0 + L7 goal reached",
              arm_err == 0 and rg.reached_goal,
              f"arm_err={arm_err} grip_pos={rg.position:.3f}")

        resp = call(node, node.goto_cli,
                    GotoNamedPose.Request(pose_name="ready"), timeout=45)
        check("5 FSM goto ready (move_group + L7 action)", resp.success,
              resp.message)

        resp = call(node, node.disable_cli, Trigger.Request(), timeout=45)
        disabled = resp.success and node.status["state"] == "DISABLED"
        check("5 FSM disable parks L7 via action -> DISABLED", disabled,
              f"{resp.message} state={node.status['state']}")

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
    print(f"\n==== F87b acceptance: {passed}/{total} ====", flush=True)
    if fatal:
        return 2
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
