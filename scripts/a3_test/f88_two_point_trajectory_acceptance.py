#!/usr/bin/env python3
"""F88 acceptance: two-point standard trajectories replace hand-rolled dense
linear interpolation.

Two phases, both simulation-only (real arm stays powered off):

  Phase 1 — mock hardware (mock_components/GenericSystem), FSM backend
  fjt_action:
    a) direct two-point FJT goal to JTC; /arm_controller/controller_state
       sampled at 200 Hz. error_code=0, end convergence, quintic bell
       profile: endpoint v ~= 0, v/(D/T) <= 0.50 at alpha=0.10 (theory
       0.243), peak ratio >= 1.5 near alpha=0.5 (theory 1.875).
    b) FSM regression: set_joint_positions jog preempt x3, goto fallback,
       move_to fallback; dispatch boundary log shows points=2.
    c) safe-park -> disable -> DISABLED.
    d) source-tree residue greps for removed params/helpers.

  Phase 2 — topic backend over vcan (vcan_motor_sim + deprecated legacy
  stack can_bridge motor_protocol_node, 200 Hz position interpolation):
    set_joint_positions jog + goto; arm follows; dispatch log shows
    backend=topic points=2.

  python3 scripts/a3_test/f88_two_point_trajectory_acceptance.py [DOMAIN]

Exit code 0 = all checks passed.
"""

import os
import re
import signal
import subprocess
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from a3_msgs.srv import (
    GotoNamedPose,
    MoveToJointPositions,
    SetJointPositions,
)

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "88"
VCAN = os.environ.get("F88_VCAN", "vcan88")
MOCK_LOG = "/tmp/f88_mock_stack.log"
LEGACY_LOG = "/tmp/f88_legacy_stack.log"
SIM_LOG = "/tmp/f88_vcan_sim.log"
RESULTS = []

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
ALL_JOINTS = [f"L{i}_joint" for i in range(1, 8)]
RESIDUE = ["goto_waypoints", "move_to_points_hz",
           "move_to_max_points", "_traj_point_count"]


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
            self.hard_kill()

    def hard_kill(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc.wait(timeout=10)


def make_env():
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = DOMAIN
    env["PYTHONNOUSERSITE"] = "1"
    return env


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


def wait_js(node, timeout_s=30):
    # Reuse HarnessNode's ctor dual-QoS subscriptions (product JSB offers
    # RELIABLE; legacy motor_protocol BEST_EFFORT) instead of adding more
    # readers — extra subscriptions on one node worsen SHM port pressure.
    t0 = time.monotonic()
    while node.js_stamp is None and time.monotonic() - t0 < timeout_s:
        spin(node, 0.1)
    return node.js_stamp is not None


def clear_injection_files():
    # vcan_motor_sim watches these continuously; leftovers from earlier
    # F-feature runs (dynamics overrides, pushes, faults, silences) would
    # otherwise leak into this acceptance.
    for path in ("/tmp/f87_dynamics.json", "/tmp/f73_ext.json",
                 "/tmp/f81_silence.json", "/tmp/f84_health.json"):
        with open(path, "w") as f:
            f.write("{}")


def ensure_vcan(interface):
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "add", "dev", interface, "type", "vcan"],
        input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "set", interface, "up"],
        input=b"temppwd\n", capture_output=True)


def force_fallback(env, tries=10):
    # ros2 param set's "Setting parameter successful" stdout is not a proof:
    # the CLI can report while the node never applied it. Verify by GET.
    for _ in range(tries):
        subprocess.run(
            ["ros2", "param", "set", "/a3_arm_controller",
             "goto_use_moveit", "false"],
            env=env, capture_output=True, text=True, timeout=15)
        g = subprocess.run(
            ["ros2", "param", "get", "/a3_arm_controller",
             "goto_use_moveit"],
            env=env, capture_output=True, text=True, timeout=15)
        if "boolean value is: false" in (g.stdout + g.stderr).lower():
            return True
        time.sleep(0.5)
    return False


_URDF_LIMITS = None


def urdf_limits():
    global _URDF_LIMITS
    if _URDF_LIMITS is not None:
        return _URDF_LIMITS
    import xml.etree.ElementTree as ET
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(
        get_package_share_directory("a3_description"), "urdf", "el_a3.urdf")
    root = ET.parse(path).getroot()
    _URDF_LIMITS = {}
    for j in root.iter("joint"):
        lim = j.find("limit")
        if lim is not None:
            _URDF_LIMITS[j.get("name")] = (
                float(lim.get("lower", "-inf")),
                float(lim.get("upper", "inf")))
    return _URDF_LIMITS


def clamp_targets(values):
    # Mirror the FSM's per-joint URDF clamp; settle comparisons must use the
    # clamped target (e.g. L3_joint upper=0: a positive jog is legal but ends
    # at 0, not at the requested value).
    lim = urdf_limits()
    out = []
    for n, v in zip(ALL_JOINTS, values):
        lo, hi = lim.get(n, (float("-inf"), float("inf")))
        out.append(min(max(float(v), lo), hi))
    return out


class HarnessNode(Node):
    def __init__(self, name="f88_acceptance"):
        super().__init__(name)
        self.state = None
        self.js = None
        self.js_stamp = None
        self.js_delay = None
        self.ctrl_samples = []
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.create_subscription(
            JointState, "/joint_states", self._on_js,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            JointState, "/joint_states", self._on_js,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            __import__("a3_msgs.msg", fromlist=["ArmStatus"]).ArmStatus,
            "/a3/arm_status",
            lambda m: self.__setattr__("state", m.state), 10)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/arm_controller/controller_state", self._on_ctrl, 200)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")
        self.disable_cli = self.create_client(Trigger, "/a3/arm/disable")
        self.setpos_cli = self.create_client(
            SetJointPositions, "/a3/arm/set_joint_positions")
        self.goto_cli = self.create_client(
            GotoNamedPose, "/a3/arm/goto_named_pose")
        self.moveto_cli = self.create_client(
            MoveToJointPositions, "/a3/arm/move_to")

    def _on_js(self, msg):
        now = time.time()
        self.js = msg
        self.js_stamp = now
        hdr = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.js_delay = now - hdr if hdr > 1e9 else None

    def _on_ctrl(self, msg):
        self.ctrl_samples.append(
            (time.time(), list(msg.reference.positions),
             list(msg.reference.velocities)))

    def arm_positions(self):
        return [self.js.position[self.js.name.index(n)] for n in ARM_JOINTS]

    def all_positions(self):
        return [self.js.position[self.js.name.index(n)] for n in ALL_JOINTS]

    def enable_to_ready(self, timeout=40):
        assert self.enable_cli.wait_for_service(timeout_sec=20), \
            "enable service absent"
        # wait_for_service only proves discovery saw the server; the
        # request/reply endpoints can still be matching, and a request sent
        # in that window is silently lost. Settle, then tolerate one retry.
        spin(self, 2.0)
        try:
            resp = call(self, self.enable_cli, Trigger.Request(),
                        timeout=timeout)
        except RuntimeError:
            spin(self, 2.0)
            resp = call(self, self.enable_cli, Trigger.Request(),
                        timeout=timeout)
        if not resp.success and "no /joint_states" in resp.message:
            # Late-joining BEST_EFFORT subscriber on FastDDS SHM can receive
            # nothing for seconds while an earlier subscriber already flows.
            # Wait for feedback, then retry the enable.
            for _ in range(8):
                spin(self, 1.0)
                resp = call(self, self.enable_cli, Trigger.Request(),
                            timeout=timeout)
                if resp.success or "no /joint_states" not in resp.message:
                    break
        if not resp.success:
            raise RuntimeError(f"enable rejected: {resp.message}")
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            spin(self, 0.1)
            if self.state == "READY":
                return
        raise RuntimeError(f"not READY: {self.state}")

    def wait_ready(self, timeout=15.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            spin(self, 0.05)
            if self.state == "READY":
                return True
        return False

    def wait_settle(self, target6, pos_tol=0.03, timeout=12.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            spin(self, 0.05)
            if self.js_stamp and time.time() - self.js_stamp < 1.0:
                cur = self.arm_positions()
                if max(abs(a - b) for a, b in zip(cur, target6)) <= pos_tol:
                    return True
        return False


def two_point_goal(q0, q1, duration_s):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(ARM_JOINTS)
    n = len(q0)
    p0 = JointTrajectoryPoint()
    p0.positions = [float(v) for v in q0]
    p0.velocities = [0.0] * n
    p0.accelerations = [0.0] * n
    goal.trajectory.points.append(p0)
    p1 = JointTrajectoryPoint()
    p1.positions = [float(v) for v in q1]
    p1.velocities = [0.0] * n
    p1.accelerations = [0.0] * n
    p1.time_from_start.sec = int(duration_s)
    goal.trajectory.points.append(p1)
    for jn in ARM_JOINTS:
        goal.path_tolerance.append(
            JointTolerance(name=jn, position=0.2, velocity=5.0))
        goal.goal_tolerance.append(
            JointTolerance(name=jn, position=0.02, velocity=0.05))
    goal.goal_time_tolerance.nanosec = 500_000_000
    return goal


def run_fjt(node, goal, result_timeout=12.0):
    ac = ActionClient(
        node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=10):
        raise RuntimeError("arm FJT action absent")
    gh_fut = ac.send_goal_async(goal)
    end = time.monotonic() + 5
    while not gh_fut.done() and time.monotonic() < end:
        spin(node, 0.05)
    handle = gh_fut.result()
    if not handle.accepted:
        raise RuntimeError("FJT goal rejected")
    res_fut = handle.get_result_async()
    end = time.monotonic() + result_timeout
    while not res_fut.done() and time.monotonic() < end:
        spin(node, 0.05)
    return res_fut.result().result


def phase1(env, node):
    print("\n==== Phase 1: mock hardware, fjt_action backend ====",
          flush=True)
    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=mock", "use_mqtt:=false", "use_teleop:=false",
         "use_rviz:=false"],
        MOCK_LOG, env=env)
    try:
        if not wait_js(node, timeout_s=35):
            raise RuntimeError("no /joint_states in 35 s")

        spin(node, 1.0)
        node.enable_to_ready()
        spin(node, 1.0)

        # ---- Criterion 1+2: direct two-point goal, quintic profile ----
        q0 = node.arm_positions()
        delta = 0.05
        q1 = [v + delta for v in q0]
        T = 3.0
        spin(node, 0.5)
        node.ctrl_samples.clear()
        result = run_fjt(node, two_point_goal(q0, q1, T))
        check("1 two-point FJT accepted, error_code=0",
              result.error_code == 0, f"code={result.error_code}")
        converge = node.wait_settle(q1, pos_tol=0.02, timeout=6.0)
        check("1 end point converges within goal tolerance", converge)

        samples = node.ctrl_samples
        ts = [s[0] for s in samples]
        speeds = [max(abs(x) for x in s[2]) for s in samples]
        # Time origin calibrated from the velocity peak (quintic peak is
        # exactly alpha=0.5): position-threshold departure would bias the
        # axis to alpha~0.13 because the reference schedules ahead.
        i_peak = speeds.index(max(speeds))
        t_origin = ts[i_peak] - T * 0.5
        const_v = delta / T
        peak_ratio = speeds[i_peak] / const_v

        def near_alpha(alpha, half=0.012):
            cand = [i for i in range(len(samples))
                    if abs(ts[i] - t_origin - T * alpha) <= half]
            return cand[len(cand) // 2] if cand else None

        def ratio_at(alpha):
            i = near_alpha(alpha)
            return (speeds[i] / const_v if i is not None else 9.9,
                    i is not None)

        in_seg = [i for i in range(len(samples))
                  if 0.05 <= (ts[i] - t_origin) / T <= 0.95
                  and speeds[i] > 0.002]
        check("2 controller_state samples dense through segment",
              len(in_seg) > 250, f"n_in_seg={len(in_seg)}")

        r01, found01 = ratio_at(0.1)
        check("2 alpha=0.10 velocity ratio <= 0.50 (quintic theory 0.243)",
              found01 and r01 <= 0.50, f"ratio={r01:.3f}")

        r005, found005 = ratio_at(0.05)
        check("2 alpha=0.05 velocity ratio < 0.5 (theory 0.068; linear would be 1.0)",
              found005 and r005 < 0.5, f"ratio={r005:.3f}")

        check("2 peak ratio >= 1.5 at alpha=0.5 (theory 1.875)",
              peak_ratio >= 1.5, f"peak={peak_ratio:.3f}")

        v_start, found_rs = ratio_at(0.0)
        v_end, found_re = ratio_at(1.0)
        check("2 endpoint velocities ~= 0 (<=0.02 rad/s)",
              found_rs and found_re and v_start <= 0.02 and v_end <= 0.02,
              f"start={v_start:.4f} end={v_end:.4f}")

        # ---- Criterion 3: FSM paths, points=2 ----
        # Force the local two-point fallback and verify the switch actually
        # took (a blind ros2 param set raced the node once and goto silently
        # went through move_group instead).
        forced = force_fallback(env)
        check("3 goto_use_moveit=false forced for fallback test", forced)

        base = node.all_positions()
        jog_targets = [
            [v + d for v in base]
            for d in (0.03, 0.06, 0.03)]
        jog_ok = True
        for tgt in jog_targets:
            req = SetJointPositions.Request(
                positions=[float(v) for v in tgt], duration=0.5)
            resp = call(node, node.setpos_cli, req, timeout=8)
            jog_ok = jog_ok and resp.success
            time.sleep(0.25)  # preempt in flight
        check("3 set_joint_positions jog preempt x3 all succeed", jog_ok)
        jog_end = clamp_targets(jog_targets[-1])
        check("3 final jog converges, FSM back to READY",
              node.wait_settle(jog_end[:6], timeout=8)
              and node.wait_ready(timeout=8))

        req = GotoNamedPose.Request(pose_name="home")
        resp = call(node, node.goto_cli, req, timeout=30)
        check("3 goto fallback succeeds (two-point fallback msg)",
              resp.success and "two-point fallback" in resp.message,
              resp.message)
        check("3 goto finishes, FSM back to READY",
              node.wait_ready(timeout=20))

        here = node.all_positions()
        req = MoveToJointPositions.Request(
            positions=[float(v) for v in here], duration_s=3.0)
        resp = call(node, node.moveto_cli, req, timeout=30)
        check("3 move_to fallback path succeeds (two-point, 2 pts msg)",
              resp.success and "two-point fallback" in resp.message,
              resp.message)
        node.wait_ready(timeout=10)

        with open(MOCK_LOG, "r", errors="replace") as f:
            logtxt = f.read()
        dispatch2 = len(re.findall(
            r"\[F88\] dispatch trajectory backend=fjt_action points=2",
            logtxt))
        check("3 dispatch boundary log: fjt_action points=2 >= 5 times",
              dispatch2 >= 5, f"count={dispatch2}")

        # ---- Criterion 4: safe-park -> disable ----
        # Jog off home first: disable from within disable_home_tol_rad
        # (0.15) short-circuits to a direct reset, so move 0.20 rad to force
        # the safe-park two-point path.
        away = [v + 0.20 for v in node.all_positions()]
        away_exp = clamp_targets(away)
        req = SetJointPositions.Request(
            positions=[float(v) for v in away], duration=1.0)
        resp = call(node, node.setpos_cli, req, timeout=10)
        settled_away = node.wait_settle(away_exp[:6], timeout=10)
        residual = 0.0 if settled_away else max(
            abs(a - b)
            for a, b in zip(node.arm_positions(), away_exp[:6]))
        moved_off = resp.success and settled_away
        check("4 moved off home before disable", moved_off,
              f"resp={resp.success} residual={residual:.3f}")

        resp = call(node, node.disable_cli, Trigger.Request(), timeout=40)
        disabled = (resp.success and node.state == "DISABLED"
                    and "safe" in resp.message.lower())
        check("4 safe-park -> disable -> DISABLED", disabled,
              f"{resp.message} state={node.state}")

        # ---- Criterion 5: residue greps ----
        grep = subprocess.run(
            ["grep", "-rn", "--include=*.py", "--include=*.yaml",
             "-E", "|".join(RESIDUE),
             os.path.join(WS, "src"),
             os.path.join(WS, "scripts")],
            capture_output=True, text=True)
        hits = [ln for ln in grep.stdout.splitlines()
                if "f88_two_point_trajectory_acceptance.py" not in ln]
        check("5 no removed param/helper residue in src/scripts",
              not hits, "\n     ".join(hits[:5]))

    except Exception:
        import traceback
        traceback.print_exc()
        return False, stack
    return True, stack


def phase2(env, node):
    print("\n==== Phase 2: vcan topic backend (motor_protocol 200 Hz) ====",
          flush=True)
    clear_injection_files()
    ensure_vcan(VCAN)
    # Legacy motor_map.yaml hardcodes arm_bus=can1; edge_legacy_stack does
    # not expose can1_name. Rewrite the map onto can0 (renamed to VCAN by
    # can0_name) instead of modifying the deprecated product launch.
    src_map = os.path.join(
        WS, "install/a3_can_bridge/share/a3_can_bridge/config/motor_map.yaml")
    with open(src_map, errors="replace") as f:
        map_txt = f.read()
    map_txt = re.sub(r"arm_bus:\s*\S+", "arm_bus: can0", map_txt, count=1)
    tmp_map = "/tmp/f88_motor_map.yaml"
    with open(tmp_map, "w") as f:
        f.write(map_txt)

    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", VCAN],
        SIM_LOG, env=env)
    time.sleep(0.8)
    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "edge_legacy_stack.launch.py",
         f"can0_name:={VCAN}", f"motor_map_file:={tmp_map}",
         "use_moveit:=false", "use_mqtt:=false",
         "use_teleop:=false", "use_target_ghost:=false",
         "use_power_sequence:=false", "use_gripper:=false",
         "use_servo:=false"],
        LEGACY_LOG, env=env)
    try:
        if not wait_js(node, timeout_s=35):
            raise RuntimeError("legacy stack: no /joint_states in 35 s")
        spin(node, 1.0)
        node.enable_to_ready()
        spin(node, 1.0)
        # use_moveit:=false only excludes move_group from the launch; the FSM
        # param still defaults to true, so goto would wait on an absent server.
        forced = force_fallback(env)
        check("topic goto_use_moveit=false forced", forced)

        base = node.all_positions()
        tgt = [v + 0.10 for v in base]
        tgt_exp = clamp_targets(tgt)
        req = SetJointPositions.Request(
            positions=[float(v) for v in tgt], duration=1.0)
        resp = call(node, node.setpos_cli, req, timeout=10)
        check("topic set_joint_positions succeeds", resp.success,
              resp.message)
        moved = node.wait_settle(tgt_exp[:6], pos_tol=0.03, timeout=10)
        check("topic two-point trajectory interpolated by motor_protocol",
              moved)
        # Positional settle on motor feedback precedes the FSM TRAJ->READY
        # transition by ~0.3-0.6 s; goto issued earlier is rejected as
        # state=TRAJ. Wait for the FSM, not just the positions.
        if not node.wait_ready(timeout=8):
            raise RuntimeError(f"FSM not READY after jog: {node.state}")

        req = GotoNamedPose.Request(pose_name="home")
        resp = call(node, node.goto_cli, req, timeout=30)
        check("topic goto fallback succeeds", resp.success, resp.message)
        node.wait_ready(timeout=10)

        with open(LEGACY_LOG, "r", errors="replace") as f:
            logtxt = f.read()
        n = len(re.findall(
            r"\[F88\] dispatch trajectory backend=topic points=2", logtxt))
        check("topic dispatch boundary log points=2 >= 2 times", n >= 2,
              f"count={n}")

    except Exception:
        import traceback
        traceback.print_exc()
        return False, stack, sim, node
    return True, stack, sim, node


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    env = make_env()
    rclpy.init()
    clear_injection_files()

    phase1_node = HarnessNode()
    ok1, stack1 = phase1(env, phase1_node)
    if stack1.proc.poll() is None:
        stack1.hard_kill()
    spin(phase1_node, 1.0)

    ok2 = False
    stack2 = sim2 = node2 = None
    if ok1:
        node2 = HarnessNode("f88_acceptance_p2")
        ok2, stack2, sim2, node2 = phase2(env, node2)
        if stack2 and stack2.proc.poll() is None:
            stack2.hard_kill()
        if sim2:
            sim2.terminate()

    if node2:
        node2.destroy_node()
    phase1_node.destroy_node()
    rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F88 acceptance: {passed}/{total} ====", flush=True)
    return 0 if ok1 and ok2 and passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
