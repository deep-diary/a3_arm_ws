#!/usr/bin/env python3
"""F97 acceptance: JTC goal-time abort & per-joint trajectory tolerances.

Fault injection (simulation only, real arm stays powered off):
  vcan_motor_sim.py --silence-file freezes one motor's feedback exactly as
  in F81; the hardware plugin enters protective freeze-hold, so the state can
  never converge to the goal. Before F97 (constraints.goal_time=0.0, no
  trajectory tolerance) the FollowJointTrajectory goal stayed pending
  forever. With F97 it must abort within trajectory_time + goal_time + margin.

  python3 scripts/a3_test/f97_jtc_tolerance_acceptance.py [DOMAIN_ID]

Acceptance (docs/edge/REQUIREMENTS.md F97):
  A. normal two-point trajectory -> SUCCESSFUL (regression)
  B. motor-4 feedback frozen from motion start: goal returns within
     2.0 s trajectory + 1.0 s goal_time + 1.0 s margin, error code is
     PATH_TOLERANCE_VIOLATED(-4) or GOAL_TOLERANCE_VIOLATED(-5)
  C. silence cleared: normal trajectory -> SUCCESSFUL again (no latch)

Exit code 0 = all checks passed.
"""

import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "97"
VCAN = os.environ.get("F97_VCAN", "vcan97")
SILENCE = "/tmp/f97_silence.json"
SIM_LOG = "/tmp/f97_sim.log"
STACK_LOG = "/tmp/f97_stack.log"

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
RESULTS = []

PATH_TOLERANCE_VIOLATED = -4
GOAL_TOLERANCE_VIOLATED = -5


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
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    env["PYTHONNOUSERSITE"] = "1"
    return env


def spin(node, t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(node, cli, request, timeout=15.0):
    fut = cli.call_async(request)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    raise RuntimeError(f"service timeout: {cli.srv_name}")


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f97_acceptance")
        self.js = None
        self.js_stamp = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")
        self.fjt = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory")

    def _on_js(self, msg):
        self.js = msg
        self.js_stamp = time.time()

    def arm_positions(self):
        return [self.js.position[self.js.name.index(n)] for n in ARM_JOINTS]

    def wait_fresh_js(self, timeout=40):
        self.js_stamp = None
        t0 = time.monotonic()
        while self.js_stamp is None and time.monotonic() - t0 < timeout:
            spin(self, 0.1)
        return self.js_stamp is not None

    def enable(self):
        assert self.enable_cli.wait_for_service(timeout_sec=20)
        spin(self, 2.0)
        resp = call(self, self.enable_cli, Trigger.Request(), timeout=40)
        if not resp.success:
            raise RuntimeError(f"enable rejected: {resp.message}")
        spin(self, 3.0)

    def run_trajectory(self, q_target, duration_s, result_wait_s):
        """Send a two-point trajectory from current q to q_target.

        Returns (accepted, elapsed_s, error_code); error_code None on timeout.
        """
        q0 = self.arm_positions()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        p0 = JointTrajectoryPoint(
            positions=[float(v) for v in q0],
            velocities=[0.0] * 6, accelerations=[0.0] * 6,
            time_from_start=RosDuration())
        p1 = JointTrajectoryPoint(
            positions=[float(v) for v in q_target],
            velocities=[0.0] * 6, accelerations=[0.0] * 6,
            time_from_start=RosDuration(
                sec=int(duration_s),
                nanosec=int((duration_s % 1) * 1e9)))
        goal.trajectory.points = [p0, p1]

        gh_fut = self.fjt.send_goal_async(goal)
        end = time.monotonic() + 8
        while not gh_fut.done() and time.monotonic() < end:
            spin(self, 0.05)
        if not gh_fut.done():
            return False, None, None
        handle = gh_fut.result()
        if not handle.accepted:
            return False, None, None

        res_fut = handle.get_result_async()
        t0 = time.monotonic()
        end = t0 + result_wait_s
        while not res_fut.done() and time.monotonic() < end:
            spin(self, 0.05)
        if not res_fut.done():
            return True, None, None
        elapsed = time.monotonic() - t0
        return True, elapsed, res_fut.result().result.error_code


def clamp_target(q):
    q = dict(q)
    q["L1_joint"] = max(-2.7, min(2.7, q["L1_joint"]))
    q["L2_joint"] = max(0.05, min(3.5, q["L2_joint"]))
    q["L3_joint"] = max(-3.9, min(-0.05, q["L3_joint"]))
    for name in ("L4_joint", "L5_joint", "L6_joint"):
        q[name] = max(-1.45, min(1.45, q[name]))
    return q


def ensure_vcan():
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "add", "dev", VCAN, "type", "vcan"],
        input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "set", VCAN, "up"],
        input=b"temppwd\n", capture_output=True)


def current_pose_dict(node):
    q = node.arm_positions()
    return dict(zip(ARM_JOINTS, q))


def shifted(node, deltas):
    q = current_pose_dict(node)
    for name, d in zip(ARM_JOINTS, deltas):
        q[name] += d
    return clamp_target(q)


def main():
    # Driver node itself must use the same domain/RMW as the spawned stack;
    # env vars passed to ProcessGroup do not affect this process.
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["PYTHONNOUSERSITE"] = "1"
    ensure_vcan()
    with open(SILENCE, "w") as f:
        f.write("{}")

    env = make_env()
    rclpy.init()
    node = HarnessNode()

    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", VCAN, "--silence-file", SILENCE],
        SIM_LOG, env=env)
    time.sleep(1.0)
    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={VCAN}",
         "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"],
        STACK_LOG, env=env)

    try:
        if not node.wait_fresh_js():
            raise RuntimeError("no /joint_states in 40 s")
        node.enable()
        assert node.fjt.wait_for_server(timeout_sec=10)

        # ---- A. regression: normal trajectory succeeds ----
        target_a = shifted(node, [0.10, 0.12, -0.12, 0.10, 0.10, 0.10])
        accepted, elapsed, code = node.run_trajectory(
            [target_a[n] for n in ARM_JOINTS], 3.0, result_wait_s=10.0)
        check("A normal trajectory accepted", accepted)
        check("A normal trajectory SUCCESSFUL",
              accepted and code == 0, f"elapsed={elapsed} code={code}")

        # ---- B. frozen feedback -> bounded abort, never hangs ----
        with open(SILENCE, "w") as f:
            json.dump({"motor": 4}, f)
        spin(node, 0.8)  # let the F81 stale latch / freeze-hold arm first

        target_b = shifted(node, [0.10, 0.12, -0.12, 0.10, 0.10, 0.10])
        accepted, elapsed, code = node.run_trajectory(
            [target_b[n] for n in ARM_JOINTS], 2.0, result_wait_s=4.5)
        check("B frozen-feedback trajectory accepted", accepted)
        # 2.0 s trajectory + 1.0 s goal_time + 1.0 s margin
        check("B frozen goal returns within 2+1+1 s (no infinite pending)",
              accepted and elapsed is not None,
              f"elapsed={elapsed}")
        check("B error code PATH/GOAL_TOLERANCE_VIOLATED",
              code in (PATH_TOLERANCE_VIOLATED, GOAL_TOLERANCE_VIOLATED),
              f"code={code}")

        # ---- C. recovery: silence cleared, success again ----
        with open(SILENCE, "w") as f:
            f.write("{}")
        spin(node, 1.5)
        target_c = shifted(node, [-0.05, -0.06, 0.06, -0.05, -0.05, -0.05])
        accepted, elapsed, code = node.run_trajectory(
            [target_c[n] for n in ARM_JOINTS], 3.0, result_wait_s=10.0)
        check("C post-recovery trajectory accepted", accepted)
        check("C post-recovery trajectory SUCCESSFUL",
              accepted and code == 0, f"elapsed={elapsed} code={code}")
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        stack.terminate()
        sim.terminate()
        if os.path.exists(SILENCE):
            os.remove(SILENCE)
        node.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F97 acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
