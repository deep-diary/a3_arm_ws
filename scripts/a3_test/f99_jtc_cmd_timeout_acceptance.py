#!/usr/bin/env python3
"""F99 acceptance: JTC cmd_timeout hard-caps stale topic-interface commands.

F97's constraints.goal_time only finalizes *action* goals. Trajectories
sent on the ~/joint_trajectory topic have no action lifecycle: after the
last point the controller keeps sampling the stale tail forever (and
allow_nonzero_velocity_at_trajectory_end=true means nonzero end velocity
is accepted). cmd_timeout (ros2_controllers JTC standard parameter) is
counted from the last point; with no new trajectory it forces hold at the
current position. It must be strictly greater than goal_time or it is
silently dropped at configure time.

  python3 scripts/a3_test/f99_jtc_cmd_timeout_acceptance.py [DOMAIN_ID]

Acceptance (docs/edge/REQUIREMENTS.md F99):
  A. arm_controller.cmd_timeout resolves to 2.0; no "must be higher than
     goal_time" warning in the stack log.
  B. Normal two-point action trajectory still SUCCESSFUL (no regression).
  C. Topic-interface trajectory: ~2 s after its last point the stack log
     contains "Aborted due to command timeout" and the arm then stays put.

Simulation only — the real arm stays powered off. Exit 0 = all passed.
"""

import json
import os
import re
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "99"
VCAN = os.environ.get("F99_VCAN", "vcan99")
DYN_FILE = "/tmp/f99_dynamics.json"
SIM_LOG = "/tmp/f99_sim.log"
STACK_LOG = "/tmp/f99_stack.log"

JOINTS = ["L1_joint", "L2_joint", "L3_joint",
          "L4_joint", "L5_joint", "L6_joint"]

RESULTS = []


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


def call(node, cli, request, timeout=30.0):
    fut = cli.call_async(request)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    raise RuntimeError(f"service timeout: {cli.srv_name}")


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f99_acceptance")
        self.q = None
        self.create_subscription(
            JointState, "/joint_states", self._on_js, 10)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)

    def _on_js(self, msg):
        cur = {}
        for j in JOINTS:
            if j in msg.name:
                cur[j] = float(msg.position[msg.name.index(j)])
        if len(cur) == len(JOINTS):
            self.q = cur

    def wait_js(self, timeout=40):
        t0 = time.monotonic()
        while self.q is None and time.monotonic() - t0 < timeout:
            spin(self, 0.1)
        return self.q is not None

    def enable_to_ready(self):
        assert self.enable_cli.wait_for_service(timeout_sec=20)
        resp = call(self, self.enable_cli, Trigger.Request())
        if not resp.success:
            raise RuntimeError(f"enable rejected: {resp.message}")
        spin(self, 8.0)


def ensure_vcan():
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "add", "dev", VCAN, "type", "vcan"],
        input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "set", VCAN, "up"],
        input=b"temppwd\n", capture_output=True)


def read_cmd_timeout(env):
    # ros2_control gives each active controller its own node (/arm_controller);
    # /controller_manager only exposes arm_controller.type.
    proc = subprocess.run(
        ["ros2", "param", "get", "/arm_controller", "cmd_timeout"],
        env=env, capture_output=True, text=True, timeout=20)
    m = re.search(r"Double value is:? ([0-9]+\.[0-9]+)", proc.stdout)
    return float(m.group(1)) if m else None


def make_trajectory(node, delta_l1, duration_s):
    msg = JointTrajectory()
    msg.joint_names = JOINTS
    p0 = JointTrajectoryPoint()
    p0.positions = [node.q[j] for j in JOINTS]
    p0.time_from_start.sec = 0
    p1 = JointTrajectoryPoint()
    p1.positions = [node.q[j] for j in JOINTS]
    p1.positions[0] += delta_l1
    p1.time_from_start.sec = int(duration_s)
    p1.time_from_start.nanosec = int((duration_s % 1) * 1e9)
    msg.points = [p0, p1]
    return msg


def action_goal(node, delta_l1, duration_s=2.5, timeout=25.0):
    ac = ActionClient(
        node, FollowJointTrajectory,
        "/arm_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=10):
        raise RuntimeError("follow_joint_trajectory action absent")
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = make_trajectory(node, delta_l1, duration_s)
    gh_fut = ac.send_goal_async(goal)
    end = time.monotonic() + timeout
    while not gh_fut.done() and time.monotonic() < end:
        spin(node, 0.05)
    handle = gh_fut.result()
    if not handle.accepted:
        raise RuntimeError("action goal rejected")
    res_fut = handle.get_result_async()
    end = time.monotonic() + timeout
    while not res_fut.done() and time.monotonic() < end:
        spin(node, 0.05)
    return res_fut.result().result


def log_contains(pattern):
    with open(STACK_LOG, errors="replace") as f:
        return pattern in f.read()


def main():
    # Driver node shares the stack domain/RMW (LL-109).
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["PYTHONNOUSERSITE"] = "1"

    env = make_env()
    ensure_vcan()
    with open(DYN_FILE, "w") as f:
        json.dump({}, f)

    rclpy.init()
    node = HarnessNode()
    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", VCAN, "--dynamics-file", DYN_FILE],
        SIM_LOG, env=env)
    time.sleep(0.8)
    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={VCAN}",
         "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"],
        STACK_LOG, env=env)

    try:
        if not node.wait_js():
            raise RuntimeError("no /joint_states in 40 s")
        node.enable_to_ready()

        val = read_cmd_timeout(env)
        check("A cmd_timeout param = 2.0", val == 2.0, f"got {val}")
        check("A no 'must be higher than goal_time' configure warning",
              not log_contains("Command timeout must be higher"),
              "(parameter would be silently ignored)")

        r = action_goal(node, 0.10)
        check("B action trajectory SUCCESSFUL",
              r.error_code == FollowJointTrajectory.Result.SUCCESSFUL,
              f"error_code={r.error_code} {r.error_string}")
        spin(node, 1.0)

        # Topic-interface trajectory back to the previous L1 position.
        msg = make_trajectory(node, -0.10, 2.5)
        node.traj_pub.publish(msg)
        spin(node, 0.3)  # let DDS deliver
        l1_at_end = None
        t0 = time.monotonic()
        fired = False
        while time.monotonic() - t0 < 8.0:
            spin(node, 0.1)
            if log_contains("Aborted due to command timeout"):
                fired = True
                l1_at_end = node.q["L1_joint"]
                break
        check("C topic trajectory cut to hold ~2 s after last point",
              fired, "(no 'Aborted due to command timeout' in stack log)")
        if fired:
            spin(node, 1.5)
            drift = abs(node.q["L1_joint"] - l1_at_end)
            check("C position stable after command timeout",
                  drift < 0.02, f"drift={drift:.3f}")
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        stack.terminate()
        sim.terminate()
        if os.path.exists(DYN_FILE):
            os.remove(DYN_FILE)
        node.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F99 acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
