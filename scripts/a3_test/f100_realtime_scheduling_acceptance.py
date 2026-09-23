#!/usr/bin/env python3
"""F100 acceptance: controller_manager runs its control loop under SCHED_FIFO.

Without RT privilege (ulimit -r = 0) ros2_control logs "Could not enable
FIFO RT scheduling policy: Operation not permitted" and falls back to
SCHED_OTHER, so the 200 Hz read/update/write loop can be preempted by
ordinary loads. F100 deploys the standard ros2_control setup:

  - realtime group + /etc/security/limits.d (rtprio 99 / memlock unlimited)
  - systemd unit LimitRTPRIO=99 / LimitMEMLOCK=infinity (separate check)

This script applies the PAM-side setup, then launches the stack inside a
fresh login session (su - <user>) so pam_limits actually applies.

  python3 scripts/a3_test/f100_realtime_scheduling_acceptance.py [DOMAIN_ID]

Acceptance (docs/edge/REQUIREMENTS.md F100):
  A. New login session: ulimit -r = 99, ulimit -l = unlimited.
  B. Stack log has no "Could not enable FIFO RT scheduling policy".
  C. controller_manager RT thread runs SCHED_FIFO (priority 50). CM calls
     sched_setscheduler on its dedicated RT thread, not the process main
     thread, so /proc/<pid>/task/* must be scanned instead of chrt -p <pid>.
  D. enable + two-point action trajectory still SUCCESSFUL.

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
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "100"
USER = os.environ.get("F100_USER", "cat")
SUDO_PW = b"temppwd\n"
VCAN = os.environ.get("F100_VCAN", "vcan100")
DYN_FILE = "/tmp/f100_dynamics.json"
WRAP_FILE = "/tmp/f100_stack_wrapper.sh"
SIM_LOG = "/tmp/f100_sim.log"
STACK_LOG = "/tmp/f100_stack.log"

JOINTS = ["L1_joint", "L2_joint", "L3_joint",
          "L4_joint", "L5_joint", "L6_joint"]

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def sudo_run(argv, timeout=60):
    return subprocess.run(
        ["sudo", "-S"] + argv, input=SUDO_PW, capture_output=True,
        timeout=timeout)


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
        super().__init__("f100_acceptance")
        self.q = None
        self.create_subscription(
            JointState, "/joint_states", self._on_js, 10)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")

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


def find_fifo_thread(deadline_s=20):
    # controller_manager spawns an "RT thread" and sets SCHED_FIFO (prio 50)
    # on it; the process main thread stays SCHED_OTHER. Scan every thread of
    # every ros2_control_node until the FIFO thread appears.
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        pids = subprocess.run(
            ["pgrep", "-f", "ros2_control_node"],
            capture_output=True, text=True).stdout.split()
        for pid in pids:
            try:
                tids = os.listdir(f"/proc/{pid}/task")
            except OSError:
                continue
            for tid in tids:
                r = subprocess.run(
                    ["chrt", "-p", tid], capture_output=True, text=True)
                if "SCHED_FIFO" in r.stdout:
                    return tid, r.stdout.replace("\n", " ")
        time.sleep(0.5)
    return None, ""


def ensure_vcan():
    sudo_run(["modprobe", "vcan"])
    sudo_run(["ip", "link", "add", "dev", VCAN, "type", "vcan"])
    sudo_run(["ip", "link", "set", VCAN, "up"])


def action_goal(node, delta_l1, duration_s=2.5, timeout=25.0):
    ac = ActionClient(
        node, FollowJointTrajectory,
        "/arm_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=10):
        raise RuntimeError("follow_joint_trajectory action absent")

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

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = msg
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


class LoginStack:
    """Stack launched in a fresh PAM login session via su - (setsid)."""

    def __init__(self, env):
        self.log = open(STACK_LOG, "wb")
        self.proc = subprocess.Popen(
            ["sudo", "-S", "su", "-", USER, "-c",
             f"setsid bash {WRAP_FILE}"],
            stdin=subprocess.PIPE, stdout=self.log, stderr=subprocess.STDOUT,
            env=env)
        self.proc.stdin.write(SUDO_PW)
        self.proc.stdin.flush()
        self.pgid = None

    def find_pgid(self, timeout=40):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            out = subprocess.run(
                ["pgrep", "-f", f"a3_bringup.launch.py.*{VCAN}"],
                capture_output=True, text=True).stdout.split()
            if out:
                pid = int(out[0])
                self.pgid = int(subprocess.run(
                    ["ps", "-o", "pgid=", "-p", str(pid)],
                    capture_output=True, text=True).stdout.strip())
                return True
            time.sleep(0.3)
        return False

    def terminate(self):
        if self.pgid is not None:
            try:
                os.killpg(self.pgid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        try:
            self.proc.wait(timeout=12)
            return
        except subprocess.TimeoutExpired:
            pass
        if self.pgid is not None:
            try:
                os.killpg(self.pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.proc.wait(timeout=10)


def main():
    # Driver node shares the stack domain/RMW (LL-109).
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["PYTHONNOUSERSITE"] = "1"

    env = make_env()
    ensure_vcan()
    with open(DYN_FILE, "w") as f:
        json.dump({}, f)

    r = sudo_run(["bash", f"{WS}/scripts/setup/setup_realtime.sh", USER])
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        raise RuntimeError("setup_realtime.sh failed")

    # A: limits in a fresh PAM login session.
    r = sudo_run(["su", "-", USER, "-c", "ulimit -r; ulimit -l"])
    lines = r.stdout.decode().split()
    check("A login session rtprio=99", "99" in lines, f"got {r.stdout!r}")
    check("A login session memlock=unlimited",
          "unlimited" in lines, f"got {r.stdout!r}")

    with open(WRAP_FILE, "w") as f:
        f.write(f"""#!/usr/bin/env bash
set -e
cd {WS}
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID={DOMAIN}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec ros2 launch a3_bringup a3_bringup.launch.py hardware:=can \
  can_interface:={VCAN} use_mqtt:=false use_teleop:=false use_rviz:=false
""")

    sim_proc = subprocess.Popen(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", VCAN, "--dynamics-file", DYN_FILE],
        stdout=open(SIM_LOG, "wb"), stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL, start_new_session=True, env=env)
    time.sleep(0.8)

    stack = LoginStack(env)
    rclpy.init()
    node = HarnessNode()

    try:
        if not stack.find_pgid():
            raise RuntimeError("stack launch process not found")
        if not node.wait_js():
            raise RuntimeError("no /joint_states in 40 s")

        with open(STACK_LOG, errors="replace") as f:
            log_text = f.read()
        check("B no 'Could not enable FIFO' fallback warning",
              "Could not enable FIFO RT scheduling policy" not in log_text,
              "(controller_manager fell back to SCHED_OTHER)")

        tid, detail = find_fifo_thread()
        check("C controller_manager RT thread runs SCHED_FIFO",
              tid is not None, detail or "(no FIFO thread found)")

        node.enable_to_ready()
        res = action_goal(node, 0.10)
        check("D action trajectory SUCCESSFUL",
              res.error_code == FollowJointTrajectory.Result.SUCCESSFUL,
              f"error_code={res.error_code} {res.error_string}")
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        stack.terminate()
        try:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        for p in (DYN_FILE, WRAP_FILE):
            if os.path.exists(p):
                os.remove(p)
        node.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F100 acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
