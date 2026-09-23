#!/usr/bin/env python3
"""F101 acceptance: hardware watchdog + systemd service watchdog.

Two failure modes get deterministic recovery:

  1. Total kernel/scheduler deadlock -> systemd feeds the hardware watchdog
     (RuntimeWatchdogSec=10); a missed ping hard-resets the board.
  2. Control-stack hang -> /joint_states stops, the feeder node stops sending
     WATCHDOG=1, systemd restarts the Type=notify unit after WatchdogSec.

Acceptance (docs/edge/REQUIREMENTS.md F101):
  1. Hardware watchdog enabled: RuntimeWatchdogUSec = 10s after setup script.
  2. Temp unit a3-f101test.service (vcan101, Type=notify/WatchdogSec=10)
     becomes active within 30 s (READY=1 on first /joint_states), NRestarts=0.
  3. No functional regression: enable + standard two-point action SUCCESSFUL.
  4. Deterministic restart on hang: kill vcan_motor_sim -> NRestarts >= 1
     within 25 s.

Simulation only — the real arm stays powered off. Exit 0 = all passed.
"""

import json
import os
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
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "101"
SUDO_PW = b"temppwd\n"
VCAN = os.environ.get("F101_VCAN", "vcan101")
TEST_UNIT = "a3-f101test"
UNIT_DST = f"/etc/systemd/system/{TEST_UNIT}.service"
PROD_UNIT = os.path.join(WS, "systemd", "a3-arm.service")
DYN_FILE = "/tmp/f101_dynamics.json"
SIM_LOG = "/tmp/f101_sim.log"
UNIT_TMP = "/tmp/f101_test_unit.service"

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
        super().__init__("f101_acceptance")
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


def ensure_vcan():
    sudo_run(["modprobe", "vcan"])
    exists = subprocess.run(
        ["ip", "link", "show", VCAN], capture_output=True, text=True)
    if exists.returncode != 0:
        sudo_run(["ip", "link", "add", "dev", VCAN, "type", "vcan"])
    sudo_run(["ip", "link", "set", VCAN, "up"])


def systemctl(*args, timeout=30):
    return sudo_run(["systemctl"] + list(args), timeout=timeout)


def unit_prop(prop):
    r = subprocess.run(
        ["systemctl", "show", "-p", prop, "--value", TEST_UNIT],
        capture_output=True, text=True)
    return r.stdout.strip()


def write_test_unit():
    unit = f"""\
[Unit]
Description=F101 temporary watchdog acceptance unit
After=network-online.target
StartLimitIntervalSec=60
StartLimitBurst=8

[Service]
Type=notify
NotifyAccess=all
TimeoutStartSec=60
WatchdogSec=10
Restart=always
RestartSec=2
User=cat
Group=cat
WorkingDirectory={WS}
Environment=PYTHONNOUSERSITE=1
Environment=ROS_DOMAIN_ID={DOMAIN}
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ExecStart=/usr/bin/bash -c 'source /opt/ros/humble/setup.bash && source {WS}/install/local_setup.bash && exec ros2 launch a3_bringup a3_bringup.launch.py hardware:=can can_interface:={VCAN} use_mqtt:=false use_teleop:=false use_rviz:=false'
KillSignal=SIGINT
KillMode=mixed
TimeoutStopSec=20
"""
    with open(UNIT_TMP, "w") as f:
        f.write(unit)
    r = sudo_run(["install", "-m", "0644", UNIT_TMP, UNIT_DST])
    return r.returncode == 0


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["PYTHONNOUSERSITE"] = "1"

    env = make_env()
    ensure_vcan()
    with open(DYN_FILE, "w") as f:
        json.dump({}, f)

    # systemd-analyze verify on the production unit (Type=notify changes).
    r = subprocess.run(
        ["systemd-analyze", "verify", PROD_UNIT],
        capture_output=True, text=True, timeout=20)
    errors = [ln for ln in r.stderr.splitlines() if "ERROR" in ln]
    check("systemd-analyze verify a3-arm.service", not errors,
          "; ".join(errors) if errors else "exit 0")

    # --- 1. Hardware watchdog enabled ---
    r = sudo_run(["bash", f"{WS}/scripts/setup/setup_watchdog.sh"])
    check("setup_watchdog.sh runs", r.returncode == 0,
          (r.stderr.decode()[-200:] if r.returncode else ""))
    cur = subprocess.run(
        ["systemctl", "show", "-p", "RuntimeWatchdogUSec", "--value"],
        capture_output=True, text=True).stdout.strip()
    check("1 RuntimeWatchdogUSec=10s (11s actual after HW round-up)",
          cur in ("10s", "11s"), f"got {cur}")

    # --- 2. Temp notify unit becomes active within 30 s ---
    check("install temp unit", write_test_unit())
    systemctl("daemon-reload")

    sim_proc = subprocess.Popen(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", VCAN, "--dynamics-file", DYN_FILE],
        stdout=open(SIM_LOG, "wb"), stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL, start_new_session=True, env=env)
    time.sleep(0.8)

    rclpy.init()
    node = HarnessNode()
    restart_seen_within = None
    try:
        # Type=notify start blocks until READY=1; poll instead with --no-block.
        systemctl("start", "--no-block", TEST_UNIT, timeout=10)

        t0 = time.monotonic()
        active = False
        while time.monotonic() - t0 < 30:
            if unit_prop("ActiveState") == "active":
                active = True
                break
            time.sleep(0.5)
        nrestarts = unit_prop("NRestarts")
        check("2 unit active within 30 s", active,
              f"state={unit_prop('ActiveState')}")
        check("2 NRestarts=0 at startup", nrestarts == "0",
              f"got {nrestarts}")

        if not active:
            raise RuntimeError("unit never active; skipping remaining checks")

        if not node.wait_js():
            raise RuntimeError("no /joint_states in 40 s")

        # --- 3. Functional regression: enable + two-point action ---
        node.enable_to_ready()
        res = action_goal(node, 0.10)
        check("3 action trajectory SUCCESSFUL",
              res.error_code == FollowJointTrajectory.Result.SUCCESSFUL,
              f"error_code={res.error_code} {res.error_string}")

        # --- 4. Kill feedback -> deterministic restart within 25 s ---
        n0 = int(unit_prop("NRestarts"))
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)

        t0 = time.monotonic()
        n1 = n0
        while time.monotonic() - t0 < 25:
            n1 = int(unit_prop("NRestarts"))
            if n1 > n0:
                restart_seen_within = time.monotonic() - t0
                break
            time.sleep(0.5)
        check("4 NRestarts >= 1 within 25 s", n1 > n0,
              f"{n0} -> {n1}"
              + (f" after {restart_seen_within:.1f}s"
                 if restart_seen_within else ""))
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        systemctl("stop", TEST_UNIT, timeout=30)
        sudo_run(["rm", "-f", UNIT_DST])
        systemctl("daemon-reload")
        try:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        for p in (DYN_FILE, UNIT_TMP):
            if os.path.exists(p):
                os.remove(p)
        node.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F101 acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
