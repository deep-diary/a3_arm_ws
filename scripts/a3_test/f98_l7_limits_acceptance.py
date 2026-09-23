#!/usr/bin/env python3
"""F98 acceptance: L7 gripper limits match the real calibration.

Calibration (src/a3_gripper_controller/config/gripper_config.yaml,
2026-09-13 motor): open=0.0 rad, mechanical stop 1.7825 rad, operational
clamp 1.78 rad. Before F98 the URDF, ros2_control position command
interface and MoveIt joint_limits all carried the reBot reference value
+/-1.5708 — the model understated gripper travel by 12% and allowed
physically meaningless negative commands.

  python3 scripts/a3_test/f98_l7_limits_acceptance.py [DOMAIN_ID]

Acceptance (docs/edge/REQUIREMENTS.md F98):
  A. Static model consistency: xacro-expanded URDF L7 joint limit and
     ros2_control position command interface are [0.0, 1.78]; MoveIt
     joint_limits.yaml L7 is [0.0, 1.78]; L5/L6 stay +/-1.5708.
  B. vcan stack: GripperCommand position=1.78 reaches >= 1.65 rad
     (past the old 1.5708 ceiling), reached_goal=true.
  C. GripperCommand position=0.0 returns L7 <= 0.05 rad.

Simulation only — the real arm stays powered off. Exit 0 = all passed.
"""

import json
import os
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "98"
VCAN = os.environ.get("F98_VCAN", "vcan98")
DYN_FILE = "/tmp/f98_dynamics.json"
SIM_LOG = "/tmp/f98_sim.log"
STACK_LOG = "/tmp/f98_stack.log"

URDF_XACRO = f"{WS}/src/a3_description/urdf/el_a3.urdf.xacro"
JOINT_LIMITS_YAML = f"{WS}/src/a3_moveit_config/config/joint_limits.yaml"

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


# ---------------- Phase A: static model consistency ----------------

def phase_a_static(env):
    proc = subprocess.run(
        ["xacro", URDF_XACRO,
         "use_mock_hardware:=false", "use_real_hardware:=true",
         f"can_interface:={VCAN}"],
        env=env, capture_output=True, text=True, timeout=30)
    if proc.returncode != 0:
        check("A xacro expands", False, proc.stderr[-300:])
        return
    check("A xacro expands", True)
    root = ET.fromstring(proc.stdout)

    def joint_limit(jname):
        j = root.find(f".//joint[@name='{jname}']")
        lim = j.find("limit")
        return float(lim.get("lower")), float(lim.get("upper"))

    check("A URDF L7 joint limit [0.0, 1.78]",
          joint_limit("L7_joint") == (0.0, 1.78),
          str(joint_limit("L7_joint")))
    for jname in ("L5_joint", "L6_joint"):
        check(f"A URDF {jname} unchanged +/-1.5708",
              joint_limit(jname) == (-1.5708, 1.5708),
              str(joint_limit(jname)))

    # ros2_control position command_interface for L7
    r2c = root.find(".//ros2_control")
    l7 = r2c.find("./joint[@name='L7_joint']")
    pci = l7.find("./command_interface[@name='position']")
    params = {p.get("name"): float(p.text) for p in pci.findall("param")}
    check("A ros2_control L7 position interface [0.0, 1.78]",
          params == {"min": 0.0, "max": 1.78}, str(params))

    with open(JOINT_LIMITS_YAML) as f:
        limits = yaml.safe_load(f)["joint_limits"]
    l7ml = limits["L7_joint"]
    check("A MoveIt joint_limits L7 [0.0, 1.78]",
          l7ml["min_position"] == 0.0 and l7ml["max_position"] == 1.78,
          f"min={l7ml['min_position']} max={l7ml['max_position']}")
    for jname in ("L5_joint", "L6_joint"):
        jm = limits[jname]
        check(f"A MoveIt {jname} unchanged +/-1.5708",
              jm["min_position"] == -1.5708
              and jm["max_position"] == 1.5708)


# ---------------- Phases B/C: full travel over vcan ----------------

class HarnessNode(Node):
    def __init__(self):
        super().__init__("f98_acceptance")
        self.l7 = None
        self.create_subscription(
            JointState, "/joint_states", self._on_js, 10)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")

    def _on_js(self, msg):
        if "L7_joint" in msg.name:
            self.l7 = float(msg.position[msg.name.index("L7_joint")])

    def wait_js(self, timeout=40):
        t0 = time.monotonic()
        while self.l7 is None and time.monotonic() - t0 < timeout:
            spin(self, 0.1)
        return self.l7 is not None

    def enable_to_ready(self):
        assert self.enable_cli.wait_for_service(timeout_sec=20)
        resp = call(self, self.enable_cli, Trigger.Request())
        if not resp.success:
            raise RuntimeError(f"enable rejected: {resp.message}")
        spin(self, 8.0)


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


def gripper_goal(node, position, max_effort=1.0, timeout=25.0):
    ac = ActionClient(
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


def ensure_vcan():
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "add", "dev", VCAN, "type", "vcan"],
        input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "set", VCAN, "up"],
        input=b"temppwd\n", capture_output=True)


def main():
    # Driver node itself must use the same domain/RMW as the spawned stack;
    # env vars passed to ProcessGroup do not affect this process (LL-109).
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["PYTHONNOUSERSITE"] = "1"

    env = make_env()
    phase_a_static(env)

    ensure_vcan()
    with open(DYN_FILE, "w") as f:
        json.dump({"7": {"gravity_nm": 0.0}}, f)

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
        ctrls = list_controllers(env)
        check("B gripper_controller active after enable",
              ctrls.get("gripper_controller", ("", ""))[1] == "active",
              str(ctrls.get("gripper_controller")))

        r = gripper_goal(node, 1.78)
        check("B close 1.78 reached_goal=true", r.reached_goal,
              f"pos={r.position:.3f} stalled={r.stalled}")
        check("B close position past old 1.5708 ceiling (>=1.65)",
              r.position >= 1.65, f"pos={r.position:.3f}")

        r = gripper_goal(node, 0.0)
        check("C open 0.0 reached_goal=true", r.reached_goal,
              f"pos={r.position:.3f} stalled={r.stalled}")
        check("C open position <= 0.05", r.position <= 0.05,
              f"pos={r.position:.3f}")
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
    print(f"\n==== F98 acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
