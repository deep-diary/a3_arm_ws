#!/usr/bin/env python3
"""F77 acceptance: PS4 D-pad joint jog via MoveIt Servo JointJog.

Run against the F75/F76 standard stack (arm powered off, mock hardware):

  ROS_DOMAIN_ID=63 ros2 launch a3_bringup edge_full_mock.launch.py use_mqtt:=false
  ROS_DOMAIN_ID=63 python3 scripts/a3_test/f77_joint_jog_acceptance.py

Checks:
  1. enable -> both JTC active, FSM READY; servo_node + mode bridge present
  2. JointJog L1 +velocity -> L1 moves positive, real velocity in /joint_states
  3. stop publishing -> holds position, control_mode IDLE after timeout, FSM READY
  4. JointJog L1 -velocity -> moves back; joint limits never violated
  5. zero messages on legacy /joint_group_effort_controller/joint_trajectory
  6. L7 gripper path still works (position service -> standard gripper JTC)
"""

import math
import sys
import time
from collections import deque
from statistics import median

import rclpy
from a3_msgs.msg import ArmStatus
from a3_msgs.srv import GripperCommand
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped  # noqa: F401  (ensures servo dep present)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from rclpy.qos import QoSProfile, ReliabilityPolicy

JOINTS = [f"L{i}_joint" for i in range(1, 8)]
LIMITS = {
    "L1_joint": 2.7925,
    "L2_joint": (0.0, 3.66519),
    "L3_joint": (-4.01426, 0.0),
    "L4_joint": 1.5708,
    "L5_joint": 1.5708,
    "L6_joint": 1.5708,
    "L7_joint": 1.5708,
}
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")


class Recorder:
    def __init__(self, node):
        self.samples = deque(maxlen=800)
        self.max_vel = {j: 0.0 for j in JOINTS}
        self.min_pos = {j: math.inf for j in JOINTS}
        self.max_pos = {j: -math.inf for j in JOINTS}

    def cb(self, msg):
        snap_p = {n: p for n, p in zip(msg.name, msg.position)}
        snap_v = {n: v for n, v in zip(msg.name, msg.velocity)}
        self.samples.append((time.monotonic(), snap_p))
        for j, p in snap_p.items():
            if j in self.min_pos:
                self.min_pos[j] = min(self.min_pos[j], p)
                self.max_pos[j] = max(self.max_pos[j], p)
        for j, v in snap_v.items():
            if math.isfinite(v):
                self.max_vel[j] = max(self.max_vel.get(j, 0.0), abs(v))

    def current(self):
        if len(self.samples) < 5:
            return None
        tail = [s for _, s in list(self.samples)[-11:]]
        return {j: median(s[j] for s in tail if j in s) for j in JOINTS}


def spin(t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(cli, request, timeout=8.0):
    fut = cli.call_async(request)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    raise RuntimeError(f"service call timeout: {cli.srv_name}")


def limits_ok():
    for j, lim in LIMITS.items():
        lo, hi = (-lim, lim) if isinstance(lim, float) else lim
        if recorder.min_pos[j] < lo - 0.005 or recorder.max_pos[j] > hi + 0.005:
            return False, j, recorder.min_pos[j], recorder.max_pos[j]
    return True, None, 0.0, 0.0


node = recorder = None


def main():
    global node, recorder
    if os.environ.get("ROS_DOMAIN_ID") != "63":
        print("WARN: ROS_DOMAIN_ID != 63", file=sys.stderr)

    rclpy.init()
    node = rclpy.create_node("f77_joint_jog_acceptance")
    recorder = Recorder(node)

    fsm_state = {"v": ""}
    mode_box = {"v": "", "changed_at": 0.0}
    legacy_count = {"n": 0}

    node.create_subscription(
        JointState, "/joint_states", recorder.cb,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    node.create_subscription(
        ArmStatus, "/a3/arm_status",
        lambda m: fsm_state.update(v=m.state), 10,
    )

    def on_mode(m):
        if m.data != mode_box["v"]:
            mode_box["v"] = m.data
            mode_box["changed_at"] = time.monotonic()

    node.create_subscription(String, "/a3/control_mode", on_mode, 10)
    node.create_subscription(
        JointTrajectory, "/joint_group_effort_controller/joint_trajectory",
        lambda m: legacy_count.update(n=legacy_count["n"] + 1), 10,
    )

    list_cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
    enable_cli = node.create_client(Trigger, "/a3/arm/enable")
    disable_cli = node.create_client(Trigger, "/a3/arm/disable")
    start_servo_cli = node.create_client(Trigger, "/servo_node/start_servo")
    pause_servo_cli = node.create_client(Trigger, "/servo_node/pause_servo")
    gripper_cli = node.create_client(GripperCommand, "/a3/gripper/command")

    jog_pub = node.create_publisher(
        JointJog,
        "/servo_node/delta_joint_cmds",
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
    )

    for cli, label in (
        (list_cli, "list_controllers"),
        (enable_cli, "arm/enable"),
        (start_servo_cli, "start_servo"),
    ):
        if not cli.wait_for_service(timeout_sec=15.0):
            print(f"FATAL: {label} unavailable", file=sys.stderr)
            return 2

    # ---- 1. enable -> controllers active, READY; servo + bridge in graph ----
    call(enable_cli, Trigger.Request())
    t0 = time.monotonic()
    states = {}
    while time.monotonic() - t0 < 12:
        spin(0.1)
        states = {c.name: c.state for c in call(
            list_cli, ListControllers.Request(), timeout=5.0).controller}
        if (fsm_state["v"] == "READY"
                and states.get("arm_controller") == "active"
                and states.get("gripper_controller") == "active"):
            break
    spin(0.5)
    bare = {n.rsplit("/", 1)[-1] for n, _ in node.get_node_names_and_namespaces()}
    check(
        "1 enable READY + JTCs active + servo/bridge present",
        fsm_state["v"] == "READY"
        and states.get("arm_controller") == "active"
        and states.get("gripper_controller") == "active"
        and "servo_node" in bare
        and "a3_servo_mode_bridge" in bare,
        f"state={fsm_state['v']} states={states}",
    )

    r = call(start_servo_cli, Trigger.Request())
    check("1b start_servo", bool(r.success), getattr(r, "message", ""))

    spin(1.0)
    q0 = recorder.current()
    if q0 is None:
        print("FATAL: no /joint_states", file=sys.stderr)
        return 2

    def jog(vel, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            msg = JointJog()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.joint_names = JOINTS[:6]
            msg.velocities = [float(vel) if i == 0 else 0.0 for i in range(6)]
            jog_pub.publish(msg)
            spin(0.033)

    # ---- 2. L1 positive jog ----
    jog(0.2, 1.5)
    spin(0.6)
    q1 = recorder.current()
    dpos = q1["L1_joint"] - q0["L1_joint"]
    check(
        "2 JointJog +vel moves L1 positive",
        dpos > 0.05 and recorder.max_vel["L1_joint"] > 0.02,
        f"dL1={dpos:.4f} max_vel={recorder.max_vel['L1_joint']:.3f}",
    )

    # ---- 3. stop -> hold, mode IDLE, FSM stays READY ----
    hold0 = recorder.current()["L1_joint"]
    t_idle0 = time.monotonic()
    saw_idle = False
    fsm_dirty = False
    while time.monotonic() - t_idle0 < 1.2:
        spin(0.05)
        if mode_box["v"] == "IDLE":
            saw_idle = True
        if fsm_state["v"] != "READY":
            fsm_dirty = True
    drift = abs(recorder.current()["L1_joint"] - hold0)
    check(
        "3 stop: hold + control_mode IDLE + FSM READY",
        saw_idle and drift < 0.02 and not fsm_dirty,
        f"drift={drift:.4f} mode={mode_box['v']} state={fsm_state['v']}",
    )

    # ---- 4. L1 negative jog back ----
    jog(-0.2, 1.5)
    spin(0.6)
    q2 = recorder.current()
    dback = q2["L1_joint"] - q1["L1_joint"]
    ok_lim, bad_j, lo_seen, hi_seen = limits_ok()
    check(
        "4 JointJog -vel moves L1 back; limits respected",
        dback < -0.05 and ok_lim,
        f"dL1={dback:.4f}" + ("" if ok_lim else f" LIMIT {bad_j} [{lo_seen},{hi_seen}]"),
    )

    # ---- 5. legacy topic stayed silent ----
    spin(0.3)
    check(
        "5 legacy joint_trajectory zero messages",
        legacy_count["n"] == 0,
        f"count={legacy_count['n']}",
    )

    # ---- 6. L7 gripper path unaffected ----
    l70 = recorder.current()["L7_joint"]
    call(gripper_cli, GripperCommand.Request(mode="position", position=0.0))
    spin(1.2)
    l71 = recorder.current()["L7_joint"]
    call(gripper_cli, GripperCommand.Request(mode="position", position=1.0))
    spin(1.2)
    l72 = recorder.current()["L7_joint"]
    check(
        "6 gripper position service still drives L7",
        abs(l72 - l70) > 0.10 or abs(l71 - l70) > 0.05,
        f"L7 {l70:.3f}->{l71:.3f}->{l72:.3f}",
    )

    # ---- cleanup: pause servo, safe-park disable ----
    call(pause_servo_cli, Trigger.Request())
    call(disable_cli, Trigger.Request())
    t0 = time.monotonic()
    while time.monotonic() - t0 < 18:
        spin(0.1)
        states = {c.name: c.state for c in call(
            list_cli, ListControllers.Request(), timeout=5.0).controller}
        if (fsm_state["v"] == "DISABLED"
                and states.get("arm_controller") == "inactive"
                and states.get("gripper_controller") == "inactive"):
            break
    check(
        "7 cleanup safe-park disable",
        fsm_state["v"] == "DISABLED"
        and states.get("arm_controller") == "inactive"
        and states.get("gripper_controller") == "inactive",
        f"state={fsm_state['v']} states={states}",
    )

    total = len(RESULTS)
    passed = sum(1 for _, ok, _ in RESULTS if ok)
    print(f"\n==== F77 acceptance: {passed}/{total} ====")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed == total else 1


if __name__ == "__main__":
    import os
    sys.exit(main())
