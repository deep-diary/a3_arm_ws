#!/usr/bin/env python3
"""F75 验收：edge_full_mock 全产品 mock-hardware 标准栈。

前置栈（无 CAN / 无电机）：
  ROS_DOMAIN_ID=61 ros2 launch a3_bringup edge_full_mock.launch.py

验收项（对应 docs/edge/REQUIREMENTS.md F75）：
  0. boot：JTC inactive、FSM IDLE；零自研 sim 节点
  1. enable → 两 JTC active、FSM READY
  2. jog（含 L7）落点 ≤0.02；/joint_states 速度字段有效（LL-072）
  3. goto move_group：ready / home 落点
  4. playback：retime + 双 JTC 回 home
  5. 夹爪 /a3/gripper/command 位置命令 → L7 经标准 JTC 实际运动
  6. disable safe-park → home，两 JTC inactive、FSM DISABLED
  7. MQTT 桥节点全程存活
"""

import math
import os
import shutil
import sys
import time
from collections import deque
from statistics import median

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from a3_msgs.msg import ArmStatus
from a3_msgs.srv import (
    GotoNamedPose,
    GripperCommand,
    PlaybackTrajectory,
    SetJointPositions,
)
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

JOINTS = [f"L{i}_joint" for i in range(1, 8)]
TOL = 0.02
RESULTS = []
FORBIDDEN_NODES = {"motor_protocol_node", "sim_power_sequence_node", "gravity_torque_node"}
PRODUCT_NODES = {
    "a3_arm_controller", "a3_gripper_controller", "ros2mqtt_bridge",
    "a3_trajectory_processing", "move_group", "controller_manager",
}


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")


class Recorder:
    def __init__(self, node):
        self.node = node
        self.samples = deque(maxlen=400)
        self.max_vel = {j: 0.0 for j in JOINTS}

    def cb(self, msg):
        snap_p = {n: p for n, p in zip(msg.name, msg.position)}
        snap_v = {n: v for n, v in zip(msg.name, msg.velocity)}
        self.samples.append((time.monotonic(), snap_p))
        for j, v in snap_v.items():
            if math.isfinite(v):
                self.max_vel[j] = max(self.max_vel.get(j, 0.0), abs(v))

    def current(self):
        if len(self.samples) < 5:
            return None
        tail = [s for _, s in list(self.samples)[-11:]]
        return {j: median(s[j] for s in tail if j in s) for j in JOINTS}


class StateWatcher:
    def __init__(self, node):
        self.state = ""
        node.create_subscription(ArmStatus, "/a3/arm_status", self._cb, 10)

    def _cb(self, msg):
        self.state = msg.state


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


def wait_state(target, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        spin(0.1)
        if watcher.state == target:
            return True
    return False


def wait_land(target, timeout=16.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        spin(0.1)
        cur = recorder.current()
        if cur is not None:
            err = max(abs(cur[j] - target[i]) for i, j in enumerate(JOINTS))
            if err < TOL:
                return True, err
    cur = recorder.current()
    per = [abs(cur[j] - target[i]) for i, j in enumerate(JOINTS)]
    return False, max(per)


def controller_states():
    r = call(list_cli, ListControllers.Request(), timeout=5.0)
    return {c.name: c.state for c in r.controller}


def graph_node_names():
    spin(0.5)
    names = set()
    for name, ns in node.get_node_names_and_namespaces():
        names.add(ns.rstrip("/") + "/" + name if ns != "/" else "/" + name)
    bare = {n.rsplit("/", 1)[-1] for n in names}
    return names, bare


def load_poses():
    share = get_package_share_directory("a3_description")
    with open(os.path.join(share, "config", "named_poses.yaml"), encoding="utf-8") as f:
        data = yaml.safe_load(f)
    poses = {k: list(v["positions"]) for k, v in data["poses"].items()}
    user_path = os.path.expanduser("~/.a3/poses.yaml")
    if os.path.exists(user_path):
        udata = yaml.safe_load(open(user_path, encoding="utf-8")) or {}
        for name, spec in (udata.get("poses") or {}).items():
            poses[name] = list(spec["positions"] if isinstance(spec, dict) else spec)
    return poses


def write_latest(home):
    traj_dir = os.path.expanduser("~/.a3/trajectories")
    os.makedirs(traj_dir, exist_ok=True)
    path = os.path.join(traj_dir, "latest.yaml")
    if os.path.exists(path):
        shutil.copy(path, "/tmp/f75_latest_backup.yaml")
    amps = [0.15, 0.20, 0.20, 0.15, 0.15, 0.10, 0.25]
    T, N = 6.0, 61
    points = []
    for k in range(N):
        u = k / (N - 1)
        s = math.sin(math.pi * u)
        points.append({
            "positions": [home[i] + amps[i] * s for i in range(7)],
            "time_from_start_sec": u * T,
        })
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump({"joint_names": JOINTS, "points": points}, f)
    return path


node = recorder = watcher = list_cli = None


def main():
    global node, recorder, watcher, list_cli
    if os.environ.get("ROS_DOMAIN_ID") != "61":
        print("WARN: ROS_DOMAIN_ID != 61", file=sys.stderr)

    rclpy.init()
    node = rclpy.create_node("f75_acceptance")
    node.create_subscription(JointState, "/joint_states", lambda m: recorder.cb(m), 20)
    recorder = Recorder(node)
    watcher = StateWatcher(node)

    list_cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
    enable_cli = node.create_client(Trigger, "/a3/arm/enable")
    disable_cli = node.create_client(Trigger, "/a3/arm/disable")
    jog_cli = node.create_client(SetJointPositions, "/a3/arm/set_joint_positions")
    goto_cli = node.create_client(GotoNamedPose, "/a3/arm/goto_named_pose")
    playback_cli = node.create_client(PlaybackTrajectory, "/a3/arm/playback")
    gripper_cli = node.create_client(GripperCommand, "/a3/gripper/command")

    print("waiting for stack ...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30:
        spin(0.1)
        if recorder.current() is not None and all(
                c.wait_for_service(timeout_sec=0.0)
                for c in (list_cli, enable_cli, jog_cli, goto_cli, playback_cli)):
            break
    else:
        print("FATAL: stack not ready")
        return 1

    # ---- 0. boot ----
    states = controller_states()
    boot_inactive = states.get("arm_controller") == "inactive" and \
        states.get("gripper_controller") == "inactive"
    check("0a JTC inactive at boot", boot_inactive, str(states))
    names, bare = graph_node_names()
    leaked = FORBIDDEN_NODES & bare
    check("0b zero custom sim nodes", not leaked, f"leaked={leaked or 'none'}")
    check("0c product nodes present", PRODUCT_NODES <= bare,
          f"missing={PRODUCT_NODES - bare or 'none'}")

    # ---- 1. enable ----
    r = call(enable_cli, Trigger.Request())
    ok_ready = r.success and wait_state("READY")
    check("1 enable -> READY", ok_ready, r.message)
    states = controller_states()
    active = states.get("arm_controller") == "active" and \
        states.get("gripper_controller") == "active"
    check("1b JTC active", active, str(states))
    if not ok_ready:
        return 1

    poses = load_poses()
    home, ready = poses["home"], poses["ready"]

    # ---- 2. jog ----
    jog_targets = [
        [0.20, 0.40, -0.45, -0.20, 0.15, 0.30, 0.25],
        [0.05, 0.10, -0.05, 0.10, -0.05, 0.05, 0.10],
        [-0.20, 0.30, -0.25, 0.20, -0.15, -0.30, 0.45],
    ]
    for k, tgt in enumerate(jog_targets):
        recorder.max_vel = {j: 0.0 for j in JOINTS}
        r = call(jog_cli, SetJointPositions.Request(positions=tgt, duration=2.8))
        if not r.success:
            check(f"2.{k} jog dispatch", False, r.message)
            continue
        ok, err = wait_land(tgt)
        if not ok:
            cur = recorder.current()
            detail = "per=" + ",".join(
                f"{JOINTS[i]}:{abs(cur[j]-tgt[i]):.3f}" for i, j in enumerate(JOINTS))
        else:
            detail = f"err={err:.4f}"
        check(f"2.{k} jog land", ok, detail)
        wait_state("READY")
    vmax = max(recorder.max_vel.values())
    check("2b /joint_states velocity valid", vmax > 0.05, f"max_vel={vmax:.3f}")

    # ---- 3. goto move_group ----
    for k, name in enumerate(("ready", "home")):
        r = call(goto_cli, GotoNamedPose.Request(pose_name=name))
        if not r.success:
            check(f"3.{k} goto {name}", False, r.message)
            continue
        ok, err = wait_land(poses[name])
        check(f"3.{k} goto move_group {name}", ok, f"err={err:.4f}")
        wait_state("READY")

    # ---- 4. playback ----
    path = write_latest(home)
    r = call(playback_cli, PlaybackTrajectory.Request(name=""))
    if not r.success:
        check("4 playback dispatch", False, r.message)
    else:
        ok, err = wait_land(home, timeout=20.0)
        check("4 playback retime land home", ok, f"err={err:.4f}")
        wait_state("READY")
    if os.path.exists("/tmp/f75_latest_backup.yaml"):
        shutil.copy("/tmp/f75_latest_backup.yaml", path)

    # ---- 5. gripper position → L7 via standard JTC topic ----
    cur0 = recorder.current()[JOINTS[6]]
    r = call(gripper_cli, GripperCommand.Request(mode="position", position=0.0))
    spin(1.5)
    mid = recorder.current()[JOINTS[6]]
    r2 = call(gripper_cli, GripperCommand.Request(mode="position", position=1.0))
    spin(1.5)
    cur1 = recorder.current()[JOINTS[6]]
    moved = abs(cur1 - cur0) > 0.10 or abs(mid - cur0) > 0.05
    check("5 gripper position moves L7", moved and r.success and r2.success,
          f"L7 {cur0:.3f}->{mid:.3f}->{cur1:.3f}")

    # ---- 6. disable safe-park ----
    r = call(disable_cli, Trigger.Request())
    t0 = time.monotonic()
    parked = False
    while time.monotonic() - t0 < 18:
        spin(0.1)
        cur = recorder.current()
        err = max(abs(cur[j] - home[i]) for i, j in enumerate(JOINTS))
        states = controller_states()
        if err < TOL and watcher.state == "DISABLED" and \
                states.get("arm_controller") == "inactive" and \
                states.get("gripper_controller") == "inactive":
            parked = True
            break
    check("6 safe-park + JTC inactive + DISABLED", parked,
          f"state={watcher.state} states={states}")

    # ---- 7. mqtt alive ----
    _, bare = graph_node_names()
    check("7 mqtt bridge alive", "ros2mqtt_bridge" in bare, "")

    total = len(RESULTS)
    passed = sum(1 for _, ok, _ in RESULTS if ok)
    print(f"\n==== F75 acceptance: {passed}/{total} ====")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
