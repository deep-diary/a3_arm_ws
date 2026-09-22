#!/usr/bin/env python3
"""F74 验收：arm_controller 的 control_backend=fjt_action 标准执行后端。

前置栈（F70 mock，无 CAN）：
  ROS_DOMAIN_ID=60 ros2 launch a3_bringup edge_ros2_control_sim.launch.py
  ROS_DOMAIN_ID=60 ros2 launch scripts/a3_test/f74_extra.launch.py

验收项（对应 docs/edge/REQUIREMENTS.md F74）：
  A. jog：set_joint_positions 多点目标（含 L7），落点 ≤ 0.02 rad
  B. goto 线性兜底：goto_use_moveit=false → ready / home
  C. playback：latest.yaml 经 ruckig retime 后 FJT 执行（含 L7）
  D. preempt：连续两个 jog，后发 goal 抢占先发 goal（不排队）
  E. 全程旧话题 /joint_group_effort_controller/joint_trajectory 零消息
  F. disable：safe-park → home，reset 服务被调用
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
from a3_can_bridge.srv import MotorCommand
from a3_msgs.msg import ArmStatus
from a3_msgs.srv import (
    GotoNamedPose,
    PlaybackTrajectory,
    SetJointPositions,
)
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

JOINTS = [f"L{i}_joint" for i in range(1, 8)]
TOL = 0.02
OLD_TOPIC = "/joint_group_effort_controller/joint_trajectory"
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")


class Recorder:
    def __init__(self, node):
        self.node = node
        self.samples = deque(maxlen=300)
        self.latest = None
        node.create_subscription(
            JointState, "/joint_states", self._cb, 20)

    def _cb(self, msg):
        snap = {n: p for n, p in zip(msg.name, msg.position)}
        self.latest = snap
        self.samples.append((time.monotonic(), snap))

    def current(self):
        if len(self.samples) < 5:
            return None
        tail = [s for _, s in list(self.samples)[-11:]]
        return {
            j: median(s[j] for s in tail if j in s)
            for j in JOINTS
        }


class StateWatcher:
    def __init__(self, node):
        self.state = ""
        node.create_subscription(ArmStatus, "/a3/arm_status", self._cb, 10)

    def _cb(self, msg):
        self.state = msg.state


def wait_state(target, timeout=8.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        spin(0.1)
        if watcher.state == target:
            return True
    return False


def spin(t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(cli, request, timeout=6.0):
    fut = cli.call_async(request)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    raise RuntimeError(f"service call timeout: {cli.srv_name}")


def wait_land(target, timeout=14.0):
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


def set_param(name, value):
    cli = node.create_client(
        SetParametersAtomically,
        "/a3_arm_controller/set_parameters_atomically")
    if not cli.wait_for_service(timeout_sec=3.0):
        raise RuntimeError("set_parameters service unavailable")
    req = SetParametersAtomically.Request()
    req.parameters = [ParameterMsg(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))]
    res = call(cli, req)
    if not res.result.successful:
        raise RuntimeError(f"set {name} failed: {res.result.reason}")


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
        shutil.copy(path, "/tmp/f74_latest_backup.yaml")
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


node = None
recorder = None
watcher = None


def main():
    global node, recorder, watcher
    if os.environ.get("ROS_DOMAIN_ID") != "60":
        print("WARN: ROS_DOMAIN_ID != 60, F70 mock 栈必须在同一 domain", file=sys.stderr)

    rclpy.init()
    node = rclpy.create_node("f74_acceptance")

    old_topic_count = 0

    def old_cb(_msg):
        nonlocal old_topic_count
        old_topic_count += 1

    node.create_subscription(JointTrajectory, OLD_TOPIC, old_cb, 10)

    reset_calls = 0

    def motor_cb(req, resp):
        nonlocal reset_calls
        if req.command == 2:
            reset_calls += 1
        resp.success = True
        resp.message = "mock ok"
        return resp

    node.create_service(MotorCommand, "/a3/motor/enable", motor_cb)
    node.create_service(MotorCommand, "/a3/motor/reset", motor_cb)

    recorder = Recorder(node)
    watcher = StateWatcher(node)

    jog_cli = node.create_client(SetJointPositions, "/a3/arm/set_joint_positions")
    goto_cli = node.create_client(GotoNamedPose, "/a3/arm/goto_named_pose")
    playback_cli = node.create_client(PlaybackTrajectory, "/a3/arm/playback")
    disable_cli = node.create_client(Trigger, "/a3/arm/disable")
    arm_enable_cli = node.create_client(Trigger, "/a3/arm/enable")
    init_cli = node.create_client(Trigger, "/a3/arm/init")

    print("waiting for /joint_states + services ...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 20:
        spin(0.1)
        if recorder.current() is not None and all(
                c.wait_for_service(timeout_sec=0.0)
                for c in (jog_cli, goto_cli, playback_cli, disable_cli,
                          arm_enable_cli)):
            break
    else:
        print("FATAL: stack not ready")
        return 1

    r = call(arm_enable_cli, Trigger.Request())
    if not r.success:
        r = call(init_cli, Trigger.Request())
        if r.success:
            r = call(arm_enable_cli, Trigger.Request())
    check("arm enable -> READY", r.success, r.message)
    if not r.success or not wait_state("READY", timeout=10.0):
        return 1

    poses = load_poses()
    home = poses["home"]
    ready = poses["ready"]

    # ---- A. jog（3 组含 L7）----
    # L2 物理限位 [0, 3.665]、L3 [-4.014, 0]（单向关节），目标必须在限位内
    jog_targets = [
        [0.20, 0.40, -0.45, -0.20, 0.15, 0.30, 0.25],
        [0.05, 0.10, -0.05, 0.10, -0.05, 0.05, 0.10],
        [-0.20, 0.30, -0.25, 0.20, -0.15, -0.30, 0.45],
    ]
    for k, tgt in enumerate(jog_targets):
        req = SetJointPositions.Request(positions=tgt, duration=2.8)
        r = call(jog_cli, req)
        if not r.success:
            check(f"A{k} jog dispatch", False, r.message)
            continue
        ok, err = wait_land(tgt)
        if not ok:
            cur = recorder.current()
            detail = "per_joint=" + ",".join(
                f"{JOINTS[i]}:{abs(cur[j]-tgt[i]):.3f}" for i, j in enumerate(JOINTS))
        else:
            detail = f"err={err:.4f}"
        check(f"A{k} jog land", ok, detail)
        if not wait_state("READY"):
            check(f"A{k} back to READY", False, f"state={watcher.state}")

    # ---- B. goto 线性兜底 ----
    set_param("goto_use_moveit", False)
    for k, name in enumerate(("ready", "home")):
        r = call(goto_cli, GotoNamedPose.Request(pose_name=name))
        if not r.success:
            check(f"B{k} goto {name} dispatch", False, r.message)
            continue
        ok, err = wait_land(poses[name])
        check(f"B{k} goto fallback {name}", ok, f"err={err:.4f}")
        if not wait_state("READY"):
            check(f"B{k} back to READY", False, f"state={watcher.state}")
    set_param("goto_use_moveit", True)

    # ---- C. playback（latest.yaml, ruckig retime）----
    path = write_latest(home)
    r = call(playback_cli, PlaybackTrajectory.Request(name=""))
    if not r.success:
        check("C playback dispatch", False, r.message)
    else:
        ok, err = wait_land(home, timeout=20.0)
        check("C playback retime+FJT land home", ok, f"err={err:.4f}")
        if not wait_state("READY", timeout=10.0):
            check("C back to READY", False, f"state={watcher.state}")
    if os.path.exists("/tmp/f74_latest_backup.yaml"):
        shutil.copy("/tmp/f74_latest_backup.yaml", path)

    # ---- D. preempt：P1 先发，0.4s 后 P2 ----
    p1 = [0.25, 0.35, -0.35, -0.25, 0.20, 0.25, 0.30]
    p2 = [-0.25, 0.30, -0.25, 0.25, -0.20, -0.25, 0.10]
    f1 = jog_cli.call_async(SetJointPositions.Request(positions=p1, duration=3.0))
    spin(0.4)
    f2 = jog_cli.call_async(SetJointPositions.Request(positions=p2, duration=3.0))
    r1ok = f1.result().success if f1.done() else False
    spin(0.4)
    r2ok = f2.result().success if f2.done() else False
    check("D1 both jog goals accepted", r1ok and r2ok, f"r1={r1ok} r2={r2ok}")
    spin(2.2)
    cur = recorder.current()
    d_p1 = max(abs(cur[j] - p1[i]) for i, j in enumerate(JOINTS))
    check("D2 P1 preempted (not tracking P1 at t~3)", d_p1 > 0.10,
          f"dist_to_p1={d_p1:.3f}")
    ok, err = wait_land(p2, timeout=8.0)
    check("D3 land P2", ok, f"err={err:.4f}")
    if not wait_state("READY"):
        check("D back to READY", False, f"state={watcher.state}")

    # ---- E. 旧话题零消息 ----
    spin(0.5)
    check("E zero messages on old traj topic", old_topic_count == 0,
          f"count={old_topic_count}")

    # ---- F. disable safe-park ----
    r = call(disable_cli, Trigger.Request())
    t0 = time.monotonic()
    parked = False
    while time.monotonic() - t0 < 15:
        spin(0.1)
        cur = recorder.current()
        err = max(abs(cur[j] - home[i]) for i, j in enumerate(JOINTS))
        if err < TOL and reset_calls >= 1:
            parked = True
            break
    check("F safe-park home + reset", parked,
          f"reset_calls={reset_calls}")

    total = len(RESULTS)
    passed = sum(1 for _, ok, _ in RESULTS if ok)
    print(f"\n==== F74 acceptance: {passed}/{total} ====")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
