#!/usr/bin/env python3
"""F116 仿真验收：/a3/arm/random_pose_tour 随机点位 MoveIt 巡游。

前置：edge_teleop_full_sim.launch.py use_moveit:=true 已起（独立 ROS_DOMAIN_ID）。
用法：python3 f116_random_tour_sim_acceptance.py
退出码 0 = 全部验收项通过。

验收点：
  1. count=5/seed=0：success、序列长度 5、相邻不重、不含 zero、total_duration_s>0，
     /joint_states 按序到每个目标点（前 6 关节 0.02 rad）；
  2. seed=123 两次序列完全相同（可复现）；
  3. count=2 覆盖默认 5；
  4. DISABLED 态被拒；
  5. 点位池 <2（exclude 参数）返回 success=false。
"""

import math
import os
import sys
import time

import rclpy
import yaml
from rcl_interfaces.msg import (
    Parameter as ParameterMsg,
    ParameterType,
    ParameterValue,
)
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from a3_msgs.msg import ArmStatus
from a3_msgs.srv import RandomPoseTour

JOINTS = [f"L{i}_joint" for i in range(1, 8)]
GOAL_ERR = 0.02


def angdiff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def load_poses():
    """读包内 named_poses.yaml（与 arm_controller 包内层一致）。"""
    path = "/home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml"
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    entries = data.get("poses")
    if not isinstance(entries, dict):
        entries = data
    poses = {}
    for key, val in entries.items():
        positions = val.get("positions") if isinstance(val, dict) else val
        if isinstance(positions, list) and len(positions) >= 6:
            poses[key] = [float(v) for v in positions]
    return poses


class Recorder(Node):
    def __init__(self):
        super().__init__("f116_acceptance")
        self.samples = []  # (t, pos[7])
        self.arm_state = "?"
        self.create_subscription(
            JointState, "/joint_states", self._js_cb, qos_profile_sensor_data)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._arm_cb, 10)

    def _js_cb(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS):
            return
        self.samples.append((
            time.monotonic(),
            [msg.position[idx[j]] for j in JOINTS],
        ))

    def _arm_cb(self, msg: ArmStatus):
        self.arm_state = msg.state

    def window(self, t0, t1):
        return [s for s in self.samples if t0 - 0.05 <= s[0] <= t1 + 0.3]


def call(node, srv_type, name, request, timeout=180.0):
    cli = node.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"service not found: {name}")
    future = cli.call_async(request)
    t0 = time.monotonic()
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - t0 > timeout:
            raise RuntimeError(f"service timeout: {name}")
    return future.result()


def spin_s(node, t):
    t0 = time.monotonic()
    while time.monotonic() - t0 < t:
        rclpy.spin_once(node, timeout_sec=0.1)


def wait_state(node, want, timeout=8.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.arm_state == want:
            return True
    return False


def set_exclude_param(node, names):
    req = SetParameters.Request(parameters=[ParameterMsg(
        name="random_tour_exclude_poses",
        value=ParameterValue(
            type=ParameterType.PARAMETER_STRING_ARRAY,
            string_array_value=list(names),
        ),
    )])
    result = call(node, SetParameters,
                  "/a3_arm_controller/set_parameters", req, timeout=10.0)
    return bool(result.results[0].successful)


def verify_ordered_visits(samples, sequence, poses, label):
    """序列每个目标点按顺序在运动采样中出现（贪心有序匹配，前 6 关节 GOAL_ERR）。

    返回 (通过, 明细)。
    """
    msgs = []
    ok_all = True
    si = 0
    for k, name in enumerate(sequence):
        target = poses[name][:6]
        match_i = None
        err_at = None
        for i in range(si, len(samples)):
            err = max(abs(angdiff(samples[i][1][j], target[j])) for j in range(6))
            if err <= GOAL_ERR:
                match_i = i
                err_at = err
                break
        if match_i is None:
            # 取后续窗口内最接近的误差做诊断
            tail = samples[si:]
            best = min(
                (max(abs(angdiff(s[1][j], target[j])) for j in range(6))
                 for s in tail),
                default=9.9,
            )
            msgs.append(f"leg{k + 1} {name}: 未到点（最佳误差 {best:.3f}）")
            ok_all = False
            break
        msgs.append(f"leg{k + 1} {name}: 到点误差 {err_at:.4f}")
        si = match_i + 1
    return ok_all, f"[{label}] " + "; ".join(msgs)


def validate_sequence(seq, count, exclude, label):
    msgs = []
    ok = True
    if len(seq) != count:
        ok = False
        msgs.append(f"长度 {len(seq)} != {count}")
    if any(a == b for a, b in zip(seq, seq[1:])):
        ok = False
        msgs.append("存在相邻重复")
    bad = [n for n in seq if n in exclude]
    if bad:
        ok = False
        msgs.append(f"含排除点 {bad}")
    msgs.insert(0, f"sequence={list(seq)}")
    return ok, f"[{label}] " + "; ".join(msgs)


def main():
    rclpy.init()
    node = Recorder()
    results = []

    try:
        poses = load_poses()
    except OSError as e:
        print(f"load poses failed: {e}")
        return 1
    required = {"idle", "ready", "home", "home_up", "home_down",
                "home_front", "home_back"}
    missing = required - set(poses)
    if missing:
        print(f"named_poses.yaml 缺少 {missing}; got {sorted(poses)}")
        return 1
    print(f"pose pool: {sorted(poses)}")

    r = call(node, Trigger, "/a3/arm/enable", Trigger.Request())
    if not r.success:
        print(f"enable failed: {r.message}")
        return 1
    print(f"enable: {r.message}")
    spin_s(node, 3.0)

    # ---- 1. count=5 / seed=0 真随机，按序到点 ----
    t0 = time.monotonic()
    r = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
             RandomPoseTour.Request(count=5, seed=0))
    t1 = time.monotonic()
    print(f"tour(5, seed=0): success={r.success} {r.message}")
    win = node.window(t0, t1)
    results.append((r.success, f"[tour1 success] {r.message}"))
    ok, msg = validate_sequence(r.sequence, 5, {"zero"}, "tour1 序列")
    results.append((ok, msg))
    dur_ok = r.total_duration_s > 0.0
    results.append((dur_ok, f"[tour1 duration] total={r.total_duration_s:.2f}s"))
    if r.success:
        ok, msg = verify_ordered_visits(win, list(r.sequence), poses, "tour1 到点")
        results.append((ok, msg))
    results.append((wait_state(node, "READY", 8.0),
                    f"[tour1 后回 READY] state={node.arm_state}"))
    spin_s(node, 1.0)

    # ---- 2. seed=123 可复现（两次相同）----
    r2a = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
               RandomPoseTour.Request(count=5, seed=123))
    print(f"tour(5, seed=123)#1: {list(r2a.sequence)}")
    wait_state(node, "READY", 8.0)
    spin_s(node, 1.0)
    r2b = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
               RandomPoseTour.Request(count=5, seed=123))
    print(f"tour(5, seed=123)#2: {list(r2b.sequence)}")
    wait_state(node, "READY", 8.0)
    same = r2a.success and r2b.success and list(r2a.sequence) == list(r2b.sequence)
    results.append((same,
                    f"[seed 可复现] #1={list(r2a.sequence)} #2={list(r2b.sequence)}"))
    if r2a.success:
        ok, msg = validate_sequence(r2a.sequence, 5, {"zero"}, "seed123 序列")
        results.append((ok, msg))
    spin_s(node, 1.0)

    # ---- 3. count=2 覆盖默认 ----
    r3 = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
              RandomPoseTour.Request(count=2, seed=7))
    print(f"tour(2, seed=7): {list(r3.sequence)}")
    results.append((r3.success and len(r3.sequence) == 2,
                    f"[count=2 覆盖] len={len(r3.sequence)} {r3.message}"))
    wait_state(node, "READY", 8.0)
    spin_s(node, 1.0)

    # ---- 4. DISABLED 态拒绝 ----
    rd = call(node, Trigger, "/a3/arm/disable", Trigger.Request(), timeout=40.0)
    print(f"disable: {rd.message}")
    spin_s(node, 3.0)
    r4 = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
              RandomPoseTour.Request(count=3, seed=1))
    print(f"tour while disabled: success={r4.success} msg={r4.message}")
    results.append((not r4.success,
                    f"[DISABLED 拒绝] success={r4.success} '{r4.message}'"))

    # 重新使能后再测池过小（需要 READY 才能区分失败原因）
    call(node, Trigger, "/a3/arm/enable", Trigger.Request())
    spin_s(node, 3.0)

    # ---- 5. 池 <2：只留 idle ----
    keep = {"idle"}
    exclude_all = sorted(set(poses) - keep)
    param_ok = set_exclude_param(node, exclude_all)
    r5 = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
              RandomPoseTour.Request(count=3, seed=1))
    print(f"tour pool<2: param_ok={param_ok} success={r5.success} msg={r5.message}")
    results.append((param_ok and not r5.success and "pool" in r5.message,
                    f"[池<2 拒绝] param={param_ok} '{r5.message}'"))
    set_exclude_param(node, ["zero"])

    print("\n===== F116 验收结果 =====")
    all_ok = True
    for ok, msg in results:
        print(("PASS " if ok else "FAIL ") + msg)
        all_ok = all_ok and ok
    print("\n总体:", "ALL PASS" if all_ok else "HAS FAILURES")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
