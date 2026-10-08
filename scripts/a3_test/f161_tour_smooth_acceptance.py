#!/usr/bin/env python3
"""F161 仿真验收：巡游固定点位 + 平滑链 A/B。

前置：edge_teleop_full_sim.launch.py use_moveit:=true 已起（独立 ROS_DOMAIN_ID）。
用法：python3 f161_tour_smooth_acceptance.py
退出码 0 = 全部验收项通过。

验收点：
  1. fixed_sequence 未知点位 → success=false；
  2. fixed_sequence 两次调用返回相同 sequence 并按序到点（前 6 关节 0.02 rad）；
  3. 平滑度 A/B：同一条固定序列分别 use_smooth=false(blend) / =true(样条+retime)，
     新链 j_rms 相对 blend 下降 ≥ 一个数量级、a_rms 下降。
"""

import math
import os
import re
import sys
import time

import rclpy
import yaml
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from a3_msgs.msg import ArmStatus
from a3_msgs.srv import RandomPoseTour

JOINTS = [f"L{i}_joint" for i in range(1, 8)]
GOAL_ERR = 0.02
# 固定验收序列（3 via 点 → 当前位 + 3 = 4 点 → 三次样条 C2）。
# 选彼此笛卡尔分离的位，避免 pilz blend「Blend radius too large」回退逐腿；
# 缺 triangle/square 故用现有位。
FIXED = ["home_front", "home_back", "home_down"]


def angdiff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def load_poses():
    path = "/home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml"
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    entries = data.get("poses") or {}
    poses = {}
    for key, val in entries.items():
        positions = val.get("positions") if isinstance(val, dict) else val
        if isinstance(positions, list) and len(positions) >= 6:
            poses[key] = [float(v) for v in positions]
    return poses


class Recorder(Node):
    def __init__(self):
        super().__init__("f161_acceptance")
        self.samples = []  # (t, pos[7])
        self.arm_state = "?"
        self.create_subscription(
            JointState, "/joint_states", self._js_cb, qos_profile_sensor_data)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._arm_cb, 10)

    def _js_cb(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS):
            return
        self.samples.append((time.monotonic(), [msg.position[idx[j]] for j in JOINTS]))

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


def set_bool_param(node, name, value):
    req = SetParameters.Request(parameters=[ParameterMsg(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value)),
    )])
    result = call(node, SetParameters, "/a3_arm_controller/set_parameters", req, timeout=10.0)
    return bool(result.results[0].successful)


def verify_ordered_visits(samples, sequence, poses, label):
    """序列每个目标点按顺序在运动采样中出现（贪心有序匹配，前 6 关节 GOAL_ERR）。"""
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
            tail = samples[si:]
            best = min(
                (max(abs(angdiff(s[1][j], target[j])) for j in range(6)) for s in tail),
                default=9.9,
            )
            msgs.append(f"leg{k + 1} {name}: 未到点（最佳误差 {best:.3f}）")
            ok_all = False
            break
        msgs.append(f"leg{k + 1} {name}: 到点误差 {err_at:.4f}")
        si = match_i + 1
    return ok_all, f"[{label}] " + "; ".join(msgs)


def parse_metrics(message):
    """从服务响应 message 解析命令轨迹的 j_rms/a_rms（控制器在命令轨迹上算，噪声自由）。"""
    j = re.search(r"j_rms=([0-9.eE+-]+)", message)
    a = re.search(r"a_rms=([0-9.eE+-]+)", message)
    if not j or not a:
        return None
    return {"a_rms": float(a.group(1)), "j_rms": float(j.group(1))}


def main():
    rclpy.init()
    node = Recorder()
    results = []

    try:
        poses = load_poses()
    except OSError as e:
        print(f"load poses failed: {e}")
        return 1
    missing = set(FIXED) - set(poses)
    if missing:
        print(f"named_poses.yaml 缺少 {missing}; got {sorted(poses)}")
        return 1
    print(f"fixed sequence: {FIXED}")

    r = call(node, Trigger, "/a3/arm/enable", Trigger.Request())
    if not r.success:
        print(f"enable failed: {r.message}")
        return 1
    print(f"enable: {r.message}")
    spin_s(node, 3.0)

    # ---- 1. fixed_sequence 未知点位拒绝 ----
    r = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
             RandomPoseTour.Request(fixed_sequence=["home", "bogus_pose"]))
    print(f"fixed(unknown): success={r.success} msg={r.message}")
    results.append((not r.success and "unknown" in r.message,
                    f"[fixed 未知点位拒绝] success={r.success} '{r.message}'"))

    # ---- 2. fixed_sequence 两次一致 + 按序到点 ----
    t0 = time.monotonic()
    r1 = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
              RandomPoseTour.Request(fixed_sequence=FIXED))
    t1 = time.monotonic()
    print(f"fixed#1: success={r1.success} {r1.message}")
    win = node.window(t0, t1)
    results.append((r1.success and list(r1.sequence) == FIXED,
                    f"[fixed 序列一致] seq={list(r1.sequence)}"))
    if r1.success:
        ok, msg = verify_ordered_visits(win, list(r1.sequence), poses, "fixed#1 到点")
        results.append((ok, msg))
    wait_state(node, "READY", 10.0)
    spin_s(node, 1.0)

    r2 = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
              RandomPoseTour.Request(fixed_sequence=FIXED))
    print(f"fixed#2: success={r2.success} {r2.message}")
    results.append((r2.success and list(r2.sequence) == list(r1.sequence) == FIXED,
                    f"[fixed 两次一致] #1={list(r1.sequence)} #2={list(r2.sequence)}"))
    wait_state(node, "READY", 10.0)
    spin_s(node, 1.0)

    # ---- 3. 平滑度 A/B：blend vs smooth ----
    # blend（use_smooth=false）
    if not set_bool_param(node, "random_tour_use_smooth", False):
        print("set use_smooth=false failed")
        return 1
    spin_s(node, 0.5)
    rb = None
    for _attempt in range(3):  # OMPL/blend 偶发失败，重试
        rb = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
                  RandomPoseTour.Request(fixed_sequence=FIXED))
        wait_state(node, "READY", 60.0)
        mb = parse_metrics(rb.message)
        if rb.success and mb is not None:
            break
        spin_s(node, 1.0)
    mb = parse_metrics(rb.message) if rb is not None else None
    print(f"blend tour: success={rb.success} {rb.message}")
    spin_s(node, 1.0)

    # smooth（use_smooth=true）
    if not set_bool_param(node, "random_tour_use_smooth", True):
        print("set use_smooth=true failed")
        return 1
    spin_s(node, 0.5)
    rs = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
              RandomPoseTour.Request(fixed_sequence=FIXED))
    wait_state(node, "READY", 60.0)  # 平滑链异步下发，服务立即返回，等运动完成回 READY
    ms = parse_metrics(rs.message)
    print(f"smooth tour: success={rs.success} {rs.message}")
    spin_s(node, 1.0)

    if rb.success and rs.success and mb is not None and ms is not None:
        j_drop = mb["j_rms"] / ms["j_rms"] if ms["j_rms"] > 1e-12 else float("inf")
        a_drop = mb["a_rms"] / ms["a_rms"] if ms["a_rms"] > 1e-12 else float("inf")
        results.append((j_drop >= 10.0,
                        f"[j_rms A/B] blend={mb['j_rms']:.3g} smooth={ms['j_rms']:.3g} "
                        f"drop=x{j_drop:.1f}"))
        results.append((ms["a_rms"] < mb["a_rms"],
                        f"[a_rms A/B] blend={mb['a_rms']:.3g} smooth={ms['a_rms']:.3g} "
                        f"drop=x{a_drop:.1f}"))
    else:
        results.append((False, f"[A/B 未完成] blend={rb.success} smooth={rs.success}"))

    set_bool_param(node, "random_tour_use_smooth", False)

    print("\n===== F161 验收结果 =====")
    all_ok = True
    for ok, msg in results:
        print(("PASS " if ok else "FAIL ") + msg)
        all_ok = all_ok and ok
    print("\n总体:", "ALL PASS" if all_ok else "HAS FAILURES")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
