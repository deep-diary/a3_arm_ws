#!/usr/bin/env python3
"""F70 仿真数值验收：ros2_control 标准栈（JTC/JSB/controller_manager + mock hw）。

前置：edge_ros2_control_sim.launch.py 已起（mock 初始位 = 包内 home）。
用法：ROS_DOMAIN_ID=<同栈> python3 f70_ros2_control_sim_acceptance.py
退出码 0 = 全部验收项通过。

验收链：
  1. 直连 /arm_controller/follow_joint_trajectory（官方 JTC）home→ready→home
  2. 直连 /gripper_controller/follow_joint_trajectory L7 开合
  3. move_group MoveGroup action plan+execute（全程无 a3_fjt_action）
  4. 进程/节点检查：栈内无自研 FJT action
"""

import math
import os
import sys
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
GRIPPER_JOINTS = ["L7_joint"]
ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS

GOAL_ERR = 0.02
START_END_V = 0.03
GOTO_S = 3.0
GRIP_S = 2.0
GRIP_AMP = 0.8
MOTION_THRESH = 0.01

# 关节限值（与 joint_limits.yaml / f67_f68 验收一致）
V_LIM = [33.0, 33.0, 33.0, 50.0, 50.0, 50.0, 50.0]
A_LIM = [20.0, 20.0, 20.0, 30.0, 30.0, 30.0, 30.0]
# 直连 JTC：固定时长 2–3 s 的位置样条，等效速度缩放约 0.1
JTC_V_SCALE = 0.1
# move_group：请求 max_velocity/acceleration_scaling_factor = 0.3
MOVE_GROUP_V_SCALE = 0.3

# 中心差分半窗（按真实时间，不用固定样本索引）：
# JSB 在轨迹激活瞬间会突发一串时间上高度压缩的陈旧样本对（burst pair，
# 20 个样本可挤在 ~4 ms 内），固定索引差分会把 0→全速的斜坡误判到 ~1 ms
# 跨距上，产生 ~60 rad/s² 的假加速度尖峰（LL-072）。
V_HALF_T = 0.05
A_HALF_T = 0.025
MIN_SPAN = 0.02
PRE_POST_T = 0.1


def angdiff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def load_package_poses():
    path = "/home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml"
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    entries = data.get("poses", data)
    return {
        k: [float(v) for v in (val.get("positions") if isinstance(val, dict) else val)]
        for k, val in entries.items()
    }


class Recorder(Node):
    def __init__(self):
        super().__init__("f70_acceptance")
        self.samples = []  # (t, pos[7], vel[7])
        self.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in ALL_JOINTS):
            return
        self.samples.append((
            time.monotonic(),
            [msg.position[idx[j]] for j in ALL_JOINTS],
            [msg.velocity[idx[j]] for j in ALL_JOINTS],
        ))

    def window(self, t0, t1):
        return [s for s in self.samples if t0 - 0.3 <= s[0] <= t1 + 0.6]

    def current(self):
        """近期样本逐关节中位数：新进程刚接入 DDS 时偶发全零陈旧样本，
        单取最后一帧会误判初始位（LL-072）。"""
        if not self.samples:
            return [0.0] * 7
        cand = [s[1] for s in self.samples[-200:]
                if any(abs(x) > 0.0 for x in s[1])]
        if not cand:
            cand = [self.samples[-1][1]]
        m = len(cand) // 2
        return [sorted(p[j] for p in cand)[m] for j in range(7)]


def find_nearest(samples, k, offset):
    """返回时间最接近 samples[k].t + offset 的样本索引。"""
    target = samples[k][0] + offset
    lo, hi = 0, len(samples) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if samples[mid][0] < target:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(samples[lo - 1][0] - target) < abs(samples[lo][0] - target):
        lo -= 1
    return lo


def smooth_velocities(samples):
    """位置中心差分估速度（±V_HALF_T 真实时间窗）：不依赖 /joint_states 的
    velocity 字段，且对 burst-pair 时间压缩样本免疫（LL-072）。"""
    n = len(samples)
    sv = [[0.0] * 7 for _ in range(n)]
    for k in range(n):
        lo = find_nearest(samples, k, -V_HALF_T)
        hi = find_nearest(samples, k, V_HALF_T)
        dt = samples[hi][0] - samples[lo][0]
        if dt >= MIN_SPAN:
            for j in range(7):
                sv[k][j] = angdiff(samples[hi][1][j], samples[lo][1][j]) / dt
    return sv


def analyze(samples, targets, v_scale, label):
    """targets: [7] 目标位置。返回 (ok, msg)。"""
    span = samples[-1][0] - samples[0][0]
    if span < 0.5 or len(samples) < 30:
        return False, f"{label}: 采样不足 ({len(samples)} 个 / {span:.2f}s)"
    sv = smooth_velocities(samples)
    moving = [i for i in range(len(samples))
              if any(abs(v) > MOTION_THRESH for v in sv[i])]
    if not moving:
        return False, f"{label}: 未检测到运动"
    i0, i1 = moving[0], moving[-1]
    dur = samples[i1][0] - samples[i0][0]

    pre_i = find_nearest(samples, i0, -PRE_POST_T)
    post_i = find_nearest(samples, i1, PRE_POST_T)
    v_start = max(abs(v) for v in sv[pre_i])
    v_end = max(abs(v) for v in sv[post_i])

    vmax = [0.0] * 7
    for k in range(i0, i1 + 1):
        for j in range(7):
            vmax[j] = max(vmax[j], abs(sv[k][j]))
    amax = [0.0] * 7
    for k in range(i0, i1 + 1):
        lo = find_nearest(samples, k, -A_HALF_T)
        hi = find_nearest(samples, k, A_HALF_T)
        dt = samples[hi][0] - samples[lo][0]
        if dt >= MIN_SPAN:
            for j in range(7):
                amax[j] = max(amax[j], abs((sv[hi][j] - sv[lo][j]) / dt))

    v_over = [ALL_JOINTS[j] for j in range(7) if vmax[j] > V_LIM[j] * v_scale + 0.5]
    a_over = [ALL_JOINTS[j] for j in range(7)
              if amax[j] > A_LIM[j] * v_scale * 1.5 + 0.5]

    settled = samples[post_i][1]
    err = max(abs(angdiff(settled[j], targets[j])) for j in range(7))

    ok = (v_start <= START_END_V and v_end <= START_END_V
          and not v_over and not a_over and err <= GOAL_ERR)
    msg = (f"{label}: 窗口 {dur:.2f}s，起止速度 {v_start:.3f}/{v_end:.3f}，"
           f"vmax={max(vmax):.3f}，amax={max(amax):.3f}，目标误差 {err:.4f}，"
           f"超速={v_over or '无'} 超加速度={a_over or '无'}")
    return ok, msg


def send_jtc(node, action_name, joint_names, start, positions, duration_s):
    """下发五次 S 曲线（31 点，含位置/速度）：JTC 单点轨迹只做匀速线性
    插值，激活瞬间存在速度阶跃；工业用法是发送完整时间参数化多点轨迹
    （move_group/TOTG 即如此）。start/positions 等长于 joint_names。"""
    cli = ActionClient(node, FollowJointTrajectory, action_name)
    if not cli.wait_for_server(timeout_sec=5.0):
        return False, "action server 不可用"
    traj = JointTrajectory(joint_names=joint_names)
    n_pts = 31
    for i in range(1, n_pts + 1):
        u = i / n_pts
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        ds = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration_s
        traj.points.append(JointTrajectoryPoint(
            positions=[start[j] + (positions[j] - start[j]) * s
                       for j in range(len(joint_names))],
            velocities=[(positions[j] - start[j]) * ds
                        for j in range(len(joint_names))],
            time_from_start=rclpy.duration.Duration(
                seconds=duration_s * u).to_msg(),
        ))
    goal = FollowJointTrajectory.Goal(trajectory=traj)
    goal.goal_time_tolerance = rclpy.duration.Duration(seconds=0.0).to_msg()
    gh_future = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, gh_future, timeout_sec=10)
    gh = gh_future.result()
    if not gh.accepted:
        return False, "goal 被拒绝"
    result_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=duration_s + 15)
    res = result_future.result().result
    if res.error_code != 0:
        return False, f"JTC error_code={res.error_code} {res.error_string}"
    return True, f"{duration_s:.1f}s"


def send_move_group(node, target, label):
    cli = ActionClient(node, MoveGroup, "/move_action")
    if not cli.wait_for_server(timeout_sec=10.0):
        return False, "move_action 不可用"
    req = MotionPlanRequest()
    req.group_name = "arm"
    req.num_planning_attempts = 10
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.3
    req.goal_constraints.append(Constraints(joint_constraints=[
        JointConstraint(
            joint_name=ARM_JOINTS[j],
            position=target[j],
            tolerance_above=0.01,
            tolerance_below=0.01,
            weight=1.0,
        )
        for j in range(6)
    ]))
    goal = MoveGroup.Goal(
        request=req,
        planning_options=PlanningOptions(plan_only=False, look_around=False),
    )
    gh_future = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, gh_future, timeout_sec=10)
    gh = gh_future.result()
    if not gh.accepted:
        return False, "move_group goal 被拒绝"
    result_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=40)
    res = result_future.result().result
    if res.error_code.val != 1:
        return False, f"MoveIt error_code={res.error_code.val}"
    pts = len(res.planned_trajectory.joint_trajectory.points)
    return True, f"{pts} pts"


def spin_s(node, t):
    t0 = time.monotonic()
    while time.monotonic() - t0 < t:
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    rclpy.init()
    rec = Recorder()
    spin_s(rec, 1.0)

    poses = load_package_poses()
    home = poses["home"][:7]
    ready = poses["ready"][:7]
    start = rec.current()
    if max(abs(angdiff(start[j], home[j])) for j in range(7)) > 0.02:
        print(f"mock 初始位 {start} 不是包内 home {home}，先核对栈")
        return 1

    results = []

    # 1a. 直连 arm JTC：home → ready（五次 S 曲线多点轨迹）
    t0 = time.monotonic()
    ok, msg = send_jtc(rec, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, home[:6], ready[:6], GOTO_S)
    results.append((ok, f"[JTC home→ready 发送] {msg}"))
    spin_s(rec, GOTO_S + 2.0)
    ok_a, msg_a = analyze(rec.window(t0, time.monotonic()), ready, JTC_V_SCALE, "JTC home→ready")
    results.append((ok and ok_a, msg_a))

    # 1b. 直连 arm JTC：ready → home
    t0 = time.monotonic()
    ok, msg = send_jtc(rec, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, ready[:6], home[:6], GOTO_S)
    results.append((ok, f"[JTC ready→home 发送] {msg}"))
    spin_s(rec, GOTO_S + 2.0)
    ok_a, msg_a = analyze(rec.window(t0, time.monotonic()), home, JTC_V_SCALE, "JTC ready→home")
    results.append((ok and ok_a, msg_a))

    # 2. 直连 gripper JTC：0 → 0.8 → 0
    t0 = time.monotonic()
    ok, msg = send_jtc(rec, "/gripper_controller/follow_joint_trajectory",
                       GRIPPER_JOINTS, [home[6]], [GRIP_AMP], GRIP_S)
    results.append((ok, f"[JTC 夹爪张开 发送] {msg}"))
    spin_s(rec, GRIP_S + 1.0)
    open_target = home[:6] + [GRIP_AMP]
    ok_a, msg_a = analyze(rec.window(t0, time.monotonic()), open_target, JTC_V_SCALE, "JTC 夹爪张开")
    results.append((ok and ok_a, msg_a))

    t0 = time.monotonic()
    ok, msg = send_jtc(rec, "/gripper_controller/follow_joint_trajectory",
                       GRIPPER_JOINTS, [GRIP_AMP], [home[6]], GRIP_S)
    results.append((ok, f"[JTC 夹爪闭合 发送] {msg}"))
    spin_s(rec, GRIP_S + 1.0)
    ok_a, msg_a = analyze(rec.window(t0, time.monotonic()), home, JTC_V_SCALE, "JTC 夹爪闭合")
    results.append((ok and ok_a, msg_a))

    # 3a. move_group plan+execute → ready
    t0 = time.monotonic()
    ok, msg = send_move_group(rec, ready, "ready")
    results.append((ok, f"[move_group → ready] {msg}"))
    spin_s(rec, 2.0)
    ok_a, msg_a = analyze(rec.window(t0, time.monotonic()), ready, MOVE_GROUP_V_SCALE, "move_group → ready")
    results.append((ok and ok_a, msg_a))

    # 3b. move_group → home（验收结束停在 home）
    t0 = time.monotonic()
    ok, msg = send_move_group(rec, home, "home")
    results.append((ok, f"[move_group → home] {msg}"))
    spin_s(rec, 2.0)
    ok_a, msg_a = analyze(rec.window(t0, time.monotonic()), home, MOVE_GROUP_V_SCALE, "move_group → home")
    results.append((ok and ok_a, msg_a))

    # 4. 栈内节点/进程无 a3_fjt_action
    node_names = rec.get_node_names()
    no_fjt = all("fjt" not in n for n in node_names)
    results.append((no_fjt, f"[无自研 FJT 节点] 节点列表: {sorted(node_names)}"))

    print("\n===== F70 验收结果 =====")
    all_ok = True
    for ok, msg in results:
        print(("PASS " if ok else "FAIL ") + msg)
        all_ok = all_ok and ok
    print("\n总体:", "ALL PASS" if all_ok else "HAS FAILURES")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
