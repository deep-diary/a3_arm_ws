#!/usr/bin/env python3
"""F72 vcan 数值验收：a3_hardware_interface/A3MITHardwareInterface 真机插件。

前置：
  1. python3 scripts/a3_test/vcan_motor_sim.py --interface vcan0
  2. ROS_DOMAIN_ID=59 ros2 launch a3_bringup edge_ros2_control_vcan.launch.py
用法：ROS_DOMAIN_ID=59 python3 f72_ros2_control_vcan_acceptance.py
退出码 0 = 全部验收项通过。

验收链：
  1. 直连官方 JTC home→ready→home + 夹爪开合（五次 S 曲线多点轨迹）
  2. move_group MoveGroup plan+execute（全程无 a3_fjt_action）
  3. CAN 侧（vcan0 独立抓包）：
     - 7 个 motor_id 均收到 type-1 控制帧
     - 指令电机位置 = direction*关节位置+offset（落稳后对比）
     - kp≈80 / kd≈2 / 速度指令=0 / torque_ff(id bits)=0
     - 反馈帧电机角 = direction*关节位置+offset（落稳后对比）
  4. 栈内无自研 FJT / can_bridge 节点
"""

import math
import os
import socket
import struct
import sys
import threading
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
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

DIRECTION = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0]
OFFSET = [0.0] * 7

GOAL_ERR = 0.02
START_END_V = 0.03
GOTO_S = 3.0
GRIP_S = 2.0
GRIP_AMP = 0.8
MOTION_THRESH = 0.01

V_LIM = [33.0, 33.0, 33.0, 50.0, 50.0, 50.0, 50.0]
A_LIM = [20.0, 20.0, 20.0, 30.0, 30.0, 30.0, 30.0]
JTC_V_SCALE = 0.1
MOVE_GROUP_V_SCALE = 0.3

V_HALF_T = 0.05
# CAN 反馈含 16 位量化（~0.0004 rad/步）+ JSB burst-pair 压缩，
# 加速度窗取 0.05s（F70 mock 用 0.025s 即可），否则起点尖峰误超 ~0.3。
A_HALF_T = 0.05
MIN_SPAN = 0.02
PRE_POST_T = 0.1

CAN_EFF_FLAG = 0x80000000
CAN_FORMAT = "=IB3x8s"
CMD_CONTROL = 0x01
CMD_FEEDBACK = 0x02
P_RANGE = 12.57
TORQUE_MAX = {i: (14.0 if i <= 3 else 6.0) for i in range(1, 8)}
SPEED_MAX = {i: (33.0 if i <= 3 else 50.0) for i in range(1, 8)}

CAN_POS_TOL = 0.01


def angdiff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def u16_to_float(raw, lo, hi):
    return lo + (raw / 65535.0) * (hi - lo)


def load_package_poses():
    path = "/home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml"
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    entries = data.get("poses", data)
    return {
        k: [float(v) for v in (val.get("positions") if isinstance(val, dict) else val)]
        for k, val in entries.items()
    }


class CanSniffer(threading.Thread):
    """独立 CAN_RAW socket 抓 vcan0，按 motor 记录最新控制/反馈帧解码值。"""

    def __init__(self, interface="vcan0"):
        super().__init__(daemon=True)
        self.interface = interface
        self.stop_ev = threading.Event()
        self.lock = threading.Lock()
        self.cmd = {}   # motor_id -> dict(t, pos, vel, kp, kd, t_ff)
        self.fb = {}    # motor_id -> dict(t, angle)
        self.sock = None

    def run(self):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((self.interface,))
        self.sock.settimeout(0.2)
        while not self.stop_ev.is_set():
            try:
                frame = self.sock.recv(72)
            except socket.timeout:
                continue
            except OSError:
                break
            can_id, _dlc, data = struct.unpack(CAN_FORMAT, frame)
            can_id &= 0x1FFFFFFF
            cmd_type = (can_id >> 24) & 0x1F
            # Control frames: motor_id low byte; feedback: motor_id bits 8-15 (low=master 0xFD)
            motor_id = ((can_id >> 8) & 0xFF
                        if cmd_type == CMD_FEEDBACK else can_id & 0xFF)
            if not (1 <= motor_id <= 7):
                continue
            now = time.monotonic()
            with self.lock:
                if cmd_type == CMD_CONTROL:
                    vmax = SPEED_MAX[motor_id]
                    tmax = TORQUE_MAX[motor_id]
                    self.cmd[motor_id] = {
                        "t": now,
                        "pos": u16_to_float((data[0] << 8) | data[1], -P_RANGE, P_RANGE),
                        "vel": u16_to_float((data[2] << 8) | data[3], -vmax, vmax),
                        "kp": u16_to_float((data[4] << 8) | data[5], 0.0, 500.0),
                        "kd": u16_to_float((data[6] << 8) | data[7], 0.0, 5.0),
                        "t_ff": u16_to_float((can_id >> 8) & 0xFFFF, -tmax, tmax),
                    }
                elif cmd_type == CMD_FEEDBACK:
                    self.fb[motor_id] = {
                        "t": now,
                        "angle": u16_to_float((data[0] << 8) | data[1], -P_RANGE, P_RANGE),
                    }

    def snapshot(self):
        with self.lock:
            return dict(self.cmd), dict(self.fb)

    def stop(self):
        self.stop_ev.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass


class Recorder(Node):
    def __init__(self):
        super().__init__("f72_acceptance")
        self.samples = []
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
        if not self.samples:
            return [0.0] * 7
        cand = [s[1] for s in self.samples[-200:]
                if any(abs(x) > 0.0 for x in s[1])]
        if not cand:
            cand = [self.samples[-1][1]]
        m = len(cand) // 2
        return [sorted(p[j] for p in cand)[m] for j in range(7)]


def find_nearest(samples, k, offset):
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


def send_move_group(node, target):
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
    sniffer = CanSniffer(os.environ.get("F72_CAN_IF", "vcan0"))
    try:
        sniffer.start()
    except OSError as e:
        print(f"vcan 抓包 socket 起不来：{e}（先起 vcan0 + 电机模拟器？）")
        return 1

    spin_s(rec, 1.0)

    poses = load_package_poses()
    home = poses["home"][:7]
    ready = poses["ready"][:7]
    start = rec.current()
    if max(abs(angdiff(start[j], home[j])) for j in range(7)) > 0.02:
        print(f"当前位 {start} 不是包内 home {home}，先核对栈/模拟器")
        return 1

    results = []

    t0 = time.monotonic()
    ok, msg = send_jtc(rec, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, home[:6], ready[:6], GOTO_S)
    results.append((ok, f"[JTC home→ready 发送] {msg}"))
    spin_s(rec, GOTO_S + 2.0)
    ok_a, msg_a = analyze(rec.window(t0, time.monotonic()), ready, JTC_V_SCALE, "JTC home→ready")
    results.append((ok and ok_a, msg_a))

    t0 = time.monotonic()
    ok, msg = send_jtc(rec, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, ready[:6], home[:6], GOTO_S)
    results.append((ok, f"[JTC ready→home 发送] {msg}"))
    spin_s(rec, GOTO_S + 2.0)
    ok_a, msg_a = analyze(rec.window(t0, time.monotonic()), home, JTC_V_SCALE, "JTC ready→home")
    results.append((ok and ok_a, msg_a))

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

    t0 = time.monotonic()
    ok, msg = send_move_group(rec, ready)
    results.append((ok, f"[move_group → ready] {msg}"))
    spin_s(rec, 2.0)
    ok_a, msg_a = analyze(rec.window(t0, time.monotonic()), ready, MOVE_GROUP_V_SCALE, "move_group → ready")
    results.append((ok and ok_a, msg_a))

    t0 = time.monotonic()
    ok, msg = send_move_group(rec, home)
    results.append((ok, f"[move_group → home] {msg}"))
    spin_s(rec, 2.0)
    ok_a, msg_a = analyze(rec.window(t0, time.monotonic()), home, MOVE_GROUP_V_SCALE, "move_group → home")
    results.append((ok and ok_a, msg_a))

    # CAN 侧校验：落稳 home 后抓最新帧（write() 每个控制周期持续发送）
    spin_s(rec, 0.5)
    settled = rec.current()
    cmd, fb = sniffer.snapshot()
    sniffer.stop()

    cmd_motors = sorted(cmd.keys())
    results.append((
        cmd_motors == list(range(1, 8)),
        f"[CAN 7 电机控制帧] 收到 motor_id={cmd_motors}",
    ))

    max_cmd_pos_err = 0.0
    max_fb_pos_err = 0.0
    max_vel = 0.0
    max_tff = 0.0
    kp_vals, kd_vals = {}, {}
    for motor in range(1, 8):
        j = motor - 1
        expect_motor = DIRECTION[j] * settled[j] + OFFSET[j]
        if motor in cmd:
            max_cmd_pos_err = max(max_cmd_pos_err, abs(cmd[motor]["pos"] - expect_motor))
            max_vel = max(max_vel, abs(cmd[motor]["vel"]))
            max_tff = max(max_tff, abs(cmd[motor]["t_ff"]))
            kp_vals[motor] = cmd[motor]["kp"]
            kd_vals[motor] = cmd[motor]["kd"]
        if motor in fb:
            max_fb_pos_err = max(max_fb_pos_err, abs(fb[motor]["angle"] - expect_motor))

    results.append((
        max_cmd_pos_err <= CAN_POS_TOL,
        f"[CAN 指令映射 motor=dir*joint+off] 最大偏差 {max_cmd_pos_err:.5f}",
    ))
    results.append((
        max_fb_pos_err <= CAN_POS_TOL,
        f"[CAN 反馈映射 motor=dir*joint+off] 最大偏差 {max_fb_pos_err:.5f}",
    ))
    results.append((
        max_vel <= 0.01,
        f"[CAN 速度指令=0] 最大 {max_vel:.5f}",
    ))
    results.append((
        max_tff <= 0.05,
        f"[CAN torque_ff=0] 最大 {max_tff:.5f}",
    ))
    kp_ok = all(abs(v - 80.0) <= 0.5 for v in kp_vals.values()) and len(kp_vals) == 7
    results.append((kp_ok, f"[CAN kp≈80] { {m: round(v, 2) for m, v in kp_vals.items()} }"))
    kd_ok = all(abs(v - 2.0) <= 0.05 for v in kd_vals.values()) and len(kd_vals) == 7
    results.append((kd_ok, f"[CAN kd≈2] { {m: round(v, 3) for m, v in kd_vals.items()} }"))

    node_names = rec.get_node_names()
    no_legacy = all(("fjt" not in n and "can_bridge" not in n
                     and "motor_protocol" not in n) for n in node_names)
    results.append((no_legacy, f"[无自研 FJT/can_bridge 节点] {sorted(node_names)}"))

    print("\n===== F72 验收结果 =====")
    all_ok = True
    for ok, msg in results:
        print(("PASS " if ok else "FAIL ") + msg)
        all_ok = all_ok and ok
    print("\n总体:", "ALL PASS" if all_ok else "HAS FAILURES")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
