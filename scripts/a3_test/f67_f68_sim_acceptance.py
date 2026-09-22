#!/usr/bin/env python3
"""F67/F68 仿真数值验收：goto(move_group+TOTG) / safe-park / playback(Ruckig)。

前置：edge_web_sim.launch.py use_moveit:=true（可同时 use_servo:=true）已起。
用法：python3 f67_f68_sim_acceptance.py
退出码 0 = 全部验收项通过。

位姿期望值不硬编码：~/.a3/poses.yaml（真机标定覆盖层）优先，缺省回退包内
named_poses.yaml —— 与 arm_controller 的两层位姿解析保持一致。
"""

import math
import os
import re
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import (
    Parameter as ParameterMsg,
    ParameterType,
    ParameterValue,
)
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from a3_msgs.srv import GotoNamedPose, PlaybackTrajectory

JOINTS = [f"L{i}_joint" for i in range(1, 8)]
# joint_limits.yaml 满速
V_LIM = [33.0, 33.0, 33.0, 50.0, 50.0, 50.0, 50.0]
A_LIM = [20.0, 20.0, 20.0, 30.0, 30.0, 30.0, 30.0]
START_END_V = 0.02
GOAL_ERR = 0.02
GOTO_V_SCALE = 0.3
PLAYBACK_NAME = "f68_acceptance"
PLAYBACK_RECORDED_S = 4.0
MOTION_THRESH = 0.015


def angdiff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def smoothstep(x):
    return x * x * (3.0 - 2.0 * x)


def load_poses():
    """包内默认 + ~/.a3 覆盖（后者优先），与 arm_controller._load_poses 同序。"""
    poses = {}
    for path in (
        "/home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml",
        os.path.expanduser("~/.a3/poses.yaml"),
    ):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except OSError:
            continue
        entries = data.get("poses")
        if not isinstance(entries, dict):
            entries = data
        for key, val in entries.items():
            positions = val.get("positions") if isinstance(val, dict) else val
            if isinstance(positions, list) and len(positions) >= 6:
                poses[key] = [float(v) for v in positions]
    return poses


def write_test_trajectory(home):
    """生成 50 Hz / 4s 稠密移动轨迹：home → A（2s smoothstep）→ B（2s smoothstep）。
    末态 B ≠ home，便于验收后续 safe-park 必须真运动；首点 = home，验收零位移
    ramp 跳过逻辑（重复点曾导致 Ruckig 失败）。"""
    amp_a = [0.30, 0.35, -0.50, 0.25, 0.35, 0.20, 0.0]
    amp_b = [0.20, 0.15, -0.30, 0.10, 0.20, 0.10, 0.0]
    dt = 0.02
    points = []
    n = int(PLAYBACK_RECORDED_S / dt) + 1
    for i in range(n):
        t = i * dt
        if t <= 2.0:
            s = smoothstep(t / 2.0)
            q = [home[j] + s * amp_a[j] for j in range(7)]
        else:
            s = smoothstep((t - 2.0) / 2.0)
            q = [home[j] + amp_a[j] + s * (amp_b[j] - amp_a[j]) for j in range(7)]
        points.append({"time_from_start_sec": round(t, 4), "positions": q})
    data = {"joint_names": JOINTS, "points": points}
    path = os.path.expanduser(f"~/.a3/trajectories/{PLAYBACK_NAME}.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f)
    return [p["positions"] for p in points]


class Recorder(Node):
    def __init__(self):
        super().__init__("f67f68_acceptance")
        self.samples = []  # (t, pos[7], vel[7])
        self.create_subscription(JointState, "/joint_states", self._cb, qos_profile_sensor_data)

    def _cb(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS):
            return
        self.samples.append((
            time.monotonic(),
            [msg.position[idx[j]] for j in JOINTS],
            [msg.velocity[idx[j]] for j in JOINTS],
        ))

    def window(self, t0, t1):
        return [s for s in self.samples if t0 - 0.05 <= s[0] <= t1 + 0.3]

    def current(self):
        if not self.samples:
            return [0.0] * 7
        return list(self.samples[-1][1])


def call(node, srv_type, name, request, timeout=30.0):
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


def set_param(node, name, value):
    result = call(
        node, SetParameters, "/a3_arm_controller/set_parameters",
        SetParameters.Request(parameters=[ParameterMsg(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value),
        )]),
    )
    return bool(result.results[0].successful)


def analyze(samples, targets, v_scale, label, allow_already_there=False):
    """返回 (通过?, 指标字符串, 运动窗口时长s)。targets: 目标位置[7] 或 None。"""
    if len(samples) < 5:
        return False, f"{label}: 采样数不足 ({len(samples)})", 0.0

    moving = [i for i, s in enumerate(samples)
              if any(abs(v) > MOTION_THRESH for v in s[2])]
    if not moving:
        if allow_already_there and targets is not None:
            err = max(abs(angdiff(samples[-1][1][j], targets[j])) for j in range(6))
            if err <= GOAL_ERR:
                return True, f"[{label}] 已在目标位（误差 {err:.4f} rad），无运动", 0.0
        return False, f"{label}: 未检测到运动", 0.0
    i0, i1 = moving[0], moving[-1]
    motion_dur = samples[i1][0] - samples[i0][0]

    msgs = [f"运动窗口 {motion_dur:.2f}s"]
    ok = True

    pre = samples[max(0, i0 - 1)]
    post = samples[min(len(samples) - 1, i1 + 1)]
    v_start = max(abs(v) for v in pre[2])
    v_end = max(abs(v) for v in post[2])
    if v_start > START_END_V:
        ok = False
    if v_end > START_END_V:
        ok = False
    msgs.append(f"起止速度 {v_start:.3f}/{v_end:.3f} rad/s")

    vmax = [0.0] * 7
    for k in range(i0, i1 + 1):
        for j in range(7):
            vmax[j] = max(vmax[j], abs(samples[k][2][j]))
    # sim 一阶跟随产生单帧速度阶跃，五点平滑后再差分估加速度
    sv = []
    for k in range(len(samples)):
        lo, hi = max(0, k - 2), min(len(samples), k + 3)
        sv.append([
            sum(samples[m][2][j] for m in range(lo, hi)) / (hi - lo)
            for j in range(7)
        ])
    amax = [0.0] * 7
    for k in range(max(1, i0), min(len(samples), i1 + 2)):
        dt = samples[k][0] - samples[k - 1][0]
        if dt > 1e-3:
            for j in range(7):
                amax[j] = max(amax[j], abs((sv[k][j] - sv[k - 1][j]) / dt))
    v_over = [JOINTS[j] for j in range(7) if vmax[j] > V_LIM[j] * v_scale + 0.5]
    a_over = [JOINTS[j] for j in range(7) if amax[j] > A_LIM[j] * v_scale * 1.5 + 0.5]
    if v_over or a_over:
        ok = False
    msgs.append(f"vmax={[round(v, 2) for v in vmax]}")
    msgs.append(f"超速关节={v_over or '无'} 超加速度关节={a_over or '无'}")

    if targets is not None:
        settled = samples[min(len(samples) - 1, i1 + 4)][1]
        err = max(abs(angdiff(settled[j], targets[j])) for j in range(6))
        if err > GOAL_ERR:
            ok = False
        msgs.append(f"目标误差 {err:.4f} rad")

    return ok, f"[{label}] " + "; ".join(msgs), motion_dur


def check_geometry(rec, t0, t1, recorded_pts, label, skip_s=0.0):
    """回放路径几何保持：录制点按路径顺序在实际采样帧中贪心邻域匹配。
    skip_s 跳过回放起始 ramp（legacy 链从末态先回到录制起点）。"""
    samples = [s for s in rec.window(t0, t1) if s[0] >= t0 + skip_s]
    if not samples:
        return False, f"{label}: 无采样"
    sparse = recorded_pts[::2]
    max_dev = 0.0
    si = 0
    used = 0
    for pt in sparse:
        best = None
        for k in range(si, min(si + 60, len(samples))):
            d = max(abs(angdiff(samples[k][1][j], pt[j])) for j in range(7))
            if best is None or d < best[0]:
                best = (d, k)
        if best and best[0] < 0.15:
            max_dev = max(max_dev, best[0])
            si = best[1]
            used += 1
    ok = used >= len(sparse) * 0.8 and max_dev <= 0.03
    return ok, f"[{label}] 几何最大偏差 {max_dev:.4f} rad，匹配 {used}/{len(sparse)}"


def parse_duration_s(message):
    m = re.search(r"([0-9]+\.[0-9]+)s", message)
    return float(m.group(1)) if m else 0.0


def main():
    rclpy.init()
    rec = Recorder()

    def spin_s(t):
        t0 = time.monotonic()
        while time.monotonic() - t0 < t:
            rclpy.spin_once(rec, timeout_sec=0.1)

    poses = load_poses()
    if "home" not in poses or "ready" not in poses:
        print(f"pose resolution failed: {sorted(poses)}")
        return 1
    home = poses["home"][:7] + [0.0] * max(0, 7 - len(poses["home"]))
    ready = poses["ready"][:7] + [0.0] * max(0, 7 - len(poses["ready"]))
    recorded_pts = write_test_trajectory(home)
    end_b = recorded_pts[-1]

    results = []

    r = call(rec, Trigger, "/a3/arm/enable", Trigger.Request())
    if not r.success:
        print(f"enable failed: {r.message}")
        return 1
    print(f"enable: {r.message}")
    spin_s(3.0)

    # Triangle: goto ready（move_group + TOTG）
    t0 = time.monotonic()
    r = call(rec, GotoNamedPose, "/a3/arm/goto_named_pose",
             GotoNamedPose.Request(pose_name="ready"))
    print(f"goto ready: {r.message}")
    if "move_group" not in r.message:
        results.append((False, "[goto ready] 未走 move_group 路径"))
    spin_s(parse_duration_s(r.message) + 2.0)
    results.append(analyze(rec.window(t0, time.monotonic()), ready,
                           GOTO_V_SCALE, "goto ready")[:2])

    # Circle: goto home（move_group + TOTG）
    t0 = time.monotonic()
    r = call(rec, GotoNamedPose, "/a3/arm/goto_named_pose",
             GotoNamedPose.Request(pose_name="home"))
    print(f"goto home: {r.message}")
    if "move_group" not in r.message:
        results.append((False, "[goto home] 未走 move_group 路径"))
    spin_s(parse_duration_s(r.message) + 2.0)
    results.append(analyze(rec.window(t0, time.monotonic()), home,
                           GOTO_V_SCALE, "goto home")[:2])

    # F67 降级链：goto_use_moveit=false → 线性兜底；再恢复走 move_group
    set_param(rec, "goto_use_moveit", False)
    t0 = time.monotonic()
    r = call(rec, GotoNamedPose, "/a3/arm/goto_named_pose",
             GotoNamedPose.Request(pose_name="ready"))
    print(f"goto ready(fallback): {r.message}")
    fb_ok = "linear fallback" in r.message
    results.append((fb_ok, f"[goto fallback] {r.message}"))
    spin_s(parse_duration_s(r.message) + 2.0)
    set_param(rec, "goto_use_moveit", True)
    t0 = time.monotonic()
    r = call(rec, GotoNamedPose, "/a3/arm/goto_named_pose",
             GotoNamedPose.Request(pose_name="home"))
    print(f"goto home(restore): {r.message}")
    restore_ok = "move_group" in r.message
    results.append((restore_ok, f"[goto restore] {r.message}"))
    spin_s(parse_duration_s(r.message) + 2.0)

    # F68: playback（Ruckig，逼近录制时长）
    t0 = time.monotonic()
    r = call(rec, PlaybackTrajectory, "/a3/arm/playback",
             PlaybackTrajectory.Request(name=PLAYBACK_NAME))
    print(f"playback: {r.message}")
    backend_ok = "retime [ruckig" in r.message
    results.append((backend_ok, f"[playback backend] {r.message}"))
    dur = parse_duration_s(r.message)
    spin_s(dur + 3.0)
    win = rec.window(t0, time.monotonic())
    ok, msg, motion_dur = analyze(win, end_b, 1.0, "playback ruckig")
    results.append((ok, msg))
    # 实际运动窗口逼近录制 4s（±25%）
    dur_ok = abs(motion_dur - PLAYBACK_RECORDED_S) <= PLAYBACK_RECORDED_S * 0.25
    results.append((dur_ok, f"[playback duration] 指令 {dur:.2f}s / 实测 {motion_dur:.2f}s"
                            f"（录制 {PLAYBACK_RECORDED_S:.1f}s）"))
    results.append(check_geometry(rec, t0, time.monotonic(), recorded_pts,
                                  "playback geometry"))

    # F68 降级链：playback_retime=false → 旧 smooth/time-warp 链
    set_param(rec, "playback_retime", False)
    t0 = time.monotonic()
    r = call(rec, PlaybackTrajectory, "/a3/arm/playback",
             PlaybackTrajectory.Request(name=PLAYBACK_NAME))
    print(f"playback(legacy): {r.message}")
    legacy_ok = "legacy chain" in r.message
    results.append((legacy_ok, f"[playback legacy] {r.message}"))
    dur = parse_duration_s(r.message)
    spin_s(max(dur, PLAYBACK_RECORDED_S) + 3.0)
    win = rec.window(t0, time.monotonic())
    ok, msg, _ = analyze(win, end_b, 1.0, "playback legacy")
    results.append((ok, msg))
    results.append(check_geometry(rec, t0, time.monotonic(), recorded_pts,
                                  "playback legacy geometry", skip_s=2.5))
    set_param(rec, "playback_retime", True)

    # R3: disable = safe park（move_group 回 home）→ 失能；此时臂在 B，必有运动
    t0 = time.monotonic()
    r = call(rec, Trigger, "/a3/arm/disable", Trigger.Request(), timeout=40.0)
    print(f"disable(safe park): {r.message}")
    park_msg_ok = "disabled" in r.message
    results.append((park_msg_ok, f"[safe park result] {r.message}"))
    spin_s(3.0)
    ok, msg, _ = analyze(rec.window(t0, time.monotonic()), home, GOTO_V_SCALE,
                         "safe park -> home", allow_already_there=True)
    results.append((ok and park_msg_ok, msg))

    print("\n===== 验收结果 =====")
    all_ok = True
    for ok, msg in results:
        print(("PASS " if ok else "FAIL ") + msg)
        all_ok = all_ok and ok
    print("\n总体:", "ALL PASS" if all_ok else "HAS FAILURES")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
