#!/usr/bin/env python3
"""F126 acceptance: 密集示教点 → 单条平滑 retime 产物（F68 Ruckig）。

edge_web_sim（DOMAIN 62）自起栈：use_gripper/use_moveit=true，use_servo/rviz=false。
F68 `_call_retime` 把密集录制点经 /a3/arm/retime_trajectory 重定时成一条整体平滑轨迹；
edge_web_sim 用 control_backend="topic"（默认）→ `_dispatch_trajectory` 把 retime 产物
（7 关节）整条单消息 publish 到 /joint_group_effort_controller/joint_trajectory（QoS 10）
→ sim_motor_node 执行。本脚本订阅该话题，playback 期间以
`len(joint_names)>=7 and len(points)>=3` 唯一挑出 retime 产物（phase-R MoveIt 是 6 关节、
L7 补发是 1 关节两点轨迹，均被过滤）。

序列：
  1. enable → READY；off_idle（FJT L2→0.4）→ mode IDLE 稳定。
  2. start_teach → TEACH；三关节旋转扫点：L2/L3/L4 各 +0.05×6（hold_s=1.4 > 0.4s×3 的
     单关节复推周期，每推续锁 → 各自单调阶梯，大空间扫点模拟手拖）；stop_teach 自动存档。
  3. 断言 latest.yaml：7 关节、points ≥ 40（密集）、L2/L3/L4 几何张幅 ≥ 0.25。
  4. goto idle 改构型 → READY（首点 L2=0.4 ≠ idle → use_ramp → phase R 回首点）。
  5. 清缓冲 → /a3/arm/playback "" → 断言：
     a. resp 含 "retime ["（走 F68 链路）与 "return=MoveIt"（回首点经 move_group）；
        status 消息序「playback return 」先于「playback 」。
     b. 话题侧 7J≥3pt 产物恰好 1 条（单条），points ≥ 40，时间单调不减。
     c. 首/末点与 latest.yaml 首/末点径迹重合（tol 0.01 / 0.05）。
     d. 相邻点差有界：逐关节 max|Δq| ≤ 0.12 rad；且 max v ≤ 2.0×录制最大瞬时 v + 0.1
        （相对监听，重定时不引入高于原始推档时刻的尖峰）。
  6. back_to_ready → READY；断言 joint_states 收敛回录制末点（tol 0.06）——单条平滑轨迹
     实际把臂带到终点。

Exit code 0 = all checks passed.
"""

import argparse
import os
import signal
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import Parameter as RclParam
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from a3_msgs.msg import ArmStatus
from a3_msgs.srv import GotoNamedPose
from a3_msgs.srv import PlaybackTrajectory

WS = "/home/cat/a3_arm_ws"
STACK_LOG = "/tmp/f126_accept_stack.log"
TRAJ_TOPIC = "/joint_group_effort_controller/joint_trajectory"
RESULTS = []

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
ALL_JOINTS = [f"L{i}_joint" for i in range(1, 8)]
STATE_READY = "READY"
STATE_TEACH = "TEACH"
MODE_IDLE = "IDLE"

# F126 扫点幅度（与 f124 对齐到 0.7 量级，规避 F107 静态力矩门禁边界）。
# 方向按 URDF 限位定：L2 [−2.79,2.79]、L3 [−4.01,**0**]（负向域，idle≈0.0002，必须向负扫）、
# L4 [−1.05,1.57]（idle≈0.3351）。L3 若反向会出上限 → F107 门禁拒发（URDF L3 upper=0）。
SWEEP_JOINTS = [1, 2, 3]         # L2/L3/L4 走旋转扫点
SWEEP_STEP = {1: 0.05, 2: -0.05, 3: 0.05}   # 每轮每关节推量（L3 负向）
ROUNDS = 6
PUSH_INTERVAL_S = 0.4
HOLD_S = 1.4                      # > 单关节复推周期 0.4×3=1.2s → 每推续锁续单调

# 产物连续界（℃）——绝对单步界 + 相对录制尖峰界
MAX_STEP_RAD = 0.12


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
            self.hard_kill()

    def hard_kill(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc.wait(timeout=10)


def spin(node, t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(node, cli, request, timeout=15.0):
    fut = cli.call_async(request)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    raise RuntimeError(f"service timeout: {cli.srv_name}")


def _v_stats(pts, key_pos, key_t=None):
    """逐关节有限差分速度（rad/s），返回 (max_v, n_valid)。跳过 dt≈0 的相邻对。"""
    max_v = 0.0
    n_valid = 0
    for i in range(1, len(pts)):
        t0 = key_t[i - 1] if key_t is not None else 0.0
        t1 = key_t[i] if key_t is not None else 0.0
        dt = t1 - t0
        if dt is None or float(dt) <= 1e-6:
            continue
        n_valid += 1
        for k in range(len(key_pos(pts[i]))):
            dq = abs(float(key_pos(pts[i])[k]) - float(key_pos(pts[i - 1])[k]))
            if dq / float(dt) > max_v:
                max_v = dq / float(dt)
    return (max_v, n_valid)


class Harness(Node):
    def __init__(self):
        super().__init__("f126_acceptance")
        self.js = None
        self.status = None
        self.status_samples = []
        self.traj_msgs = []          # 仅收 len(joint_names)>=7 and len(points)>=3
        self.traj_all = 0
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_status, 10)
        self.create_subscription(
            JointTrajectory, TRAJ_TOPIC, self._on_traj, 10)
        self.en = self.create_client(Trigger, "/a3/arm/enable")
        self.start_teach = self.create_client(Trigger, "/a3/arm/start_teach")
        self.stop_teach = self.create_client(Trigger, "/a3/arm/stop_teach")
        self.goto_cli = self.create_client(
            GotoNamedPose, "/a3/arm/goto_named_pose")
        self.playback_cli = self.create_client(
            PlaybackTrajectory, "/a3/arm/playback")
        self.fsm_set_cli = self.create_client(
            SetParametersAtomically, "/a3_arm_controller/set_parameters_atomically")
        self.fsm_get_cli = self.create_client(
            GetParameters, "/a3_arm_controller/get_parameters")
        self.push_param_cli = self.create_client(
            SetParametersAtomically,
            "/motor_protocol_node/set_parameters_atomically")
        self.push_trig = self.create_client(Trigger, "/a3/motor/sim_push")
        self.fjt = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

    def _on_js(self, msg):
        self.js = msg

    def _on_status(self, msg):
        self.status = msg
        self.status_samples.append((time.monotonic(), msg.state, msg.mode,
                                    msg.message))

    def _on_traj(self, msg):
        self.traj_all += 1
        if len(msg.joint_names) >= 7 and len(msg.points) >= 3:
            self.traj_msgs.append(msg)

    def joint_pos(self, name):
        js = self.js
        if js is None:
            return None
        return float(js.position[js.name.index(name)])

    def wait_status(self, timeout=30.0):
        end = time.monotonic() + timeout
        while self.status is None and time.monotonic() < end:
            spin(self, 0.05)
        return self.status

    def wait_state(self, state, timeout=30.0):
        self.wait_status()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            spin(self, 0.05)
            if self.status.state == state:
                return True
        return self.status.state == state

    def wait_mode(self, mode, timeout=15.0):
        self.wait_status()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            spin(self, 0.05)
            if self.status.mode == mode:
                return True
        return self.status.mode == mode

    def wait_mode_stable(self, mode, hold_s=1.0, timeout=8.0):
        """等 mode 连续稳定 hold_s（跨过 gravity 名义 end_time 尾巴，见 f119 注释）。"""
        self.wait_status()
        stable_since = None
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            spin(self, 0.05)
            if self.status.mode == mode:
                stable_since = stable_since if stable_since is not None else time.monotonic()
                if time.monotonic() - stable_since >= hold_s:
                    return True
            else:
                stable_since = None
        return self.status.mode == mode

    def enable(self):
        assert self.en.wait_for_service(timeout_sec=20)
        resp = call(self, self.en, Trigger.Request(), timeout=40)
        if not resp.success:
            raise RuntimeError(f"enable rejected: {resp.message}")
        spin(self, 3.0)
        # 使能后 0.5s：让 arm_monitor 的使能沿重基准先完成再发轨迹（LL-132/133，f119 同款）
        spin(self, 0.5)

    def fjt_l2_040(self, duration_s=3.0, result_wait_s=6.0):
        q0 = [self.joint_pos(n) for n in ARM_JOINTS]
        if any(v is None for v in q0):
            raise RuntimeError("no /joint_states for FJT")

        def dur(s):
            return Duration(sec=int(s), nanosec=int((s % 1) * 1e9))

        end = list(q0)
        end[1] = 0.4
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=q0, velocities=[0.0] * 6,
                                 accelerations=[0.0] * 6, time_from_start=dur(0.0)),
            JointTrajectoryPoint(positions=end, velocities=[0.0] * 6,
                                 accelerations=[0.0] * 6, time_from_start=dur(duration_s)),
        ]
        gh = self.fjt.send_goal_async(goal)
        t0 = time.monotonic()
        while not gh.done() and time.monotonic() - t0 < 8:
            spin(self, 0.05)
        if not gh.done() or not gh.result().accepted:
            return False, None, None
        rf = gh.result().get_result_async()
        t0 = time.monotonic()
        while not rf.done() and time.monotonic() - t0 < result_wait_s:
            spin(self, 0.05)
        if not rf.done():
            return True, None, None
        return True, time.monotonic() - t0, rf.result().result.error_code

    def off_idle(self):
        acc, _, code = self.fjt_l2_040()
        if not acc or code != 0:
            raise RuntimeError(f"off-idle FJT failed acc={acc} code={code}")
        if not self.wait_mode_stable(MODE_IDLE, hold_s=1.0, timeout=8.0):
            raise RuntimeError(
                f"mode not IDLE-stable after off-idle (mode={self.status.mode})")

    def set_push(self, rad7, hold_s):
        req = SetParametersAtomically.Request()
        req.parameters = [
            RclParam(name="sim_push_enabled",
                     value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                          bool_value=True)),
            RclParam(name="sim_push_rad",
                     value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                          double_array_value=[float(v) for v in rad7])),
            RclParam(name="sim_push_hold_s",
                     value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                          double_value=hold_s)),
        ]
        resp = call(self, self.push_param_cli, req, timeout=10)
        if not resp.result.successful:
            raise RuntimeError(f"push param rejected: {resp.result.reason}")

    def push_once(self, sleep_s=0.45):
        resp = call(self, self.push_trig, Trigger.Request(), timeout=10)
        if not resp.success:
            raise RuntimeError(f"sim_push rejected: {resp.message}")
        spin(self, sleep_s)

    def goto_idle(self):
        assert self.goto_cli.wait_for_service(timeout_sec=10)
        resp = call(self, self.goto_cli,
                    GotoNamedPose.Request(pose_name="idle"), timeout=40)
        if not resp.success:
            raise RuntimeError(f"goto idle rejected: {resp.message}")
        if not self.wait_state(STATE_READY, timeout=60):
            raise RuntimeError(f"not READY after goto idle (state={self.status.state})")


def wait_future(h, fut, status_timeout=90.0):
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < status_timeout:
        spin(h, 0.05)
    done = fut.done()
    resp = fut.result() if done else None
    spin(h, 1.5)   # 冲掉 service 响应抵达后仍 DDS 在途的 retime 产物消息
    return done, resp, list(h.status_samples)


def messages(samples):
    return [s[3] for s in samples if s[3]]


def read_latest_yaml():
    import yaml
    path = os.path.expanduser("~/.a3/trajectories/latest.yaml")
    with open(path) as f:
        return yaml.safe_load(f), path


def pos_of(pt):
    # latest.yaml 的 point 是 dict（positions）；FJT 产物是 JointTrajectoryPoint msg
    return list(pt.positions) if hasattr(pt, "positions") else list(pt["positions"])


def tfs_sec(t):
    return float(t.sec) + float(t.nanosec) * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("domain", nargs="?", default="62",
                    help="罗斯 DOMAIN_ID（edge_web_sim 自起于该域）")
    args = ap.parse_args()

    os.environ["ROS_DOMAIN_ID"] = args.domain
    os.environ["PYTHONNOUSERSITE"] = "1"
    env = dict(os.environ)

    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "edge_web_sim.launch.py",
         "use_gripper:=true", "use_rviz:=false", "use_moveit:=true",
         "use_servo:=false", "use_servo_anchor:=false"],
        STACK_LOG, env=env)

    rclpy.init()
    h = Harness()

    try:
        assert h.en.wait_for_service(timeout_sec=60) and \
            h.playback_cli.wait_for_service(timeout_sec=60) and \
            h.fsm_get_cli.wait_for_service(timeout_sec=60) and \
            h.push_trig.wait_for_service(timeout_sec=60) and \
            h.fjt.wait_for_server(timeout_sec=60), \
            "core services absent (edge_web_sim failed to boot?)"

        print("\n==== 1. enable -> READY; off_idle -> L2=0.4 ====", flush=True)
        h.enable()
        if not h.wait_state(STATE_READY, timeout=15):
            raise RuntimeError("arm not READY")
        h.off_idle()
        check("off-idle 后 mode IDLE 稳定", h.status.mode == MODE_IDLE, "")
        if not h.wait_state(STATE_READY, timeout=15):
            raise RuntimeError(f"state not READY after off_idle (state={h.status.state})")

        print("\n==== 2. start_teach + 三关节旋转扫点录制 ====", flush=True)
        resp = call(h, h.start_teach, Trigger.Request(), timeout=10)
        check("start_teach success", resp.success, f"msg={resp.message}")
        if not h.wait_state(STATE_TEACH, timeout=5.0):
            raise RuntimeError("state did not reach TEACH")
        l2_0 = h.joint_pos("L2_joint")
        check("录制起点 L2 来自 off-idle(≈0.4)", l2_0 is not None and abs(l2_0 - 0.4) < 0.05,
              f"L2={l2_0}")
        # hold_s=1.4s > 单关节复推周期 1.2s（3 关节 ×0.4s）→ 每推续锁，各关节单调阶梯。
        # 6 轮 → L2:0.4→0.7，L3:0.00→−0.30（负向，URDF upper=0），L4:0.335→0.635：
        # 三关节大空间扫点（模拟手拖），方向各按其限位向内。
        h.set_push([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], hold_s=HOLD_S)
        for _ in range(ROUNDS):
            for idx in SWEEP_JOINTS:
                rad7 = [0.0] * 7
                rad7[idx] = SWEEP_STEP[idx]
                h.set_push(rad7, hold_s=HOLD_S)
                h.push_once(sleep_s=PUSH_INTERVAL_S)
        resp = call(h, h.stop_teach, Trigger.Request(), timeout=15)
        check("stop_teach success + 自动存档",
              resp.success and "auto-saved latest.yaml" in resp.message,
              f"msg={resp.message}")
        if not h.wait_state(STATE_READY, timeout=10):
            raise RuntimeError("state did not return READY after stop_teach")

        print("\n==== 3. latest.yaml 契约（密集 + 大空间扫点）====", flush=True)
        data, path = read_latest_yaml()
        jn = list(data.get("joint_names", []))
        pts = list(data.get("points", []))
        check("latest.yaml 含 7 关节 (含 L7_joint)",
              jn == ALL_JOINTS, f"joint_names={jn}")
        check("密集：采样点 ≥ 40",
              len(pts) >= 40, f"n={len(pts)}")
        p0, p1 = pos_of(pts[0]), pos_of(pts[-1])
        spans = {idx: abs(p1[idx] - p0[idx]) for idx in SWEEP_JOINTS}
        check("大空间扫点：L2/L3/L4 张幅均 ≥ 0.25",
              all(spans[idx] >= 0.25 for idx in SWEEP_JOINTS),
              f"spans={ {f'L{i+1}_joint': round(spans[i], 3) for i in SWEEP_JOINTS} }")
        check("L2 单调推进（sim_push 生效，首 0.4 → 末 0.7）",
              p0[1] < 0.6 and abs(p1[1] - 0.7) < 0.05,
              f"first_L2={p0[1]:.3f} last_L2={p1[1]:.3f}")
        check("扫点方向按限位向内：L3 向负（−0.3）、L4 向正（0.335→0.635）",
              p1[2] < p0[2] - 0.2 and p1[3] > p0[3] + 0.2,
              f"L3 {p0[2]:.3f}→{p1[2]:.3f}  L4 {p0[3]:.3f}→{p1[3]:.3f}")

        print("\n==== 4. goto idle 改构型（首点 L2=0.4 ≠ idle）====", flush=True)
        h.goto_idle()
        check("goto idle 后 READY", h.status.state == STATE_READY, "")
        l2 = h.joint_pos("L2_joint")
        check("当前 L2 离开录制首点（ramp_dist>0.02 → use_ramp）",
              l2 is not None and abs(l2 - 0.4) > 0.02, f"L2={l2}")

        print("\n==== 5. playback -> 单条平滑 retime 产物 ====", flush=True)
        h.status_samples, h.traj_msgs = [], []
        done, resp, samples = wait_future(
            h, h.playback_cli.call_async(PlaybackTrajectory.Request(name="")))
        check("playback service returned", done, "")
        if done:
            msgs = messages(samples)
            r_idx = next((i for i, m in enumerate(msgs)
                          if m.startswith("playback return")), None)
            p_idx = next((i for i, m in enumerate(msgs)
                          if m.startswith("playback ")
                          and not m.startswith("playback return")), None)
            check("状态消息序「playback return 」先于「playback 」",
                  r_idx is not None and p_idx is not None and r_idx < p_idx,
                  f"r_idx={r_idx} p_idx={p_idx} msgs={msgs[-4:]}")
            check("playback success=True", resp.success, f"msg={resp.message}")
            check("resp 含 retime 标记（F68 链路生效）",
                  "retime [" in resp.message, resp.message)
            check("resp 含 return=MoveIt（回首点经 move_group）",
                  "return=MoveIt" in resp.message, resp.message)

            prod = list(h.traj_msgs)
            check(f"retime 产物恰好 1 条（单条下发；共见 {h.traj_all} 条话题消息）",
                  len(prod) == 1, f"n_product={len(prod)}")
            if prod:
                pr = prod[-1]
                t = [tfs_sec(p.time_from_start) for p in pr.points]
                check("产物关节序 = L1..L7", list(pr.joint_names) == ALL_JOINTS,
                      f"joint_names={list(pr.joint_names)}")
                check("产物密集（points ≥ 40）", len(pr.points) >= 40,
                      f"n={len(pr.points)}")
                check("产物时间单调",
                      all(t[i] >= t[i - 1] for i in range(1, len(t))) and t[-1] > t[0],
                      f"t0={t[0]:.3f} tN={t[-1]:.3f}")
                d0 = max(abs(a - b) for a, b in zip(pos_of(pr.points[0]), p0))
                dN = max(abs(a - b) for a, b in zip(pos_of(pr.points[-1]), p1))
                check("首点径迹重合录制首点 (≤0.01)", d0 <= 0.01, f"max|Δ|={d0:.4f}")
                check("末点径迹重合录制末点 (≤0.05)", dN <= 0.05, f"max|Δ|={dN:.4f}")

                max_step = max(
                    abs(float(pr.points[i].positions[k]) - float(pr.points[i - 1].positions[k]))
                    for i in range(1, len(pr.points))
                    for k in range(len(pr.joint_names)))
                check(f"相邻点差有界：逐关节 max|Δq| ≤ {MAX_STEP_RAD}",
                      max_step <= MAX_STEP_RAD, f"max_step={max_step:.4f}")

            # 速度相对界：产物最大瞬时 v 不应高于录制原始推档尖峰（×2 余量 + 0.1 绝对）
            v_prod, n_prod = _v_stats(pr.points, pos_of, key_t=t) if prod else (None, 0)
            rec_t = [float(p["time_from_start_sec"]) for p in pts]
            v_raw, n_raw = _v_stats(pts, pos_of, key_t=rec_t)
            if prod and n_prod and n_raw:
                bound = 2.0 * v_raw + 0.1
                ok = v_prod <= bound
                check("速度连续：产物 max v ≤ 2×录制 max v + 0.1（无新尖峰）",
                      ok, f"v_prod={v_prod:.3f} v_raw={v_raw:.3f} bound={bound:.3f}")
            elif prod:
                check("速度连续：产物有限差分有有效段", n_prod >= 2, f"n_valid={n_prod}")
        if not h.wait_state(STATE_READY, timeout=120):
            raise RuntimeError("not READY after playback (back_to_ready)")

        print("\n==== 6. back_to_ready 后收敛回录制末点 ====", flush=True)
        check("playback 结束后回 READY（back_to_ready）",
              h.status.state == STATE_READY, f"state={h.status.state}")
        js_pos = [h.joint_pos(n) for n in ALL_JOINTS]
        d_end = max(abs(a - b) for a, b in zip(js_pos, p1)) if all(
            v is not None for v in js_pos) else float("inf")
        check("joint_states 收敛回录制末点 (≤0.06)",
              all(v is not None for v in js_pos) and d_end <= 0.06,
              f"max|Δ(末点)|={d_end:.4f} pos={[round(v, 3) if v is not None else None for v in js_pos]}")

    except Exception:
        import traceback
        traceback.print_exc()
        with open(STACK_LOG, errors="replace") as f:
            txt = f.read()
        print("\n---- tail stack log ----\n", txt[-3000:], flush=True)
    finally:
        if stack is not None and stack.proc.poll() is None:
            stack.hard_kill()
        h.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F126 dense-smooth acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())