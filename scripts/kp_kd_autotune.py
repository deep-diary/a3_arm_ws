#!/usr/bin/env python3
"""F138 — MIT 位置环 kp/kd 伺服层自动整定（逐关节，安全自循环）。

在 home 基位附近小幅摆动各关节（L1 严格限幅、L2/L3 为主），用单点 FJT（JTC spline
快速斜坡 + 落定）激励位置环，采 /joint_states 计算：
  落定误差、超调、ringing(RMS)、峰值速度、力矩纹波
以坐标下降（粗→细）逐关节搜最优 (kp, kd)，全程带安全护栏：
  超速 / 超力矩 / 越限 / fault → 立即中止并回落名义增益。

增益经 /a3_hardware_health 的运行时参数 kp_<joint>/kd_<joint> 下发（F138 P0 落地），
无需重启 controller_manager。整定结果写 ~/.a3/kp_kd_tuned.yaml。

模式：
  --mode connect  附着当前运行栈（默认；机械臂已上电、栈已在跑时用）
  --mode real     自起 a3_bringup.launch.py hardware:=can
  --mode mock     自起 mock 栈（GenericSystem，无 /a3_hardware_health，仅演示流程）

示例：
  python3 scripts/kp_kd_autotune.py --mode connect
  python3 scripts/kp_kd_autotune.py --mode connect --joints L2,L3
  python3 scripts/kp_kd_autotune.py --mode connect --dry-run
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

WS_DIR = "/home/cat/a3_arm_ws"
ARM_JOINTS = ["L1_joint", "L2_joint", "L3_joint",
              "L4_joint", "L5_joint", "L6_joint"]

# F138 逐关节激励幅值（rad，相对当前基位）。幅值带符号 = 摆动方向，须朝关节行程
# 内部摆、避开贴限位的边界。home 半抬位下：L2≈0.785 向正(抬臂)空间充裕；L3≈-0.785
# 向负(折前臂)空间充裕。L1 用户确认 ±0.2 rad（±12°）无干涉。腕部 ±1.57 内任意方向皆可。
DEFAULT_AMP = {
    "L1_joint": 0.20,
    "L2_joint": 0.20,
    "L3_joint": -0.20,
    "L4_joint": 0.15,
    "L5_joint": -0.15,
    "L6_joint": 0.15,
}

# 名义增益（与 xacro 起始一致，也是安全回落目标）。
NOMINAL_KP = 80.0
NOMINAL_KD = 2.0

# URDF 关节限位（rad，源自 el_a3_ros2_control.xacro command_interface min/max）。
# 激励目标须落在 [min+margin, max-margin] 内，避免贴边/越界。
JOINT_LIMITS = {
    "L1_joint": (-2.79253, 2.79253),
    "L2_joint": (0.0, 3.66519),
    "L3_joint": (-4.01426, 0.0),
    "L4_joint": (-1.5708, 1.5708),
    "L5_joint": (-1.5708, 1.5708),
    "L6_joint": (-1.5708, 1.5708),
}
LIMIT_MARGIN = 0.05

# 扫参范围与坐标下降网格（粗→细）。
KP_MIN, KP_MAX = 40.0, 150.0
KD_MIN, KD_MAX = 0.5, 5.0
KP_COARSE = [40.0, 60.0, 80.0, 100.0, 120.0, 135.0, 150.0]
KD_COARSE = [0.5, 1.5, 2.5, 4.0, 5.0]

MOVE_DT = 0.8          # 单点 FJT 斜坡时长（快斜坡暴露超调/ringing）
SETTLE_S = 0.6         # 落定观察窗
GOAL_TOL = 1.0         # FJT goal_time_tolerance (s)

# 安全护栏阈值。
VEL_ABORT = 3.0        # rad/s，超此视为发散
EFF_ABORT = 9.0        # Nm，超此视为近峰值/发散（低于 RS00 14 / EL05 6 峰值）
DEV_ABORT = 0.6        # rad，关节偏离基位超过此值视为失控（远大于激励幅值）
MAX_ITER = 200         # 全局最大评估次数兜底


def ensure_env(mode: str) -> None:
    """Re-exec under a3_shell_env.sh so rclpy/ament packages are importable."""
    if os.environ.get("A3_F138_ENVED") == "1":
        return
    out = subprocess.run(
        ["bash", "-c",
         f"source {WS_DIR}/scripts/a3_shell_env.sh >/dev/null 2>&1 && env"],
        capture_output=True, text=True, check=False,
    )
    env = {}
    for line in out.stdout.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            env[k] = v
    if "ROS_DOMAIN_ID" not in env and "ROS_DOMAIN_ID" in os.environ:
        env["ROS_DOMAIN_ID"] = os.environ["ROS_DOMAIN_ID"]
    if mode == "mock":
        env["ROS_DOMAIN_ID"] = "106"
    env["A3_F138_ENVED"] = "1"
    os.execvpe(sys.executable,
               [sys.executable, os.path.abspath(__file__), *sys.argv[1:]], env)


def wait_future(fut, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < end:
        time.sleep(0.02)
    return fut.done()


class Recorder:
    """采集 /joint_states 并做实时安全护栏（超速/超力矩/越限）。"""

    def __init__(self, node, base_pos: Dict[str, float],
                 vel_abort: float = VEL_ABORT, eff_abort: float = EFF_ABORT,
                 dev_abort: float = DEV_ABORT):
        self._node = node
        self._lock = threading.Lock()
        self.samples: List[Tuple[float, Dict[str, float], Dict[str, float],
                                 Dict[str, float]]] = []
        self.base = base_pos
        self.vel_abort = vel_abort
        self.eff_abort = eff_abort
        self.dev_abort = dev_abort
        self.abort_reason = ""
        from sensor_msgs.msg import JointState
        node.create_subscription(
            JointState, "/joint_states", self._cb, 20)

    def _cb(self, msg):
        t = time.monotonic()
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        eff = dict(zip(msg.name, msg.effort)) if msg.effort else {}
        # 实时护栏：速度 / 力矩 / 偏离基位。
        for name, p in pos.items():
            if name in self.base and name in ARM_JOINTS:
                dev = abs(p - self.base[name])
                if dev > self.dev_abort:
                    self.abort_reason = f"dev {name} {dev:.3f} rad"
            if name in vel and abs(vel[name]) > self.vel_abort:
                self.abort_reason = self.abort_reason or \
                    f"vel {name} {vel[name]:.2f} rad/s"
            if name in eff and abs(eff[name]) > self.eff_abort:
                self.abort_reason = self.abort_reason or \
                    f"eff {name} {eff[name]:.2f} Nm"
        with self._lock:
            self.samples.append((t, pos, vel, eff))
            if len(self.samples) > 200000:
                self.samples = self.samples[-100000:]

    def clear(self):
        with self._lock:
            self.samples = []
            self.abort_reason = ""

    def window(self, t0: float, t1: float):
        with self._lock:
            return [s for s in self.samples if t0 <= s[0] <= t1]

    def latest_positions(self, joints) -> List[float]:
        with self._lock:
            if not self.samples:
                return []
            _, pos, _, _ = self.samples[-1]
            return [pos.get(j, 0.0) for j in joints]


class StateRecorder:
    def __init__(self, node):
        self._lock = threading.Lock()
        self.state = ""
        from a3_msgs.msg import ArmStatus
        node.create_subscription(ArmStatus, "/a3/arm_status", self._cb, 10)

    def _cb(self, msg):
        with self._lock:
            self.state = msg.state

    def get(self) -> str:
        with self._lock:
            return self.state


class TempMonitor:
    """订阅 /a3/motor/states，跟踪各关节温度与当前最大温度（过温护栏）。"""

    def __init__(self, node):
        self._lock = threading.Lock()
        self.temps: Dict[str, float] = {}
        from a3_can_bridge.msg import MotorStates
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(MotorStates, "/a3/motor/states", self._cb, qos)

    def _cb(self, msg):
        with self._lock:
            for s in msg.states:
                if 1 <= s.motor_id <= 7:
                    self.temps[f"L{s.motor_id}_joint"] = s.temperature_c

    def max_temp(self) -> float:
        with self._lock:
            return max(self.temps.values()) if self.temps else 0.0

    def joint_temp(self, joint: str) -> float:
        with self._lock:
            return self.temps.get(joint, 0.0)


class Gains:
    """运行时增益读写（/a3_hardware_health 参数）。"""

    def __init__(self, node):
        self._node = node
        from rcl_interfaces.srv import GetParameters, SetParameters
        self._get_cli = node.create_client(
            GetParameters, "/a3_hardware_health/get_parameters")
        self._set_cli = node.create_client(
            SetParameters, "/a3_hardware_health/set_parameters")
        self.available = False

    def wait(self, timeout: float = 5.0) -> bool:
        self.available = (
            self._get_cli.wait_for_service(timeout_sec=timeout) and
            self._set_cli.wait_for_service(timeout_sec=timeout))
        return self.available

    def get(self, joints) -> Dict[str, Tuple[float, float]]:
        from rcl_interfaces.srv import GetParameters
        names = []
        for j in joints:
            names += [f"kp_{j}", f"kd_{j}"]
        req = GetParameters.Request()
        req.names = names
        fut = self._get_cli.call_async(req)
        if not wait_future(fut, 3.0):
            raise RuntimeError("get_parameters timeout")
        res = fut.result()
        out: Dict[str, Tuple[float, float]] = {}
        for i, j in enumerate(joints):
            kp = res.values[2 * i].double_value
            kd = res.values[2 * i + 1].double_value
            out[j] = (kp, kd)
        return out

    def set(self, joint: str, kp: float, kd: float) -> bool:
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        params = []
        for name, val in ((f"kp_{joint}", kp), (f"kd_{joint}", kd)):
            params.append(Parameter(
                name=name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(val))))
        req = SetParameters.Request()
        req.parameters = params
        fut = self._set_cli.call_async(req)
        if not wait_future(fut, 3.0):
            return False
        return all(r.successful for r in fut.result().results)

    def restore_nominal(self, joints):
        ok = True
        for j in joints:
            ok = self.set(j, NOMINAL_KP, NOMINAL_KD) and ok
        return ok


def send_fjt(node, ac, target: Sequence[float], duration: float) -> bool:
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration as RDuration
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ARM_JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = [float(x) for x in target]
    pt.velocities = [0.0] * 6
    pt.time_from_start = RDuration(
        sec=int(duration), nanosec=int((duration - int(duration)) * 1e9))
    goal.trajectory.points = [pt]
    goal.goal_time_tolerance = RDuration(sec=int(GOAL_TOL), nanosec=0)
    gh_fut = ac.send_goal_async(goal)
    if not wait_future(gh_fut, 5.0):
        return False
    gh = gh_fut.result()
    if not gh.accepted:
        return False
    res_fut = gh.get_result_async()
    if not wait_future(res_fut, duration + 15.0):
        return False
    return res_fut.result().result.error_code == \
        FollowJointTrajectory.Result.SUCCESSFUL


def open_gate(node, timeout: float = 10.0) -> bool:
    """F111: 发 /power_sequence/command=start 开电源门（gate 锁存，使能前置）。"""
    from std_msgs.msg import String, Bool
    pub = node.create_publisher(String, "/power_sequence/command", 10)
    gate = {"v": False}
    node.create_subscription(Bool, "/power_sequence/gate_open",
                             lambda m: gate.__setitem__("v", bool(m.data)), 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        pub.publish(String(data="start"))
        if gate["v"]:
            return True
        time.sleep(0.5)
    return False


def goto_named_pose(node, pose_name: str, timeout: float = 60.0) -> bool:
    """经编排层 /a3/arm/goto_named_pose 移动到命名位姿（需 gate 已开 + READY）。"""
    from a3_msgs.srv import GotoNamedPose
    cli = node.create_client(GotoNamedPose, "/a3/arm/goto_named_pose")
    if not cli.wait_for_service(timeout_sec=5.0):
        return False
    req = GotoNamedPose.Request()
    req.pose_name = pose_name
    fut = cli.call_async(req)
    if not wait_future(fut, timeout):
        return False
    return bool(fut.result().success)


PROGRESS_FILE = os.path.expanduser("~/.a3/kp_kd_autotune_progress.yaml")
TEMP_WARN_C = 90.0


def save_progress(path: str, results: Dict[str, dict]) -> None:
    import yaml
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump({"joints": results}, f, allow_unicode=True, sort_keys=False)


def load_progress(path: str) -> Dict[str, dict]:
    import yaml
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data.get("joints", {}) or {}


def check_health(st_rec, temp_mon, warn_c: float = TEMP_WARN_C) -> Tuple[bool, str]:
    """FSM 须 READY 才可继续；过温/故障由 F44 保护自动转 COOLING/FAULT。

    温度 ≥ warn 仅由调用方日志提醒，不在此触发恢复（恢复只在 FSM 离开 READY 时）。
    """
    s = st_rec.get()
    if s != "READY":
        return False, f"FSM state={s or '?'}"
    return True, ""


def recover_after_overtemp(node, st_rec, temp_mon, gains, rec, warn_c: float = TEMP_WARN_C,
                           cool_timeout: float = 900.0):
    """过温/FAULT 后：回落名义→等 F44 冷却到 DISABLED→开门→使能→回 home→返回新基位。

    返回 (new_base, ok)。ok=False 表示无法恢复（需人工）。
    """
    print(f"\n[!] 触发恢复：state={st_rec.get()} max_temp={temp_mon.max_temp():.0f}C",
          flush=True)
    if gains.available:
        gains.restore_nominal(ARM_JOINTS)
    # 等 F44 冷却（COOLING/FAULT → DISABLED）。FAULT 表示 reset 被拒（如
    # switch_controller STRICT）但臂已 safe-park 到 idle；调 disable 清理到 DISABLED。
    t0 = time.monotonic()
    disable_tried = False
    while time.monotonic() - t0 < cool_timeout:
        s = st_rec.get()
        if s == "DISABLED":
            break
        if s == "FAULT" and not disable_tried:
            disable_tried = True
            from std_srvs.srv import Trigger
            cli = node.create_client(Trigger, "/a3/arm/disable")
            if cli.wait_for_service(timeout_sec=3.0):
                fut = cli.call_async(Trigger.Request())
                if wait_future(fut, 10.0):
                    print(f"[i] FAULT 下 disable: {fut.result().message}", flush=True)
        if int(time.monotonic() - t0) % 10 == 0:
            print(f"    等冷却中 state={s} max_temp={temp_mon.max_temp():.0f}C",
                  flush=True)
        time.sleep(2.0)
    else:
        print("[x] 冷却超时（600s 未回 DISABLED），需人工介入", flush=True)
        return None, False
    print(f"[+] 已冷却（DISABLED，max_temp={temp_mon.max_temp():.0f}C），"
          f"重新开门+使能+回 home ...", flush=True)
    if not open_gate(node):
        print("[x] open_gate 失败", flush=True)
        return None, False
    from std_srvs.srv import Trigger
    en_cli = node.create_client(Trigger, "/a3/arm/enable")
    if en_cli.wait_for_service(timeout_sec=15.0):
        fut = en_cli.call_async(Trigger.Request())
        if not wait_future(fut, 20.0):
            print("[x] enable 超时", flush=True)
            return None, False
        if not fut.result().success:
            print(f"[x] enable 失败: {fut.result().message}", flush=True)
            return None, False
    end = time.monotonic() + 20.0
    while time.monotonic() < end and st_rec.get() != "READY":
        time.sleep(0.2)
    if st_rec.get() != "READY":
        print(f"[x] 未回 READY（state={st_rec.get()}）", flush=True)
        return None, False
    if not goto_named_pose(node, "home"):
        print("[x] goto home 失败", flush=True)
        return None, False
    time.sleep(1.0)
    got = []
    for _ in range(50):
        got = rec.latest_positions(ARM_JOINTS)
        if len(got) == len(ARM_JOINTS):
            break
        time.sleep(0.1)
    if len(got) != len(ARM_JOINTS):
        print("[x] 回 home 后读不到 /joint_states", flush=True)
        return None, False
    rec.base = dict(zip(ARM_JOINTS, got))
    rec.abort_reason = ""  # 清掉恢复前（回 idle 大幅偏离）留下的陈旧护栏触发
    print(f"[+] 恢复完成，新基位: {dict(zip(ARM_JOINTS, [round(x,4) for x in got]))}",
          flush=True)
    return got, True


def rms(vals: Sequence[float]) -> float:
    if not vals:
        return 0.0
    return math.sqrt(sum(x * x for x in vals) / len(vals))


def compute_cost(samples, joint: str, target: float, base: float,
                 settle_s: float) -> Tuple[float, dict]:
    """由一次激励的采样算该关节的成本 J 与分解指标。"""
    pos_all = [(s[0], s[1].get(joint)) for s in samples if joint in s[1]]
    vel_all = [(s[0], s[2].get(joint)) for s in samples if joint in s[2]]
    eff_all = [(s[0], s[3].get(joint)) for s in samples if joint in s[3]]
    if len(pos_all) < 5:
        return 1e9, {"error": "no samples"}

    pos = [p for _, p in pos_all]
    vel = [v for _, v in vel_all]
    eff = [e for _, e in eff_all]

    final = pos[-1]
    settle_err = abs(final - target)

    # 超调：正向步 target>base 时 max(pos)-target；反向步相反。
    if target >= base:
        overshoot = max(0.0, max(pos) - target)
    else:
        overshoot = max(0.0, target - min(pos))

    # 落定尾窗 ringing：末 settle_s 内 pos 相对 target 的 RMS。
    t_max = pos_all[-1][0]
    tail_pos = [p for t, p in pos_all if t >= t_max - settle_s]
    if len(tail_pos) < 2:
        tail_pos = pos[-5:]
    ringing = rms([p - target for p in tail_pos])

    peak_vel = max((abs(v) for v in vel), default=0.0)
    tail_eff = [e for t, e in eff_all if t >= t_max - settle_s]
    if len(tail_eff) < 2:
        tail_eff = eff[-5:] if eff else []
    torque_rms = rms(tail_eff)

    metrics = {
        "settle_err": settle_err,
        "overshoot": overshoot,
        "ringing": ringing,
        "peak_vel": peak_vel,
        "torque_rms": torque_rms,
    }
    # 权重：落定误差主导，超调/ringing 次之，力矩纹波轻罚。
    cost = (4.0 * settle_err + 3.0 * overshoot + 2.0 * ringing
            + 0.05 * torque_rms)
    return cost, metrics


def _safe_return(node, ac, gains, joint, base):
    """失败/中止时：回落名义增益并尽力回到基位。"""
    gains.set(joint, NOMINAL_KP, NOMINAL_KD)
    time.sleep(0.15)
    send_fjt(node, ac, base, MOVE_DT)
    time.sleep(SETTLE_S)


def evaluate(node, ac, recorder, gains, joint: str, base: List[float],
             amp: float, kp: float, kd: float,
             move_dt: float, settle_s: float):
    """设增益 → 跑一次激励（正向+回位）→ 算成本。返回 (cost, metrics)。"""
    if not gains.set(joint, kp, kd):
        return 1e9, {"error": "set gains failed"}
    time.sleep(0.15)  # 等 RT 线程读到新增益（良性竞态下保守等待）

    idx = ARM_JOINTS.index(joint)
    target = list(base)
    target[idx] = base[idx] + amp
    lo, hi = JOINT_LIMITS[joint]
    if not (lo + LIMIT_MARGIN <= target[idx] <= hi - LIMIT_MARGIN):
        return 1e9, {"error": f"target {target[idx]:.3f} out of [{lo},{hi}]"}

    # 正向段。
    recorder.clear()
    t0 = time.monotonic()
    ok = send_fjt(node, ac, target, move_dt)
    time.sleep(settle_s)
    t1 = time.monotonic()
    fwd_samples = recorder.window(t0, t1)
    fwd_abort = recorder.abort_reason
    if not ok or fwd_abort:
        _safe_return(node, ac, gains, joint, base)
        return 1e9, {"error": fwd_abort or "FJT move failed/aborted"}

    # 回位段（回到 base）。
    recorder.clear()
    ok2 = send_fjt(node, ac, base, move_dt)
    time.sleep(settle_s)
    if not ok2 or recorder.abort_reason:
        _safe_return(node, ac, gains, joint, base)
        return 1e9, {"error": recorder.abort_reason or "FJT return failed"}

    back = recorder.latest_positions(ARM_JOINTS)
    if back and abs(back[idx] - base[idx]) > DEV_ABORT:
        _safe_return(node, ac, gains, joint, base)
        return 1e9, {"error": "return deviation too large"}

    cost, metrics = compute_cost(fwd_samples, joint, target[idx], base[idx],
                                 settle_s)
    return cost, metrics


class RecoverNeeded(Exception):
    """扫参中途 FSM 离开 READY（过温/故障），需恢复。"""


def tune_joint(node, ac, recorder, gains, joint: str, base: List[float],
               amp: float, kp_c: List[float], kd_c: List[float],
               move_dt: float, settle_s: float,
               dry_run: bool, health_fn=None) -> Tuple[float, float, float, dict]:
    """对一个关节做坐标下降（kp 粗→kd 粗→kp 细→kd 细）。

    health_fn() 返回 (ok, reason)；扫参中途 ok=False 抛 RecoverNeeded 中断，
    由外层做「冷却→重新使能→回 home→续跑」，避免与 F44 过温保护抢轨迹。
    """
    best = {"kp": NOMINAL_KP, "kd": NOMINAL_KD, "cost": None, "m": None}

    def _try(kp, kd, tag):
        if health_fn is not None:
            ok, why = health_fn()
            if not ok:
                raise RecoverNeeded(why)
        cost, m = evaluate(node, ac, recorder, gains, joint, base, amp,
                           kp, kd, move_dt, settle_s)
        print(f"    {tag}: kp={kp:6.1f} kd={kd:5.2f} -> J={cost:7.4f} "
              f"{m}", flush=True)
        if best["cost"] is None or cost < best["cost"]:
            best.update(kp=kp, kd=kd, cost=cost, m=m)
        return cost

    print(f"\n== joint {joint} amp={amp:.3f} ==", flush=True)
    if dry_run:
        print("  [dry-run] 跳过实际扫参", flush=True)
        return NOMINAL_KP, NOMINAL_KD, 0.0, {"dry_run": True}

    # 1) kp 粗扫（kd 固定名义）。
    for kp in kp_c:
        if _try(kp, NOMINAL_KD, "kp coarse") == 1e9 and recorder.abort_reason:
            break
    kp_best = best["kp"]

    # 2) kd 粗扫（kp 固定 kp_best）。
    for kd in kd_c:
        if _try(kp_best, kd, "kd coarse") == 1e9 and recorder.abort_reason:
            break
    kd_best = best["kd"]

    # 3) kp 细扫（kd_best 固定）。
    kp_fine = sorted({kp_best - 20, kp_best - 10, kp_best,
                      kp_best + 10, kp_best + 20})
    for kp in kp_fine:
        if kp < KP_MIN or kp > KP_MAX:
            continue
        if _try(kp, kd_best, "kp fine") == 1e9 and recorder.abort_reason:
            break

    # 4) kd 细扫（用 kp 细扫后的最优 kp 固定）。
    kd_fine = sorted({kd_best - 1.0, kd_best - 0.5, kd_best,
                      kd_best + 0.5, kd_best + 1.0})
    for kd in kd_fine:
        if kd < KD_MIN or kd > KD_MAX:
            continue
        if _try(best["kp"], kd, "kd fine") == 1e9 and recorder.abort_reason:
            break

    return best["kp"], best["kd"], best["cost"], best["m"]


def main():
    ap = argparse.ArgumentParser(description="F138 kp/kd autotune")
    ap.add_argument("--mode", choices=["connect", "real", "mock"],
                    default="connect")
    ap.add_argument("--joints", default=",".join(ARM_JOINTS),
                    help="逗号分隔，如 L2_joint,L3_joint")
    ap.add_argument("--can-interface", default="can1")
    ap.add_argument("--wait-sec", type=float, default=45.0)
    ap.add_argument("--dry-run", action="store_true",
                    help="只走 enable→READY→扫参框架，不实际设增益/运动")
    ap.add_argument("--move-dt", type=float, default=MOVE_DT)
    ap.add_argument("--settle-s", type=float, default=SETTLE_S)
    ap.add_argument("--out", default=os.path.expanduser("~/.a3/kp_kd_tuned.yaml"))
    ap.add_argument("--fresh", action="store_true",
                    help="忽略进度文件从头开始（默认自动续跑已完成关节）")
    args = ap.parse_args()
    ensure_env(args.mode)

    import rclpy
    import rclpy.action
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger
    from control_msgs.action import FollowJointTrajectory

    joints = [j.strip() for j in args.joints.split(",") if j.strip()]
    joints = [j if j.endswith("_joint") else j + "_joint" for j in joints]
    for j in joints:
        if j not in ARM_JOINTS:
            print(f"ERROR: unknown joint {j}", flush=True)
            sys.exit(2)

    rclpy.init()
    node = rclpy.create_node("f138_kp_kd_autotune")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    launch_proc = None
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_path = f"/tmp/f138_launch_{ts}.log"

    def cleanup():
        if launch_proc is not None and launch_proc.poll() is None:
            try:
                os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                launch_proc.wait(timeout=12.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                launch_proc.wait(timeout=5.0)

    def sig_handler(signum, frame):
        print(f"\nsignal {signum}, cleaning up...", flush=True)
        cleanup()
        sys.exit(1)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    gains = Gains(node)

    # 自起栈（real/mock）；connect 附着已有栈。
    if args.mode in ("real", "mock"):
        launch_args = ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
                       f"hardware:={'mock' if args.mode == 'mock' else 'can'}"]
        if args.mode == "real":
            launch_args.append(f"can_interface:={args.can_interface}")
        else:
            launch_args += ["use_rviz:=false", "use_mqtt:=false",
                            "use_teleop:=false"]
        logf = open(log_path, "w")
        launch_proc = subprocess.Popen(
            launch_args, stdout=logf, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid)
        print(f"launch started: {' '.join(launch_args)}", flush=True)

    try:
        # 等栈 + 使能 + READY。
        end = time.monotonic() + args.wait_sec
        from controller_manager_msgs.srv import ListControllers
        lc = node.create_client(ListControllers,
                                "/controller_manager/list_controllers")
        arm_active = False
        while time.monotonic() < end:
            if lc.wait_for_service(timeout_sec=1.0):
                req = ListControllers.Request()
                fut = lc.call_async(req)
                if wait_future(fut, 3.0):
                    ctrls = {c.name: c.state for c in fut.result().controller}
                    arm_active = ctrls.get("arm_controller") == "active"
                    if arm_active:
                        break
            time.sleep(0.5)
        # F111: 先开电源门（gate 锁存；使能/goto/move 前置）。
        if not open_gate(node):
            print("WARN: open_gate 失败（可能已开或 mock 无 power_sequence）", flush=True)

        if not arm_active:
            print("arm_controller not active; trying /a3/arm/enable", flush=True)
            en_cli = node.create_client(Trigger, "/a3/arm/enable")
            if en_cli.wait_for_service(timeout_sec=15.0):
                # 等新鲜 /joint_states 后 enable（FSM 前置检查）。
                for _ in range(20):
                    time.sleep(0.5)
                fut = en_cli.call_async(Trigger.Request())
                if not wait_future(fut, 10.0):
                    print("ERROR: /a3/arm/enable timeout", flush=True)
                    sys.exit(1)
                if not fut.result().success:
                    print(f"ERROR: enable failed: {fut.result().message}",
                          flush=True)
                    sys.exit(1)

        st_rec = StateRecorder(node)
        temp_mon = TempMonitor(node)
        end = time.monotonic() + 10.0
        while time.monotonic() < end and st_rec.get() != "READY":
            time.sleep(0.1)
        print(f"FSM state: {st_rec.get()}", flush=True)
        if st_rec.get() != "READY":
            print("ERROR: FSM not READY", flush=True)
            sys.exit(1)

        # 必须先回 home 位姿（上次标定位），避免 idle 折叠碰撞。
        if not goto_named_pose(node, "home"):
            print("ERROR: goto home 失败", flush=True)
            sys.exit(1)
        # 等落定稳定后再采基位。
        time.sleep(1.5)

        # 读当前位姿作为基位。
        rec = Recorder(node, {})
        got = []
        for _ in range(50):
            got = rec.latest_positions(ARM_JOINTS)
            if len(got) == len(ARM_JOINTS):
                break
            time.sleep(0.1)
        if len(got) != len(ARM_JOINTS):
            print("ERROR: no /joint_states", flush=True)
            sys.exit(1)
        base = got
        print(f"base pose: {dict(zip(ARM_JOINTS, [round(x,4) for x in base]))}",
              flush=True)

        # 重建 Recorder，注入基位以启用越限护栏。
        rec = Recorder(node, dict(zip(ARM_JOINTS, base)))

        # 读名义增益（当前运行时值），作为安全回落目标。
        gains.wait(timeout=5.0)
        if gains.available:
            try:
                cur = gains.get(ARM_JOINTS)
                print(f"current gains: { {j: (round(v[0],1), round(v[1],2)) for j,v in cur.items()} }",
                      flush=True)
            except Exception as e:
                print(f"WARN: read gains failed ({e}); use nominal 80/2", flush=True)
                cur = {j: (NOMINAL_KP, NOMINAL_KD) for j in ARM_JOINTS}
        else:
            print("WARN: /a3_hardware_health 不可达（mock 无此节点）；"
                  "仅演示流程", flush=True)
            cur = {j: (NOMINAL_KP, NOMINAL_KD) for j in ARM_JOINTS}

        ac = rclpy.action.ActionClient(
            node, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory")
        if not ac.wait_for_server(timeout_sec=10.0):
            print("ERROR: FJT action server unavailable", flush=True)
            sys.exit(1)

        results: Dict[str, dict] = {}
        # 续跑：加载已完成关节（--fresh 忽略）。
        if not args.fresh:
            done = load_progress(PROGRESS_FILE)
            results.update(done)
            if done:
                print(f"续跑：已完成 {list(done.keys())}", flush=True)
        eval_count = 0
        abort_outer = False
        try:
            for j in joints:
                if j in results:
                    print(f"跳过已完成的 {j}", flush=True)
                    continue
                while True:
                    ok, why = check_health(st_rec, temp_mon)
                    t = temp_mon.max_temp()
                    if t >= TEMP_WARN_C:
                        print(f"[!] 温度 {t:.0f}C ≥ warn {TEMP_WARN_C:.0f}C"
                              f"（达 protect 由 F44 保护自动回 idle）", flush=True)
                    if not ok:
                        print(f"[!] {j} 前 {why}；保存进度并恢复 ...", flush=True)
                        save_progress(PROGRESS_FILE, results)
                        new_base, rec_ok = recover_after_overtemp(
                            node, st_rec, temp_mon, gains, rec)
                        if not rec_ok:
                            abort_outer = True
                            break
                        base = new_base
                        continue
                    amp = DEFAULT_AMP.get(j, 0.15)
                    try:
                        kp_best, kd_best, cost, m = tune_joint(
                            node, ac, rec, gains, j, base, amp,
                            KP_COARSE, KD_COARSE, args.move_dt, args.settle_s,
                            args.dry_run,
                            health_fn=lambda: check_health(st_rec, temp_mon))
                    except RecoverNeeded as e:
                        print(f"[!] {j} 扫参中 {e}；保存进度并恢复 ...", flush=True)
                        save_progress(PROGRESS_FILE, results)
                        new_base, rec_ok = recover_after_overtemp(
                            node, st_rec, temp_mon, gains, rec)
                        if not rec_ok:
                            abort_outer = True
                            break
                        base = new_base
                        continue
                    results[j] = {"kp": kp_best, "kd": kd_best, "cost": cost,
                                  "metrics": m}
                    if gains.available and not args.dry_run:
                        gains.set(j, kp_best, kd_best)
                    save_progress(PROGRESS_FILE, results)
                    eval_count += len(KP_COARSE) + len(KD_COARSE) + 6
                    break
                if abort_outer:
                    break
                if rec.abort_reason:
                    print(f"ABORT: {rec.abort_reason}", flush=True)
                    break
                if eval_count >= MAX_ITER:
                    print("MAX_ITER reached", flush=True)
                    break
        finally:
            # 无论如何回落名义增益（安全）。
            if gains.available and not args.dry_run:
                gains.restore_nominal(ARM_JOINTS)
                print("restored nominal gains (80/2)", flush=True)

        # 汇总。
        print("\n==== F138 autotune result ====", flush=True)
        for j, r in results.items():
            print(f"  {j}: kp={r['kp']:.1f} kd={r['kd']:.2f} J={r['cost']:.4f}",
                  flush=True)

        out_dir = os.path.dirname(args.out)
        os.makedirs(out_dir, exist_ok=True)
        import yaml
        with open(args.out, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                {"joints": {j: {"kp": r["kp"], "kd": r["kd"],
                                "cost": r["cost"], "metrics": r["metrics"]}
                            for j, r in results.items()},
                 "base": dict(zip(ARM_JOINTS, base)),
                 "stamp": ts},
                f, allow_unicode=True, sort_keys=False)
        print(f"written: {args.out}", flush=True)

    finally:
        cleanup()
        # 干净退出：先停后台 executor spin 线程并 join，再 shutdown，避免进程退出时
        # rclpy 上下文析构与 spin 线程竞争触发 SIGABRT（core dumped）。
        try:
            executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
        try:
            spin_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
