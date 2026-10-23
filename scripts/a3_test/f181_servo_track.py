#!/usr/bin/env python3
"""F181 真机 servo 跟踪验收：sweep（三轴往返）与 random（随机方形跟踪）。

前置：
  - 真机栈已起（默认 servo 链即 a3_servo_target）：
        ./scripts/a3_stack.sh restart
  - 臂已使能（PS4 L3 或 /a3/arm/enable），state=READY，且门禁已开（/power_sequence Running）。
  - /a3/ee_pose（F175）可用。
  - **不需要手工摆到 home**：脚本开始会自动 goto home 并以那里为参考点；结束时再回 home。
    加 --no-home 则跳过（以当前位姿为参考）。

用法：
  python3 scripts/a3_test/f181_servo_track.py sweep
  python3 scripts/a3_test/f181_servo_track.py sweep --v 0.03 --axis xyz
  # 段式方形（默认）：每步之间停 0.15~0.35s，专门制造 正→0 / 正→负
  python3 scripts/a3_test/f181_servo_track.py random --vmax 0.05 --duration 60 --bound 0.10
  # 连续随机速度（OU 过程、不停顿），更接近人脸跟踪
  python3 scripts/a3_test/f181_servo_track.py random --continuous --vmax 0.05 --duration 60 --axes xy
  python3 scripts/a3_test/f181_servo_track.py random --vmax 0.03 --duration 20 --bound 0.05   # 预演

输出：~/.a3/servo_track/<时间戳>/<mode>.json + 终端指标表（PASS/FAIL）。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from a3_msgs.msg import ArmStatus
from a3_msgs.srv import GotoNamedPose

try:
    import pinocchio as pin
except ImportError:  # 命令侧 FK 不可用时退化为只用实测
    pin = None

JOINTS = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]

# 每个子命令的 PASS 阈值
SWEEP_MAX_CMD_CV = 0.15
SWEEP_MAX_RIPPLE = 10.0      # [%]
SWEEP_MAX_HOME_ERR = 5.0     # [mm]
RANDOM_MAX_RMS_ERR = 0.010   # [m]
RANDOM_MAX_SETTLE_DRIFT = 0.003  # [m] 停稳后额外 0.5s 的漂移


def log(msg: str) -> None:
    print(msg, flush=True)


def lpf(x: np.ndarray, dt: float, fc: float = 10.0) -> np.ndarray:
    if len(x) == 0:
        return x
    a = dt / (1.0 / (2.0 * math.pi * fc) + dt)
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def ripple_pct(v: np.ndarray, dt: float) -> float:
    """0.3–3Hz 频带 RMS / |均值|，单位 %。"""
    if len(v) < 32 or abs(v.mean()) < 1e-9:
        return float("nan")
    v0 = v - v.mean()
    f = np.fft.rfftfreq(len(v0), dt)
    a = np.abs(np.fft.rfft(v0)) / len(v0) * 2.0
    band = (f >= 0.3) & (f <= 3.0)
    return float(np.sqrt(np.sum(a[band] ** 2)) / abs(v.mean()) * 100.0)


def ee_track_error(cmd: np.ndarray, meas: np.ndarray) -> np.ndarray:
    """把两组 (t, xyz) 按时间对齐后求逐点距离。"""
    if len(cmd) == 0 or len(meas) == 0:
        return np.zeros(0)
    tc = cmd[:, 0]
    tm = meas[:, 0]
    out = []
    j = 0
    for i in range(len(tc)):
        while j + 1 < len(tm) and tm[j + 1] <= tc[i]:
            j += 1
        out.append(np.linalg.norm(cmd[i, 1:] - meas[j, 1:]))
    return np.array(out)


def in_range(arr: np.ndarray, t0: float, t1: float) -> np.ndarray:
    if len(arr) == 0:
        return arr
    return arr[(arr[:, 0] >= t0) & (arr[:, 0] <= t1)]


def uniform_time(arr: np.ndarray) -> np.ndarray:
    """把带接收抖动的采样时间换成等间隔（位置顺序不变）。

    Python 订阅回调会成批到达（15ms 后紧跟 5ms），直接用它求导会假造速度尖峰。
    """
    if len(arr) < 2:
        return arr[:, 0] if len(arr) else np.zeros(0)
    t = arr[:, 0]
    span = t[-1] - t[0]
    if span <= 0:
        return t
    rate = (len(t) - 1) / span
    return t[0] + np.arange(len(t)) / rate


class ServoTrack(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("f181_servo_track")
        self.args = args
        self.lock = threading.Lock()
        self.js = []      # (t, positions[6], velocities[6])
        self.cmd = []     # (t, positions[6])
        self.ee = []      # (t, x, y, z)
        self.states = []  # (t, state, mode)
        self.twist_vec = (0.0, 0.0, 0.0)
        self.running = True
        self.pumping = True   # goto home 期间必须暂停：否则本脚本的 twist 会让节点抢轨迹
        self.home = None

        be = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
        rel = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(JointState, "/joint_states", self._on_js, be)
        self.create_subscription(PoseStamped, "/a3/ee_pose", self._on_ee, rel)
        self.create_subscription(JointTrajectory, "/arm_controller/joint_trajectory",
                                 self._on_cmd, rel)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_status, rel)
        self.twist_pub = self.create_publisher(TwistStamped, args.twist_topic, rel)
        self.goto_cli = self.create_client(GotoNamedPose, "/a3/arm/goto_named_pose")

        self._pump = threading.Thread(target=self._pump_loop, daemon=True)
        self._pump.start()

    # ---- 回调 ----
    def _on_js(self, msg: JointState) -> None:
        try:
            idx = [msg.name.index(n) for n in JOINTS]
        except ValueError:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        pos = [msg.position[i] for i in idx]
        vel = [msg.velocity[i] if i < len(msg.velocity) else 0.0 for i in idx]
        with self.lock:
            self.js.append((t, pos, vel))

    def _on_ee(self, msg: PoseStamped) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.position
        with self.lock:
            self.ee.append((t, p.x, p.y, p.z))

    def _on_cmd(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        try:
            idx = [msg.joint_names.index(n) for n in JOINTS]
        except ValueError:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        pt = msg.points[0]
        pos = [pt.positions[i] for i in idx]
        vel = [pt.velocities[i] if i < len(pt.velocities) else 0.0 for i in idx]
        with self.lock:
            self.cmd.append((t, pos, vel))

    def _on_status(self, msg: ArmStatus) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            self.states.append((t, msg.state, msg.mode))

    # ---- twist 泵（50Hz，与 PS4 mapper 一致）----
    def _pump_loop(self) -> None:
        while self.running and rclpy.ok():
            if not self.pumping:
                time.sleep(0.02)
                continue
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = self.twist_vec
            self.twist_pub.publish(m)
            time.sleep(0.02)

    # ---- 工具 ----
    def spin_for(self, duration: float, check=None) -> bool:
        """spin 指定时长；check() 返回 False 时提前退出（返回 False）。"""
        t_end = time.monotonic() + duration
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.005)
            if check is not None and check() is False:
                return False
        return True

    def state_now(self) -> str:
        with self.lock:
            return self.states[-1][1] if self.states else ""

    def ee_now(self):
        with self.lock:
            return self.ee[-1] if self.ee else None

    def wait_ready(self, timeout: float = 10.0) -> bool:
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state_now() == "READY":
                return True
        return False

    def wait_ee(self, timeout: float = 5.0) -> bool:
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.ee_now() is not None:
                return True
        return False

    def goto_home(self) -> bool:
        # 暂停 twist 泵：节点在 READY 下会让路，但停发更稳妥（避免旧 twist 新鲜度干扰）
        self.pumping = False
        self.stop()
        self.spin_for(0.35)
        # 已经在 home（关节差 <0.03rad）就跳过 goto：否则 move_group 会规划一条 2~3 点的
        # 0.1s 微调轨迹，JTC + MIT 位置环执行时会"抖一下"（用户实测在 home 跑脚本必现）。
        exp = self.expected_pose("home")
        with self.lock:
            meas = list(self.js[-1][1]) if self.js else None
        if exp and meas and len(exp) == len(meas):
            dmax = max(abs(a - b) for a, b in zip(exp, meas))
            if dmax < 0.03:
                log(f"[+] 已在 home（关节最大偏差 {dmax:.4f} rad），跳过 goto")
                p = self.ee_now()
                if p is None:
                    self.pumping = True
                    return False
                self.home = np.array(p[1:4])
                log(f"[+] home EE = {np.round(self.home * 1000, 2)} mm")
                self.pumping = True
                return True
        if not self.goto_cli.service_is_ready():
            self.goto_cli.wait_for_service(timeout_sec=5.0)
        if not self.goto_cli.service_is_ready():
            log("[x] /a3/arm/goto_named_pose 不可用（栈没起？）")
            self.pumping = True
            return False
        log("[*] goto home …")
        fut = self.goto_cli.call_async(GotoNamedPose.Request(pose_name="home"))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        resp = fut.result()
        if resp is None or not resp.success:
            msg = getattr(resp, "message", "no response")
            log(f"[x] goto home 失败：{msg}")
            if "gate" in str(msg).lower():
                log("    提示：门禁未开（/power_sequence 需 Running）——PS4 L3，或：")
                log("    ros2 topic pub --once /power_sequence/command std_msgs/msg/String \"{data: start}\"")
            self.pumping = True
            return False
        log(f"[+] {resp.message}")
        is_fallback = "fallback" in str(resp.message).lower()
        self.wait_settle()
        p = self.ee_now()
        if p is None:
            self.pumping = True
            return False
        if is_fallback:
            # 2 点线性兜底 = move_group 执行失败（典型：CAN 反馈 stale → freeze-hold →
            # JTC 状态超差 0.3rad 判 PATH_TOLERANCE_VIOLATED）。兜底有时能到 home，
            # 所以按关节位核对：到 home 就继续，没到就中止（避免在不可达位形里空跑）。
            exp = self.expected_pose("home")
            with self.lock:
                meas = list(self.js[-1][1]) if self.js else None
            dmax = (max(abs(a - b) for a, b in zip(exp, meas))
                    if (exp and meas and len(exp) == len(meas)) else 1e9)
            if dmax >= 0.05:
                log(f"[x] goto home 走了 2 点兜底且未到 home（关节最大误差 {dmax:.3f} rad），中止。")
                log("    请先确认 can1 为 ERROR-ACTIVE、日志无 'feedback stale'，再重试。")
                self.pumping = True
                return False
            log(f"[!] goto home 走了 2 点兜底（move_group 执行失败），但已到 home（误差 {dmax:.3f} rad），继续")
        self.home = np.array(p[1:4])
        log(f"[+] home EE = {np.round(self.home * 1000, 2)} mm")
        self.pumping = True
        return True

    def expected_pose(self, name: str = "home"):
        """从描述包的 named_poses.yaml 读目标关节角（用于核对兜底是否到位）。"""
        try:
            import yaml
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory(self.args.desc_pkg),
                                "config", "named_poses.yaml")
            with open(path, encoding="utf-8") as f:
                doc = yaml.safe_load(f)
            return [float(x) for x in doc["poses"][name]["positions"]]
        except Exception as exc:  # noqa: BLE001
            log(f"[!] 读取 named_poses 失败（跳过兜底核对）：{exc}")
            return None

    def wait_settle(self, tol: float = 0.003, hold: float = 0.4, timeout: float = 8.0) -> None:
        """等实测末端速度连续 hold 秒 < tol（m/s）。

        用 0.3s 窗口位移求速度：/a3/ee_pose 只有 20Hz，两点差分的量化噪声 ~2mm/s。
        """
        t_end = time.monotonic() + timeout
        stable_since = None
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            with self.lock:
                if len(self.ee) < 8:
                    continue
                a = self.ee[-8]
                b = self.ee[-1]
            dt = b[0] - a[0]
            if dt <= 0.0:
                continue
            v = float(np.linalg.norm(np.array(b[1:4]) - np.array(a[1:4])) / dt)
            if v < tol:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= hold:
                    return
            else:
                stable_since = None
        log("[!] 等待停稳超时（继续）")

    def set_twist(self, vx: float, vy: float, vz: float) -> None:
        self.twist_vec = (float(vx), float(vy), float(vz))

    def stop(self) -> None:
        self.set_twist(0.0, 0.0, 0.0)

    # ---- 测量 ----
    def cmd_ee(self):
        """命令关节角 → 末端位置 (t, x, y, z) 与末端速度 (t, vx, vy, vz)。

        速度直接用消息里的关节速度经雅可比投影：节点每拍给的是真实命令速度，
        比“接收时间差分”稳（Python 订阅回调有批次抖动，会假造 15ms/5ms 间隔）。
        """
        if pin is None:
            return np.zeros((0, 4)), np.zeros((0, 4))
        pkg = self.args.desc_pkg
        try:
            from ament_index_python.packages import get_package_share_directory
            urdf = os.path.join(get_package_share_directory(pkg), "urdf", "el_a3.urdf")
        except Exception:
            urdf = self.args.urdf
        try:
            model = pin.buildModelFromUrdf(urdf)
        except Exception as exc:  # noqa: BLE001
            log(f"[!] 载入 URDF 失败（命令侧 FK 跳过）：{exc}")
            return np.zeros((0, 4)), np.zeros((0, 4))
        data = model.createData()
        fid = model.getFrameId("end_effector")
        with self.lock:
            rows = list(self.cmd)
        pos_out, vel_out = [], []
        for t, q, qd in rows:
            qa = np.array(q)
            pin.forwardKinematics(model, data, qa)
            pin.updateFramePlacements(model, data)
            p = data.oMf[fid].translation
            jac = pin.computeFrameJacobian(model, data, qa, fid, pin.LOCAL_WORLD_ALIGNED)[:, :6]
            v = jac @ np.array(qd)
            pos_out.append((t, p[0], p[1], p[2]))
            vel_out.append((t, v[0], v[1], v[2]))
        return np.array(pos_out), np.array(vel_out)

    def measured_ee(self):
        with self.lock:
            return np.array(self.ee) if self.ee else np.zeros((0, 4))


# ---------------------------------------------------------------- sweep
def run_sweep(node: ServoTrack, args: argparse.Namespace) -> dict:
    phases = []
    for axis in args.axis:
        ai = "xyz".index(axis)
        for sgn, dist in ((1.0, 0.10), (-1.0, 0.20), (1.0, 0.10)):
            phases.append((ai, sgn * args.v, dist / args.v, axis, sgn, dist))

    log(f"[*] sweep：v={args.v:.3f} m/s，轴={args.axis}，共 {len(phases)} 段")
    rec = []
    for ai, vel, dur, axis, sgn, dist in phases:
        v = [0.0, 0.0, 0.0]
        v[ai] = vel
        node.set_twist(*v)
        t0 = node.get_clock().now().nanoseconds * 1e-9
        ok = node.spin_for(dur, check=lambda: node.state_now() in ("READY", ""))
        node.stop()
        t1 = node.get_clock().now().nanoseconds * 1e-9
        log(f"    {axis} {'+' if sgn > 0 else '-'}{dist * 100:.0f}cm  {t0:.2f}..{t1:.2f}s")
        if not ok:
            log("[x] 状态不再是 READY，中止 sweep")
            break
        rec.append(dict(axis=axis, ai=ai, sgn=sgn, dist=dist, t0=t0, t1=t1))
        node.spin_for(0.4)
    node.stop()
    node.spin_for(1.0)
    node.wait_settle(timeout=10.0)

    cmd_pos, cmd_vel = node.cmd_ee()
    meas = node.measured_ee()
    if rec:
        t_lo, t_hi = rec[0]["t0"] - 0.2, rec[-1]["t1"] + 1.2
        cmd_pos = in_range(cmd_pos, t_lo, t_hi)
        cmd_vel = in_range(cmd_vel, t_lo, t_hi)
        meas = in_range(meas, t_lo, t_hi)
    gaps = 0
    cmd_rate = float("nan")
    if len(cmd_pos) > 10:
        span = cmd_pos[-1, 0] - cmd_pos[0, 0]
        cmd_rate = (len(cmd_pos) - 1) / span if span > 0 else float("nan")
        gaps = int((np.diff(cmd_pos[:, 0]) > 0.02).sum())

    rows = []
    for r in rec:
        ai = r["ai"]
        # 两端各裁掉 pad 秒（跳过加减速段）。pad 按段时长自适应：
        # 固定 0.7s 在 0.4/0.8 m/s 时（10cm 段只有 0.25/0.13s）会把整段裁空 → 表格无行。
        pad = min(0.7, 0.15 * max(0.0, r["t1"] - r["t0"]))
        m = (cmd_pos[:, 0] >= r["t0"] + pad) & (cmd_pos[:, 0] <= r["t1"] - pad)
        mm = (meas[:, 0] >= r["t0"] + pad) & (meas[:, 0] <= r["t1"] - pad)
        if m.sum() < 10 or mm.sum() < 10:
            continue
        vc = cmd_vel[m, 1 + ai]  # 节点下发的末端轴向速度（J·q̇）
        tc_dt = 1.0 / cmd_rate if cmd_rate and cmd_rate > 1.0 else 0.01
        tm = uniform_time(meas[mm])
        dtm = float(np.median(np.diff(tm))) if len(tm) > 1 else 0.05
        vm = np.gradient(lpf(meas[mm, 1 + ai], dtm), tm)
        dcmd = (cmd_pos[m, 1 + ai][-1] - cmd_pos[m, 1 + ai][0]) * 1000
        dmeas = (meas[mm, 1 + ai][-1] - meas[mm, 1 + ai][0]) * 1000
        err = ee_track_error(cmd_pos[m][:, [0, 1 + ai]], meas[mm][:, [0, 1 + ai]])
        resid = meas[mm, 1 + ai] - np.polyval(np.polyfit(tm - tm[0], meas[mm, 1 + ai], 1), tm - tm[0])
        rows.append(dict(
            axis=r["axis"], sgn=r["sgn"], dist=r["dist"],
            cmd_mm=dcmd, meas_mm=dmeas,
            cmd_v=vc.mean(), cmd_cv=float(vc.std() / abs(vc.mean())) if abs(vc.mean()) > 1e-9 else float("nan"),
            cmd_ripple=ripple_pct(vc, tc_dt),
            meas_cv=float(vm.std() / abs(vm.mean())) if abs(vm.mean()) > 1e-9 else float("nan"),
            meas_ripple=ripple_pct(vm, dtm),
            err_mean=float(err.mean()) if len(err) else float("nan"),
            err_max=float(err.max()) if len(err) else float("nan"),
            resid_std=float(resid.std()),
        ))

    home_err = float(np.linalg.norm(meas[-1, 1:4] - node.home)) if len(meas) else float("nan")
    worst_cv = max((r["cmd_cv"] for r in rows), default=float("nan"))
    worst_rip = max((r["cmd_ripple"] for r in rows), default=float("nan"))
    rate_ok = (not math.isnan(cmd_rate)) and cmd_rate > 97.0
    ok = (worst_cv < SWEEP_MAX_CMD_CV and worst_rip < SWEEP_MAX_RIPPLE
          and home_err * 1000 < SWEEP_MAX_HOME_ERR and rate_ok)

    log("\n 轴  方向  行程(cm)  命令CV  命令纹波  实测CV  实测纹波  跟随均值/最大(mm)  残差std(mm)")
    for r in rows:
        log(f"  {r['axis']}   {'+' if r['sgn'] > 0 else '-'}     "
            f"{r['dist'] * 100:5.1f}   {r['cmd_cv']:6.3f}  {r['cmd_ripple']:7.1f}%  "
            f"{r['meas_cv']:6.3f}  {r['meas_ripple']:7.1f}%   "
            f"{r['err_mean'] * 1000:5.1f}/{r['err_max'] * 1000:5.1f}      {r['resid_std'] * 1000:5.2f}")
    log(f"\n  回 home 误差 {home_err * 1000:.2f} mm；命令流 {cmd_rate:.1f} Hz"
        f"（到达间隔 >20ms 的批抖动 {gaps} 处，仅供参考）")
    log(f"  >>> sweep {'PASS' if ok else 'FAIL'}"
        f"（阈值：命令CV<{SWEEP_MAX_CMD_CV}、纹波<{SWEEP_MAX_RIPPLE}%、回home<{SWEEP_MAX_HOME_ERR}mm、"
        f"命令流>97Hz）")

    return dict(mode="sweep", v=args.v, gap_count=gaps, cmd_rate_hz=cmd_rate,
                home_err_mm=home_err * 1000, pass_=bool(ok), rows=rows,
                worst_cmd_cv=worst_cv, worst_cmd_ripple=worst_rip)


# ---------------------------------------------------------------- random
def run_random(node: ServoTrack, args: argparse.Namespace) -> dict:
    rng = random.Random(args.seed)
    home = node.home
    soft = args.bound * 0.9
    hard = args.bound
    log(f"[*] random：vmax={args.vmax:.3f} m/s，{args.duration:.0f}s，边界 home±{hard * 100:.0f}cm"
        f"（软边界 {soft * 100:.0f}cm）")

    t_start = time.monotonic()
    t_run0 = node.get_clock().now().nanoseconds * 1e-9
    reversals = 0
    last_dir = {}
    max_dev = np.zeros(3)
    boundary_hit = 0
    abort_reason = ""
    steps = 0
    err_samples = []
    stop_travel = []
    settle_drift = []

    def measured_dev() -> np.ndarray:
        p = node.ee_now()
        if p is None:
            return np.zeros(3)
        return np.abs(np.array(p[1:4]) - home)

    def check() -> bool:
        nonlocal boundary_hit, abort_reason
        st = node.state_now()
        if st not in ("READY", ""):
            abort_reason = f"state={st}"
            return False
        dev = measured_dev()
        if np.any(dev >= hard):
            boundary_hit = 1
            abort_reason = f"触边界 {np.round(dev * 1000, 1)} mm"
            return False
        return True

    while True:
        elapsed = time.monotonic() - t_start
        if elapsed >= args.duration:
            abort_reason = "达到时长"
            break
        if not check():
            break

        # 选轴：优先选离软边界最远的轴；方向偏向反转上一段
        dev = measured_dev()
        room = soft - dev
        candidates = [i for i in range(3) if room[i] > args.min_step]
        if not candidates:
            abort_reason = "无可用运动余量（到软边界）"
            break
        weights = [max(1e-3, room[i]) for i in candidates]
        ai = rng.choices(candidates, weights=weights, k=1)[0]
        prev = last_dir.get(ai, 0.0)
        sgn = -prev if (prev != 0.0 and rng.random() < 0.7) else rng.choice((-1.0, 1.0))
        speed = rng.uniform(0.01, args.vmax)
        # 这一步最多能走多远（留 1cm 余量），时长限制在 0.6–1.6s
        allowed = max(0.0, room[ai] - 0.01)
        dur = min(rng.uniform(0.6, 1.6), allowed / speed)
        if dur < args.min_step / speed:
            # 该方向放不下最小步长 → 换反方向
            sgn = -sgn
            dur = min(rng.uniform(0.6, 1.6), allowed / speed)
            if dur < args.min_step / speed:
                abort_reason = "余量不足最小步长"
                break

        vec = np.zeros(3)
        vec[ai] = sgn * speed
        if prev != 0.0 and sgn != prev:
            reversals += 1
        last_dir[ai] = sgn
        node.set_twist(*vec)
        steps += 1
        ok = node.spin_for(dur, check=check)
        node.stop()
        # 松手后：0.6s 内的滑行（含节点内平滑减速）+ 再 0.5s 的停稳漂移
        p0 = node.ee_now()
        node.spin_for(0.6, check=check)
        p1 = node.ee_now()
        node.spin_for(0.5, check=check)
        p2 = node.ee_now()
        if p0 is not None and p1 is not None and p2 is not None:
            stop_travel.append(float(np.linalg.norm(np.array(p1[1:4]) - np.array(p0[1:4]))))
            settle_drift.append(float(np.linalg.norm(np.array(p2[1:4]) - np.array(p1[1:4]))))
        d = measured_dev()
        for i in range(3):
            if d[i] > max_dev[i]:
                max_dev[i] = d[i]
        # 暂停 0.15–0.35s（制造 正→0）
        node.spin_for(rng.uniform(0.15, 0.35), check=check)
        if not ok and abort_reason:
            break

    node.stop()
    node.spin_for(0.6)
    t_run1 = node.get_clock().now().nanoseconds * 1e-9

    # ---- 指标 ----
    cmd_pos, _ = node.cmd_ee()
    cmd_pos = in_range(cmd_pos, t_run0 - 0.1, t_run1 + 0.1)
    meas = in_range(node.measured_ee(), t_run0 - 0.1, t_run1 + 0.1)
    gaps = int((np.diff(cmd_pos[:, 0]) > 0.02).sum()) if len(cmd_pos) > 1 else 0
    cmd_rate = float("nan")
    if len(cmd_pos) > 10:
        span = cmd_pos[-1, 0] - cmd_pos[0, 0]
        cmd_rate = (len(cmd_pos) - 1) / span if span > 0 else float("nan")
    if len(cmd_pos) and len(meas):
        err = ee_track_error(cmd_pos, meas)
        err_samples = err.tolist()
    rms = float(np.sqrt(np.mean(np.square(err_samples)))) if err_samples else float("nan")
    emax = float(np.max(err_samples)) if err_samples else float("nan")

    # 命令关节速度逐拍变化（平滑性）：节点每拍给的 qd 直接差分，|Δq̇| 应 ≤ max_jerk*dt = 0.5 rad/s
    max_dqd = float("nan")
    with node.lock:
        rows = [r for r in node.cmd if t_run0 - 0.1 <= r[0] <= t_run1 + 0.1]
    if len(rows) > 2:
        qd = np.array([r[2] for r in rows])
        max_dqd = float(np.max(np.abs(np.diff(qd, axis=0))))

    run_time = time.monotonic() - t_start
    stop_max = max(stop_travel) if stop_travel else float("nan")
    settle_max = max(settle_drift) if settle_drift else float("nan")
    rate_ok = (not math.isnan(cmd_rate)) and cmd_rate > 97.0
    ok = (rms < RANDOM_MAX_RMS_ERR and settle_max < RANDOM_MAX_SETTLE_DRIFT
          and rate_ok and boundary_hit == 0)

    log(f"\n  运行 {run_time:.1f}s，步数 {steps}，方向反转 {reversals} 次")
    log(f"  最大偏离(mm)：x={max_dev[0] * 1000:.1f}  y={max_dev[1] * 1000:.1f}  z={max_dev[2] * 1000:.1f}")
    log(f"  跟踪误差 RMS {rms * 1000:.1f} mm / 最大 {emax * 1000:.1f} mm")
    log(f"  松手 0.6s 滑行最大 {stop_max * 1000:.2f} mm；随后 0.5s 漂移最大 {settle_max * 1000:.2f} mm")
    log(f"  命令关节 |Δq̇| 最大 {max_dqd:.3f} rad/s；命令流 {cmd_rate:.1f} Hz（批抖动 {gaps} 处）")
    log(f"  触边界 {boundary_hit}；停止原因：{abort_reason}")
    log(f"  >>> random {'PASS' if ok else 'FAIL'}"
        f"（阈值：RMS<{RANDOM_MAX_RMS_ERR * 1000:.0f}mm、停稳漂移<{RANDOM_MAX_SETTLE_DRIFT * 1000:.0f}mm、"
        f"命令流>97Hz、不越界）")

    return dict(mode="random", vmax=args.vmax, duration_s=run_time, steps=steps,
                reversals=reversals, max_dev_mm=(max_dev * 1000).tolist(),
                rms_err_mm=rms * 1000, max_err_mm=emax * 1000,
                stop_travel_mm=stop_max * 1000, settle_drift_mm=settle_max * 1000,
                max_dqd=max_dqd, gap_count=gaps, cmd_rate_hz=cmd_rate,
                boundary_hit=boundary_hit, abort_reason=abort_reason,
                pass_=bool(ok), home=node.home.tolist())


# ------------------------------------------------- random --continuous
def run_random_continuous(node: ServoTrack, args: argparse.Namespace) -> dict:
    """连续随机速度跟踪（OU 过程），更贴近人脸跟踪：中途不停、方向连续变化。"""
    rng = random.Random(args.seed)
    home = node.home
    hard = args.bound
    soft = hard * 0.7
    axes = [i for i, a in enumerate("xyz") if a in args.axes]
    sigma = args.sigma if args.sigma > 0 else 0.8 * args.vmax
    dt = 0.02
    log(f"[*] random --continuous：vmax={args.vmax:.3f} m/s，{args.duration:.0f}s，"
        f"边界 home±{hard * 100:.0f}cm，τ={args.tau:.2f}s，accel={args.accel:.2f} m/s²，轴={args.axes}")

    t_start = time.monotonic()
    t_run0 = node.get_clock().now().nanoseconds * 1e-9
    v = np.zeros(3)
    v_prev = np.zeros(3)
    speeds = []
    reversals = 0
    last_sign = {i: 0 for i in axes}
    max_dev = np.zeros(3)
    boundary_hit = 0
    abort_reason = ""
    ticks = 0

    def signed_dev() -> np.ndarray:
        p = node.ee_now()
        if p is None:
            return np.zeros(3)
        return np.array(p[1:4]) - home

    def check() -> bool:
        nonlocal boundary_hit, abort_reason
        st = node.state_now()
        if st not in ("READY", ""):
            abort_reason = f"state={st}"
            return False
        dev = np.abs(signed_dev())
        if np.any(dev >= hard):
            boundary_hit = 1
            abort_reason = f"触边界 {np.round(dev * 1000, 1)} mm"
            return False
        return True

    while True:
        elapsed = time.monotonic() - t_start
        if elapsed >= args.duration:
            abort_reason = "达到时长"
            break
        if not check():
            break
        # OU 过程：速度本身平滑变化，且会自然过零换向
        noise = np.array([rng.gauss(0.0, 1.0) for _ in range(3)])
        v = v - (v / args.tau) * dt + sigma * math.sqrt(dt) * noise
        for i in range(3):
            if i not in axes:
                v[i] = 0.0
        # 接近软边界时叠加回中速度
        dev = signed_dev()
        for i in axes:
            over = abs(dev[i]) - soft
            if over > 0.0:
                v[i] -= math.copysign(1.0, dev[i]) * (over / max(1e-3, hard - soft)) * 2.0 * args.vmax
        # 加速度限幅（与节点内 twist 平滑叠乘，保证视觉上不会突跳）
        v = v_prev + np.clip(v - v_prev, -args.accel * dt, args.accel * dt)
        n = float(np.linalg.norm(v))
        if n > args.vmax and n > 1e-9:
            v *= args.vmax / n
        node.set_twist(*v)
        v_prev = v.copy()
        ticks += 1
        speeds.append(float(np.linalg.norm(v)))
        for i in axes:
            s = 1 if v[i] > 0.005 else (-1 if v[i] < -0.005 else 0)
            if s != 0 and last_sign[i] != 0 and s != last_sign[i]:
                reversals += 1
            if s != 0:
                last_sign[i] = s
        d = np.abs(signed_dev())
        max_dev = np.maximum(max_dev, d)
        if not node.spin_for(dt, check=check):
            break

    node.stop()
    p0 = node.ee_now()
    node.spin_for(0.6)
    p1 = node.ee_now()
    node.spin_for(0.5)
    p2 = node.ee_now()
    stop_max = settle_max = float("nan")
    if p0 is not None and p1 is not None and p2 is not None:
        stop_max = float(np.linalg.norm(np.array(p1[1:4]) - np.array(p0[1:4])))
        settle_max = float(np.linalg.norm(np.array(p2[1:4]) - np.array(p1[1:4])))
    t_run1 = node.get_clock().now().nanoseconds * 1e-9

    cmd_pos, _ = node.cmd_ee()
    cmd_pos = in_range(cmd_pos, t_run0 - 0.1, t_run1 + 0.1)
    meas = in_range(node.measured_ee(), t_run0 - 0.1, t_run1 + 0.1)
    gaps = int((np.diff(cmd_pos[:, 0]) > 0.02).sum()) if len(cmd_pos) > 1 else 0
    cmd_rate = float("nan")
    if len(cmd_pos) > 10:
        span = cmd_pos[-1, 0] - cmd_pos[0, 0]
        cmd_rate = (len(cmd_pos) - 1) / span if span > 0 else float("nan")
    err = ee_track_error(cmd_pos, meas) if (len(cmd_pos) and len(meas)) else np.zeros(0)
    rms = float(np.sqrt(np.mean(np.square(err)))) if len(err) else float("nan")
    emax = float(np.max(err)) if len(err) else float("nan")
    max_dqd = float("nan")
    with node.lock:
        rows = [r for r in node.cmd if t_run0 - 0.1 <= r[0] <= t_run1 + 0.1]
    if len(rows) > 2:
        qd = np.array([r[2] for r in rows])
        max_dqd = float(np.max(np.abs(np.diff(qd, axis=0))))

    run_time = time.monotonic() - t_start
    mean_speed = float(np.mean(speeds)) if speeds else float("nan")
    rate_ok = (not math.isnan(cmd_rate)) and cmd_rate > 97.0
    ok = (rms < RANDOM_MAX_RMS_ERR and settle_max < RANDOM_MAX_SETTLE_DRIFT
          and rate_ok and boundary_hit == 0)

    log(f"\n  运行 {run_time:.1f}s，控制拍 {ticks}，平均速度 {mean_speed * 1000:.1f} mm/s，"
        f"方向反转 {reversals} 次")
    log(f"  最大偏离(mm)：x={max_dev[0] * 1000:.1f}  y={max_dev[1] * 1000:.1f}  z={max_dev[2] * 1000:.1f}")
    log(f"  跟踪误差 RMS {rms * 1000:.1f} mm / 最大 {emax * 1000:.1f} mm")
    log(f"  松手 0.6s 滑行 {stop_max * 1000:.2f} mm；随后 0.5s 漂移 {settle_max * 1000:.2f} mm")
    log(f"  命令关节 |Δq̇| 最大 {max_dqd:.3f} rad/s；命令流 {cmd_rate:.1f} Hz（批抖动 {gaps} 处）")
    log(f"  触边界 {boundary_hit}；停止原因：{abort_reason}")
    log(f"  >>> random(continuous) {'PASS' if ok else 'FAIL'}"
        f"（阈值：RMS<{RANDOM_MAX_RMS_ERR * 1000:.0f}mm、停稳漂移<{RANDOM_MAX_SETTLE_DRIFT * 1000:.0f}mm、"
        f"命令流>97Hz、不越界）")

    return dict(mode="random_continuous", vmax=args.vmax, tau=args.tau,
                accel=args.accel, axes=args.axes, duration_s=run_time,
                ticks=ticks, mean_speed_mm_s=mean_speed * 1000, reversals=reversals,
                max_dev_mm=(max_dev * 1000).tolist(),
                rms_err_mm=rms * 1000, max_err_mm=emax * 1000,
                stop_travel_mm=stop_max * 1000, settle_drift_mm=settle_max * 1000,
                max_dqd=max_dqd, gap_count=gaps, cmd_rate_hz=cmd_rate,
                boundary_hit=boundary_hit, abort_reason=abort_reason,
                pass_=bool(ok), home=node.home.tolist())


def main() -> int:
    ap = argparse.ArgumentParser(description="F181 servo 真机跟踪验收")
    sub = ap.add_subparsers(dest="mode", required=True)

    sp = sub.add_parser("sweep", help="三轴 +10 → -20 → +10 cm")
    sp.add_argument("--v", type=float, default=0.03, help="速度 m/s（默认 0.03）")
    sp.add_argument("--axis", default="xyz", help="轴子集，如 xyz/xy/x")

    rp = sub.add_parser("random", help="随机方形跟踪，60s 或触边界先到先停")
    rp.add_argument("--vmax", type=float, default=0.05, help="最大速度 m/s（默认 0.05）")
    rp.add_argument("--duration", type=float, default=60.0, help="累计运行秒数（默认 60）")
    rp.add_argument("--bound", type=float, default=0.10, help="home 边界 m（默认 0.10）")
    rp.add_argument("--seed", type=int, default=None, help="随机种子")
    rp.add_argument("--min-step", type=float, default=0.005, dest="min_step",
                    help="单步最小行程 m（默认 0.005，仅分段模式）")
    rp.add_argument("--continuous", action="store_true",
                    help="连续随机速度（OU 过程，中途不停，贴近人脸跟踪）")
    rp.add_argument("--tau", type=float, default=0.8, help="OU 相关时间 s（默认 0.8，仅 --continuous）")
    rp.add_argument("--accel", type=float, default=0.8, help="速度加速度限幅 m/s²（默认 0.8，仅 --continuous）")
    rp.add_argument("--sigma", type=float, default=0.0, help="OU 强度 m/s^1.5（默认 0.8*vmax）")
    rp.add_argument("--axes", default="xyz", help="连续模式参与轴，如 xyz/xy（默认 xyz）")

    ap.add_argument("--twist-topic", default="/servo_node/delta_twist_cmds")
    ap.add_argument("--desc-pkg", default="a3_description_6j")
    ap.add_argument("--urdf", default="", help="命令侧 FK 的 URDF（默认按 desc-pkg 解析）")
    ap.add_argument("--out-dir", default="", help="输出目录（默认 ~/.a3/servo_track/<时间戳>）")
    ap.add_argument("--no-home", action="store_true", help="跳过起止回 home（调试用）")
    args = ap.parse_args()
    if args.mode == "random" and args.continuous:
        if not any(a in "xyz" for a in args.axes):
            log("[x] --axes 至少要含 x/y/z 之一")
            return 2

    out_dir = args.out_dir or os.path.join(
        os.path.expanduser("~/.a3/servo_track"), datetime.now().strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    node = ServoTrack(args)
    rc = 0
    home_reached = False
    result = {"mode": args.mode, "error": "not started"}
    try:
        if not node.wait_ready(5.0):
            log("[x] arm_status 不是 READY（先 PS4 L3 使能或 /a3/arm/enable）")
            return 2
        if not node.wait_ee(5.0):
            log("[x] 收不到 /a3/ee_pose（需统一栈 / use_servo_target 起栈）")
            return 2
        if not args.no_home:
            if not node.goto_home():
                return 2
            home_reached = True
        else:
            p = node.ee_now()
            node.home = np.array(p[1:4]) if p else np.zeros(3)
        if args.mode == "sweep":
            result = run_sweep(node, args)
        elif getattr(args, "continuous", False):
            result = run_random_continuous(node, args)
        else:
            result = run_random(node, args)
        if home_reached and not args.no_home:
            node.goto_home()
        rc = 0 if result.get("pass_") else 1
    except KeyboardInterrupt:
        log("[!] Ctrl-C")
        node.stop()
        rc = 130
    finally:
        node.stop()
        node.running = False
        path = os.path.join(out_dir, f"{args.mode}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        log(f"[+] 结果已保存：{path}")
        try:
            node.destroy_node()
        finally:
            # Ctrl-C 时 rclpy 的信号处理已 shutdown 过 context，重复调用会抛 RCLError
            if rclpy.ok():
                rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
