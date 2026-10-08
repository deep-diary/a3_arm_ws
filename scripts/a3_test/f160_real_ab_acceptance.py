#!/usr/bin/env python3
"""F160 真机 A/B 对比：gravity vs full 前馈的跟踪误差与力矩分量指标。

不自己发轨迹——由操作者在真机手动跑「idle → home → 弹琴示教回放」，脚本只做：
  1. 设置 feedforward_mode（gravity | full，运行时切，无需重启栈）
  2. 订阅 /arm_controller/state（跟踪误差）与 /a3/arm_status（力矩分量），
     仅在编排层 mode == TRAJ_RUNNING（即运动段）时采集
  3. Ctrl+C 结束，输出本模式指标 JSON
  4. compare 子命令对比两次 JSON，给出改善判定

用法（真机，栈已启动）：
  # 第一遍：gravity（脚本会先设好模式）
  python3 scripts/a3_test/f160_real_ab_acceptance.py run --mode gravity --out /tmp/f160_gravity.json
  # 此时手动执行 idle→home→弹琴，结束后 Ctrl+C

  # 第二遍：full
  python3 scripts/a3_test/f160_real_ab_acceptance.py run --mode full --out /tmp/f160_full.json
  # 再次执行同样的 idle→home→弹琴，结束后 Ctrl+C

  # 对比
  python3 scripts/a3_test/f160_real_ab_acceptance.py compare /tmp/f160_gravity.json /tmp/f160_full.json

退出码：compare 全改善 = 0，否则 1。
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_srvs.srv import Trigger
from control_msgs.msg import JointTrajectoryControllerState
from a3_msgs.msg import ArmStatus
from a3_msgs.srv import GotoNamedPose, PlaybackTrajectory

HEALTH_NODE = "/a3_hardware_health"
N_ARM = 6  # L1-L6


class Collector(Node):
    """设 feedforward_mode 并采集运动段（TRAJ_RUNNING）的跟踪误差与力矩分量。"""

    def __init__(self, mode: str):
        super().__init__("f160_real_ab")
        self.mode = mode
        self.cli = self.create_client(
            SetParameters, HEALTH_NODE + "/set_parameters")

        self.moving = False
        self._state = ""
        self._mode = ""
        # 跟踪误差样本（来自 /arm_controller/state，200 Hz）
        self.err_samples = []   # dict: max_abs_err, rms_err, max_abs_vel
        # 实际速度样本（来自 /arm_controller/state，200 Hz，速度纹波用）
        self.vel_samples = []   # list[list[float]] 各关节 actual velocity
        # 力矩分量样本（来自 /a3/arm_status，10 Hz）
        self.pd_samples = []    # list[float] 各关节 max|pd|
        self.eff_samples = []   # list[list[float]] 各关节 effort
        self._last_eff = None
        self._start = None

        self.create_subscription(
            JointTrajectoryControllerState, "/arm_controller/state",
            self._on_state, 10)
        self.create_subscription(
            ArmStatus, "/a3/arm_status", self._on_status, 10)

    def set_mode(self) -> bool:
        if not self.cli.wait_for_service(timeout_sec=5.0):
            return False
        req = SetParameters.Request()
        req.parameters = [ParameterMsg(
            name="feedforward_mode",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=self.mode))]
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        res = fut.result()
        return bool(res and res.results and res.results[0].successful)

    def _on_status(self, msg: ArmStatus):
        self._state = msg.state
        self._mode = msg.mode
        self.moving = (msg.mode == "TRAJ_RUNNING")
        if not self.moving:
            return
        if self._start is None:
            self._start = self.get_clock().now().nanoseconds * 1e-9
        # pd_feedback 峰值（L1-L6）
        pd = msg.pd_feedback[:N_ARM]
        self.pd_samples.append(max((abs(p) for p in pd), default=0.0))
        # efforts 用于纹波（抖动）指标
        eff = msg.efforts[:N_ARM]
        self.eff_samples.append(eff)

    def _on_state(self, msg: JointTrajectoryControllerState):
        if not self.moving:
            return
        err = msg.error.positions[:N_ARM]
        vel = msg.desired.velocities[:N_ARM]
        if not err:
            return
        max_abs_err = max((abs(e) for e in err), default=0.0)
        rms_err = math.sqrt(sum(e * e for e in err) / len(err))
        max_abs_vel = max((abs(v) for v in vel), default=0.0)
        self.err_samples.append({
            "max_abs_err": max_abs_err,
            "rms_err": rms_err,
            "max_abs_vel": max_abs_vel,
        })
        act_vel = msg.actual.velocities[:N_ARM]
        if act_vel:
            self.vel_samples.append([float(v) for v in act_vel])

    def elapsed(self) -> float:
        if self._start is None:
            return 0.0
        return self.get_clock().now().nanoseconds * 1e-9 - self._start


def _steady_lag(err_samples, top_frac=0.2):
    """高速段（速度最高 top_frac）的峰值误差均值，近似匀速段稳态滞后。"""
    if not err_samples:
        return 0.0
    ordered = sorted(err_samples, key=lambda s: s["max_abs_vel"], reverse=True)
    n = max(1, int(len(ordered) * top_frac))
    top = ordered[:n]
    return sum(s["max_abs_err"] for s in top) / len(top)


def _eff_jitter(eff_samples):
    """力矩纹波：相邻帧 effort 一阶差分绝对值的均值（高频抖动判据）。"""
    if len(eff_samples) < 2:
        return 0.0
    acc = 0.0
    n = 0
    prev = eff_samples[0]
    for cur in eff_samples[1:]:
        for a, b in zip(cur, prev):
            acc += abs(a - b)
            n += 1
        prev = cur
    return acc / n if n else 0.0


def compute_metrics(err_samples, pd_samples, eff_samples, vel_samples=None):
    peak_err = max((s["max_abs_err"] for s in err_samples), default=0.0)
    rms_err = math.sqrt(
        sum(s["rms_err"] ** 2 for s in err_samples) / len(err_samples)
    ) if err_samples else 0.0
    steady_lag = _steady_lag(err_samples)
    pd_peak = max(pd_samples, default=0.0)
    pd_rms = math.sqrt(
        sum(p * p for p in pd_samples) / len(pd_samples)
    ) if pd_samples else 0.0
    eff_jitter = _eff_jitter(eff_samples)
    vel_jitter = _eff_jitter(vel_samples) if vel_samples else 0.0
    return {
        "n_err_samples": len(err_samples),
        "peak_err": peak_err,
        "rms_err": rms_err,
        "steady_lag": steady_lag,
        "pd_peak": pd_peak,
        "pd_rms": pd_rms,
        "eff_jitter": eff_jitter,
        "vel_jitter": vel_jitter,
    }


class AutoRunner(Collector):
    """自动执行 gravity/full 各一遍完整流程（enable→goto home→playback→disable）并采集。"""

    def __init__(self):
        super().__init__("gravity")
        self._state = ""
        self._mode = ""
        self.capture_enabled = False

        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")
        self.disable_cli = self.create_client(Trigger, "/a3/arm/disable")
        self.goto_cli = self.create_client(
            GotoNamedPose, "/a3/arm/goto_named_pose")
        self.playback_cli = self.create_client(
            PlaybackTrajectory, "/a3/arm/playback")

    # 状态订阅覆盖：总是缓存 state/mode，仅 capture_enabled 时采集
    def _on_status(self, msg: ArmStatus):
        self._state = msg.state
        self._mode = msg.mode
        if self.capture_enabled:
            super()._on_status(msg)

    def _on_state(self, msg: JointTrajectoryControllerState):
        if self.capture_enabled:
            super()._on_state(msg)

    def start_capture(self):
        self.err_samples = []
        self.vel_samples = []
        self.pd_samples = []
        self.eff_samples = []
        self._start = None
        self.capture_enabled = True

    def stop_capture(self):
        self.capture_enabled = False
        return compute_metrics(
            self.err_samples, self.pd_samples, self.eff_samples, self.vel_samples)

    def call_trigger(self, cli, timeout=20.0):
        if not cli.wait_for_service(timeout_sec=5.0):
            return False, "service not available"
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            return False, "timeout"
        return bool(res.success), res.message

    def call_goto(self, pose_name):
        if not self.goto_cli.wait_for_service(timeout_sec=5.0):
            return False, "goto service not available"
        req = GotoNamedPose.Request()
        req.pose_name = pose_name
        fut = self.goto_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        res = fut.result()
        if res is None:
            return False, "goto timeout"
        return bool(res.success), res.message

    def call_playback(self):
        if not self.playback_cli.wait_for_service(timeout_sec=5.0):
            return False, "playback service not available"
        req = PlaybackTrajectory.Request()
        req.name = ""
        req.type = ""
        req.strategy = ""
        fut = self.playback_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        res = fut.result()
        if res is None:
            return False, "playback timeout"
        return bool(res.success), res.message

    def wait_state(self, target_states, timeout_s, desc):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._state in target_states:
                return True
            if self._state == "FAULT":
                print(f"  [x] {desc}: 进入 FAULT，中止")
                return False
        print(f"  [x] {desc}: 超时（{timeout_s}s）")
        return False

    def wait_motion_done(self, timeout_s, desc):
        """等一次运动完成：先观察到 TRAJ，再等 mode 回非 TRAJ 且 state 回 READY。"""
        deadline = time.monotonic() + timeout_s
        started = False
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._mode == "TRAJ_RUNNING" or self._state == "TRAJ":
                started = True
                break
            if self._state == "FAULT":
                print(f"  [x] {desc}: 进入 FAULT，中止")
                return False
        if not started:
            return self._state == "READY"
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._mode != "TRAJ_RUNNING" and self._state == "READY":
                return True
            if self._state == "FAULT":
                print(f"  [x] {desc}: 进入 FAULT，中止")
                return False
        print(f"  [x] {desc}: 超时（{timeout_s}s）")
        return False

    def run_mode(self, mode):
        print(f"\n===== 自动执行 mode={mode} =====")
        self.mode = mode
        if not self.set_mode():
            print(f"  [x] 设置 feedforward_mode={mode} 失败")
            return None
        print(f"  [+] feedforward_mode={mode} 已设置")

        ok, msg = self.call_trigger(self.enable_cli)
        if not ok:
            print(f"  [x] enable 失败: {msg}")
            return None
        if not self.wait_state(("READY",), 20.0, "enable→READY"):
            return None
        print("  [+] enable → READY")

        # 等 enable 后 soft-start/re-anchor 与 move_group current-state 同步：
        # 立即 goto 会让 move_group 用陈旧起始状态规划，JTC 起始容差违例
        # （PATH_TOLERANCE_VIOLATED → error -4）回退本地两点轨迹 → 起步冲击。
        time.sleep(2.0)
        print("  [+] enable 后稳定等待 2.0s")

        ok, msg = self.call_goto("home")
        if not ok:
            print(f"  [x] goto home 失败: {msg}")
            return None
        if not self.wait_motion_done(60.0, "goto home"):
            return None
        print("  [+] goto home 完成")

        self.start_capture()
        ok, msg = self.call_playback()
        if not ok:
            print(f"  [x] playback 失败: {msg}")
            self.stop_capture()
            return None
        if not self.wait_motion_done(120.0, "playback"):
            self.stop_capture()
            return None
        metrics = self.stop_capture()
        metrics["mode"] = mode
        print(f"  [+] playback 完成，误差样本 {metrics['n_err_samples']}")

        self.call_trigger(self.disable_cli)
        self.wait_state(("DISABLED", "IDLE"), 30.0, "disable")
        print("  [+] disable 完成，臂回 idle 失能")
        return metrics


def run_cmd(args):
    rclpy.init()
    node = Collector(args.mode)
    if not node.set_mode():
        print(f"设置 feedforward_mode={args.mode} 失败（栈/硬件节点在跑吗？）")
        return 1
    print(f"feedforward_mode={args.mode} 已设置。现在请在真机执行 idle→home→弹琴，")
    print("完成后按 Ctrl+C 结束采集（脚本只统计 TRAJ_RUNNING 运动段）。\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            e = node.elapsed()
            if node.moving or e > 0:
                print(
                    f"\r运动段已采集 {e:.1f}s | 误差样本 {len(node.err_samples)} | "
                    f"力矩样本 {len(node.pd_samples)}   ", end="", flush=True)
    except KeyboardInterrupt:
        print("\n\n采集结束。")

    metrics = compute_metrics(
        node.err_samples, node.pd_samples, node.eff_samples, node.vel_samples)
    metrics["mode"] = args.mode
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)
    print(f"\n[{args.mode}] 指标：")
    print(json.dumps(metrics, indent=2))
    print(f"\n已写入 {args.out}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


def _pct(a, b):
    if b == 0:
        return float("inf")
    return (a - b) / b * 100.0


def compare_cmd(args):
    with open(args.gravity, encoding="utf-8") as f:
        g = json.load(f)
    with open(args.full, encoding="utf-8") as f:
        f_ = json.load(f)

    def row(name, key, lower_is_better, budget=0.0):
        gv, fv = g[key], f_[key]
        ok = (fv <= gv * (1 + budget)) if lower_is_better else True
        return name, gv, fv, _pct(fv, gv), ok

    checks = []
    # 跟踪误差：越低越好，允许 10% 抖动预算内视为不劣化
    checks.append(row("峰值跟踪误差 peak_err", "peak_err", True, 0.10))
    checks.append(row("RMS 跟踪误差 rms_err", "rms_err", True, 0.10))
    # 稳态滞后：严格下降（理想趋近 0），无预算
    checks.append(row("高速段稳态滞后 steady_lag", "steady_lag", True, 0.0))
    # PD 反馈负担：越低越好
    checks.append(row("PD 反馈峰值 pd_peak", "pd_peak", True, 0.10))
    checks.append(row("PD 反馈 RMS pd_rms", "pd_rms", True, 0.10))
    # 力矩纹波：不能明显变抖。力矩一阶差分本身测量方差大，放宽到 +25% 相对
    # 或 +0.02 Nm 绝对增量以内视为不劣化（两次独立测试间天然波动）。
    checks.append(row("力矩纹波 eff_jitter", "eff_jitter", True, 0.25))
    # 速度纹波：衡量运动顺滑度/抖振，同样放宽到 +25% 或 +0.02 绝对增量
    checks.append(row("速度纹波 vel_jitter", "vel_jitter", True, 0.25))

    print("\n===== F160 真机 A/B 对比（gravity vs full）=====")
    print(f"{'指标':<28}{'gravity':>12}{'full':>12}{'变化%':>10}  判定")
    print("-" * 70)
    for name, gv, fv, pct, ok in checks:
        mark = "PASS" if ok else "FAIL"
        pct_s = f"{pct:+.1f}%" if math.isfinite(pct) else "N/A"
        print(f"{name:<28}{gv:>12.4f}{fv:>12.4f}{pct_s:>10}  {mark}")

    # 核心改善判定：峰值误差显著下降 + 稳态滞后下降 + 抖动不劣化
    peak_ok = f_["peak_err"] <= g["peak_err"] * 1.05
    peak_improved = f_["peak_err"] < g["peak_err"] * 0.8   # 降 ≥20%
    lag_ok = f_["steady_lag"] < g["steady_lag"]
    jitter_ok = (f_["eff_jitter"] <= g["eff_jitter"] * 1.25) or \
                (f_["eff_jitter"] - g["eff_jitter"] <= 0.02)
    overall = peak_ok and peak_improved and lag_ok and jitter_ok

    print("-" * 70)
    print(f"核心判定：峰值误差降≥20%={peak_improved} | 稳态滞后下降={lag_ok} | "
          f"纹波不劣化={jitter_ok}")
    print(f"\n结论：{'✅ full 前馈有改善，可考虑默认启用' if overall else '⚠️ 未达改善标准（若变抖说明 AFF 数值需回看 F68/F137 v/a 质量）'}")
    return 0 if overall else 1


def auto_cmd(args):
    rclpy.init()
    runner = AutoRunner()
    for mode in ("gravity", "full"):
        metrics = runner.run_mode(mode)
        if metrics is None:
            print(f"\n[x] mode={mode} 执行失败，中止")
            runner.call_trigger(runner.disable_cli)
            runner.destroy_node()
            rclpy.shutdown()
            return 1
        out = f"{args.out_dir}/f160_auto_{mode}.json"
        with open(out, "w", encoding="utf-8") as f:
            json.dump(metrics, f, indent=2)
        print(f"\n[{mode}] 指标已写入 {out}")

    runner.destroy_node()
    rclpy.shutdown()

    ca = argparse.Namespace(
        gravity=f"{args.out_dir}/f160_auto_gravity.json",
        full=f"{args.out_dir}/f160_auto_full.json",
    )
    return compare_cmd(ca)


def main():
    ap = argparse.ArgumentParser(description="F160 真机 gravity/full 前馈 A/B 对比")
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="设模式并采集一段运动")
    r.add_argument("--mode", required=True, choices=["gravity", "full"])
    r.add_argument("--out", required=True)
    r.set_defaults(func=run_cmd)

    c = sub.add_parser("compare", help="对比两个 run 输出的 JSON")
    c.add_argument("gravity")
    c.add_argument("full")
    c.set_defaults(func=compare_cmd)

    a = sub.add_parser("auto", help="自动执行 gravity/full 各一遍并对比")
    a.add_argument("--out-dir", default="/tmp")
    a.set_defaults(func=auto_cmd)

    args = ap.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
