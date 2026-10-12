#!/usr/bin/env python3
"""模式×速度扫描轨迹的仿真/真机运行采集（跟踪误差 + 力矩）。

复用 f160 的采集思路：订阅 /arm_controller/state（跟踪误差）与 /a3/arm_status
（state/mode/力矩），仅在 TRAJ_RUNNING 运动段采集，输出全程指标 JSON。

用法（栈已启动，已 enable）：
  python3 scripts/a3_test/melodion_sweep_run.py --stream sweep --out /tmp/sweep_run.json

脚本会：goto 首键 → playback sweep → 等运动完成 → 输出 JSON。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from a3_msgs.msg import ArmStatus
from a3_msgs.srv import GotoNamedPose, PlaybackTrajectory

N_ARM = 6


class SweepRunner(Node):
    def __init__(self):
        super().__init__("melodion_sweep_run")
        self._state = ""
        self._mode = ""
        self.capture = False
        self.err_samples = []
        self.pd_samples = []
        self.eff_samples = []
        self._last_eff = None
        self.create_subscription(
            JointTrajectoryControllerState, "/arm_controller/state", self._on_state, 10)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_status, 10)
        self.goto_cli = self.create_client(GotoNamedPose, "/a3/arm/goto_named_pose")
        self.playback_cli = self.create_client(PlaybackTrajectory, "/a3/arm/playback")

    def _on_status(self, msg):
        self._state = msg.state
        self._mode = msg.mode
        if not self.capture or msg.mode != "TRAJ_RUNNING":
            return
        pd = msg.pd_feedback[:N_ARM]
        self.pd_samples.append(max((abs(p) for p in pd), default=0.0))
        eff = msg.efforts[:N_ARM]
        if self._last_eff is not None:
            self.eff_samples.extend(
                abs(a - b) for a, b in zip(eff, self._last_eff))
        self._last_eff = eff

    def _on_state(self, msg):
        if not self.capture:
            return
        err = msg.error.positions[:N_ARM]
        if not err:
            return
        self.err_samples.append({
            "max_abs_err": max((abs(e) for e in err), default=0.0),
            "rms_err": math.sqrt(sum(e * e for e in err) / len(err)),
        })

    def call_goto(self, name):
        if not self.goto_cli.wait_for_service(timeout_sec=5):
            return False, "goto svc unavailable"
        req = GotoNamedPose.Request()
        req.pose_name = name
        fut = self.goto_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        r = fut.result()
        return (bool(r.success), r.message) if r else (False, "timeout")

    def call_playback(self, name):
        if not self.playback_cli.wait_for_service(timeout_sec=5):
            return False, "playback svc unavailable"
        req = PlaybackTrajectory.Request()
        req.name = name
        req.type = "stream"
        fut = self.playback_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=600)
        r = fut.result()
        return (bool(r.success), r.message) if r else (False, "timeout")

    def wait_state(self, target, timeout, desc):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._state in target:
                return True
            if self._state == "FAULT":
                return False
        return False

    def wait_done(self, timeout, desc):
        deadline = time.monotonic() + timeout
        started = False
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._mode == "TRAJ_RUNNING" or self._state == "TRAJ":
                started = True
                break
        if not started:
            return self._state == "READY"
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._mode != "TRAJ_RUNNING" and self._state == "READY":
                return True
            if self._state == "FAULT":
                return False
        return False


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stream", default="sweep")
    ap.add_argument("--start", default="melodion_F3_up")
    ap.add_argument("--out", default="/tmp/sweep_run.json")
    args = ap.parse_args()

    rclpy.init()
    node = SweepRunner()
    rclpy.spin_once(node, timeout_sec=0.5)

    ok, msg = node.call_goto(args.start)
    if not ok:
        print(f"[x] goto {args.start} 失败: {msg}")
        sys.exit(1)
    if not node.wait_done(60, "goto start"):
        print("[x] goto 超时/FAULT")
        sys.exit(1)
    print(f"[+] goto {args.start} 完成")

    t0 = time.monotonic()
    node.capture = True
    ok, msg = node.call_playback(args.stream)
    if not ok:
        print(f"[x] playback 失败: {msg}")
        node.capture = False
        sys.exit(1)
    done = node.wait_done(600, "playback")
    node.capture = False
    elapsed = time.monotonic() - t0

    n = len(node.err_samples)
    peak = max((s["max_abs_err"] for s in node.err_samples), default=0.0)
    rms = math.sqrt(
        sum(s["rms_err"] ** 2 for s in node.err_samples) / n) if n else 0.0
    pd_peak = max(node.pd_samples, default=0.0)
    eff_jitter = (sum(node.eff_samples) / len(node.eff_samples)) if node.eff_samples else 0.0

    result = {
        "stream": args.stream,
        "done": done,
        "wall_time_s": round(elapsed, 1),
        "n_err_samples": n,
        "peak_err": peak,
        "rms_err": rms,
        "pd_peak": pd_peak,
        "eff_jitter": eff_jitter,
        "final_state": node._state,
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))
    print(f"已写入 {args.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
