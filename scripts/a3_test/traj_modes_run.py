#!/usr/bin/env python3
"""多路点轨迹模式基准对比（在线：运行 + 跟踪误差采集）。

对每种模式×速度运行轨迹并采集跟踪误差（/arm_controller/state 的 error）+ 力矩。
触发方式：
  - l2/l3/l1p5：离线生成命令轨迹 → 写 continuous YAML → playback type=continuous
  - ptp：goto_named_pose 逐点
  - l1：playback type=stream（mode=lin，pilz blend，需先生成 stream 文件）

用法（栈已启动，已 enable）：
  python3 scripts/a3_test/traj_modes_run.py \
      --config src/a3_arm_controller/config/traj_modes_bench.yaml --out /tmp/traj_modes_run.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from a3_msgs.msg import ArmStatus
from a3_msgs.srv import GotoNamedPose, PlaybackTrajectory

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, WS)
sys.path.insert(0, os.path.join(WS, "src", "a3_arm_controller", "a3_arm_controller"))
sys.path.insert(0, os.path.join(WS, "scripts"))

import yaml  # noqa: E402
from arm_controller import (  # noqa: E402
    _bell_timed_traj,
    _interpolate_via_spline,
    _jerk_limited_timed_traj,
    _trapezoidal_timed_traj,
)
from traj_modes_bench import gen_offline, load_named_pose_joints  # noqa: E402

JOINTS = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint", "L7_joint"]
TRAJ_DIR = os.path.join(WS, "trajectories")


class Runner(Node):
    def __init__(self):
        super().__init__("traj_modes_run")
        self._state = ""
        self._mode = ""
        self.capture = False
        self.err_samples: List[dict] = []
        self.create_subscription(
            JointTrajectoryControllerState, "/arm_controller/state", self._on_state, 10)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_status, 10)
        self.goto_cli = self.create_client(GotoNamedPose, "/a3/arm/goto_named_pose")
        self.playback_cli = self.create_client(PlaybackTrajectory, "/a3/arm/playback")

    def _on_status(self, msg):
        self._state = msg.state
        self._mode = msg.mode

    def _on_state(self, msg):
        if not self.capture or self._mode != "TRAJ_RUNNING":
            return
        err = msg.error.positions[:6]
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

    def call_playback(self, name, type_):
        if not self.playback_cli.wait_for_service(timeout_sec=5):
            return False, "playback svc unavailable"
        req = PlaybackTrajectory.Request()
        req.name = name
        req.type = type_
        fut = self.playback_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=120)
        r = fut.result()
        return (bool(r.success), r.message) if r else (False, "timeout")

    def wait_done(self, timeout):
        deadline = time.monotonic() + timeout
        started = False
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._mode == "TRAJ_RUNNING" or self._state == "TRAJ":
                started = True
                break
            if self._state == "FAULT":
                return False
        if not started:
            return self._state == "READY"
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._mode != "TRAJ_RUNNING" and self._state == "READY":
                return True
            if self._state == "FAULT":
                return False
        return False

    def collect_metrics(self):
        n = len(self.err_samples)
        peak = max((s["max_abs_err"] for s in self.err_samples), default=0.0)
        rms = math.sqrt(
            sum(s["rms_err"] ** 2 for s in self.err_samples) / n) if n else 0.0
        return {"n_err_samples": n, "peak_err": peak, "rms_err": rms}


def write_continuous_yaml(name: str, times, pos) -> str:
    os.makedirs(TRAJ_DIR, exist_ok=True)
    path = os.path.join(TRAJ_DIR, name + ".yaml")
    data = {
        "joint_names": list(JOINTS),
        "points": [
            {"positions": [float(v) for v in p] + [0.0],
             "time_from_start_sec": float(t)}
            for t, p in zip(times, pos)
        ],
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--named-poses", default=None)
    ap.add_argument("--out", default="/tmp/traj_modes_run.json")
    args = ap.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if args.named_poses:
        np_path = args.named_poses
    else:
        from ament_index_python.packages import get_package_share_directory
        np_path = os.path.join(
            get_package_share_directory("a3_description"), "config", "named_poses.yaml")

    points = [load_named_pose_joints(np_path, p) for p in cfg["points"]]
    if any(p is None for p in points):
        print(f"命名位姿缺失", file=sys.stderr)
        sys.exit(2)
    vmax = float(cfg.get("vmax_rad_s", 1.5))
    amax = float(cfg.get("amax_rad_s2", 15.0))
    modes = cfg["modes"]
    speeds = [float(s) for s in cfg["speeds"]]

    rclpy.init()
    node = Runner()
    rclpy.spin_once(node, timeout_sec=0.5)

    results = []
    for mode in modes:
        for speed in speeds:
            v = vmax * speed
            a = amax * speed
            entry = {"mode": mode, "speed": speed, "done": False,
                     "peak_err": None, "rms_err": None, "msg": ""}
            node.capture = False
            node.err_samples = []

            if mode in ("l2", "l3", "l1p5"):
                timed = gen_offline(mode, points, v, a)
                if timed is None:
                    entry["msg"] = "gen failed"
                    results.append(entry)
                    continue
                times, pos = timed
                name = f"tm_bench_{mode}_{int(speed * 100)}"
                write_continuous_yaml(name, times, pos)
                node.capture = True
                ok, msg = node.call_playback(name, "continuous")
                done = node.wait_done(120)
                node.capture = False
                entry.update(node.collect_metrics())
                entry["done"] = done
                entry["msg"] = msg
            elif mode == "ptp":
                node.capture = True
                ok = True
                for p in cfg["points"]:
                    ok2, msg2 = node.call_goto(p)
                    ok = ok and ok2
                    if not ok2:
                        break
                done = node.wait_done(60)
                node.capture = False
                entry.update(node.collect_metrics())
                entry["done"] = done and ok
                entry["msg"] = msg2 if not ok else "goto done"
            else:  # l1：跳过（需 stream mode=lin，待扩展）
                entry["msg"] = "l1 skipped (needs stream mode=lin)"
            results.append(entry)
            print(f"[{mode} speed={speed:.1f}] done={entry['done']} "
                  f"peak_err={entry['peak_err']}")

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"结果 -> {args.out}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
