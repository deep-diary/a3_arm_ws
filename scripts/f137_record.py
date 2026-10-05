#!/usr/bin/env python3
"""F137 A/B 真机录制：订阅 /joint_states，触发 playback，录制并算平滑度指标。

用法：
  python3 scripts/f137_record.py --name latest --duration 40 --out /tmp/ab.csv

录制 positions 与 velocities 两套。指标优先用 **velocity 字段** 求加速度/加加速度
（只差分一次，绕开位置差分二次求导对量化噪声的放大）；同时打印位置差分结果作对照，
velocity 字段缺失时自动回退位置差分。
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a3_msgs.srv import PlaybackTrajectory  # noqa: E402


class RecordNode(Node):
    def __init__(self, name: str, duration: float, out_csv: str, skip_start: float = 0.0):
        super().__init__("f137_recorder")
        self._name = name
        self._duration = duration
        self._out = out_csv
        self._skip_start = skip_start
        self._rows = []  # (t, positions[7], velocities[7])
        self._t0 = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub = self.create_subscription(
            JointState, "/joint_states", self._on_js, qos)
        self._cli = self.create_client(PlaybackTrajectory, "/a3/arm/playback")

    def _on_js(self, msg: JointState) -> None:
        if self._t0 is None:
            return
        t = time.monotonic() - self._t0
        pos = [float(v) for v in msg.position]
        vel = [float(v) for v in msg.velocity] if msg.velocity else [0.0] * len(pos)
        self._rows.append((t, pos, vel))

    def _trigger(self) -> None:
        req = PlaybackTrajectory.Request()
        req.name = self._name
        req.type = ""
        req.strategy = ""
        if not self._cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("playback service 不可用")
            return
        self._cli.call_async(req)

    def run(self) -> None:
        self._trigger()
        self._t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - self._t0) < self._duration:
            rclpy.spin_once(self, timeout_sec=0.05)

        times = [r[0] for r in self._rows]
        pos = [r[1] for r in self._rows]
        vel = [r[2] for r in self._rows]
        n_j = len(pos[0]) if pos else 0
        with open(self._out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t"] + [f"q{i}" for i in range(n_j)] + [f"v{i}" for i in range(n_j)])
            for i in range(len(pos)):
                w.writerow([times[i]] + pos[i] + vel[i])
        self.get_logger().info(f"recorded {len(pos)} samples -> {self._out}")

        t2, p2, v2 = _trim_motion(times, pos, vel, skip_start=self._skip_start)
        if p2 and len(p2) >= 3:
            print(f"\n[{self._name}] 运动段 {len(p2)}/{len(pos)} 样本, "
                  f"时长 {t2[-1]-t2[0]:.2f}s")
            mp = _metrics_positions(t2, p2)
            mv = _metrics_velocities(t2, v2)
            print(f"  [velocity字段] v_rms={mv['v_rms']:.4f} a_rms={mv['a_rms']:.4f} "
                  f"j_rms={mv['j_rms']:.4f} a_peak={mv['a_peak']:.4f} "
                  f"j_peak={mv['j_peak']:.4f}")
            print(f"  [位置差分对照] v_rms={mp['v_rms']:.4f} a_rms={mp['a_rms']:.4f} "
                  f"j_rms={mp['j_rms']:.4f} a_peak={mp['a_peak']:.4f} "
                  f"j_peak={mp['j_peak']:.4f}")
        else:
            print(f"\n[{self._name}] 未捕获到足够运动样本")


def _trim_motion(times, pos, vel, vel_thresh: float = 0.05, skip_start: float = 0.0):
    if len(pos) < 3:
        return times, pos, vel
    p = np.asarray(pos, dtype=float)
    t = np.asarray(times, dtype=float)
    v = np.gradient(p, t, axis=0)
    speed = np.abs(v).max(axis=1)
    # 0.2s 滑动平均压掉单样本噪声尖峰，再找运动起止
    kern = np.ones(40) / 40.0
    ss = np.convolve(speed, kern, mode="same")
    idx = np.flatnonzero(ss > vel_thresh)
    if len(idx) == 0:
        return times, pos, vel
    lo, hi = int(idx[0]), int(idx[-1])
    lo = max(0, lo - 10)
    hi = min(len(times) - 1, hi + 10)
    # 排除开头 skip_start 秒（return 回首段 + 录制起始瞬态/样本突发）
    if skip_start > 0:
        lo = max(lo, int(np.searchsorted(times, times[lo] + skip_start)))
    return times[lo:hi + 1], pos[lo:hi + 1], vel[lo:hi + 1]


def _metrics_positions(times, pos):
    p = np.asarray(pos, dtype=float)
    t = np.asarray(times, dtype=float)
    v = np.gradient(p, t, axis=0)
    a = np.gradient(v, t, axis=0)
    j = np.gradient(a, t, axis=0)
    return {
        "v_rms": float(np.sqrt((v ** 2).mean())),
        "a_rms": float(np.sqrt((a ** 2).mean())),
        "j_rms": float(np.sqrt((j ** 2).mean())),
        "a_peak": float(np.abs(a).max()),
        "j_peak": float(np.abs(j).max()),
    }


def _metrics_velocities(times, vel):
    v = np.asarray(vel, dtype=float)
    t = np.asarray(times, dtype=float)
    a = np.gradient(v, t, axis=0)
    j = np.gradient(a, t, axis=0)
    return {
        "v_rms": float(np.sqrt((v ** 2).mean())),
        "a_rms": float(np.sqrt((a ** 2).mean())),
        "j_rms": float(np.sqrt((j ** 2).mean())),
        "a_peak": float(np.abs(a).max()),
        "j_peak": float(np.abs(j).max()),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="F137 A/B 真机录制")
    ap.add_argument("--name", required=True)
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--skip-start", type=float, default=0.0,
                    help="运动段开头排除多少秒（排除 return 回首段与录制起始瞬态，如 6.0）")
    args = ap.parse_args()

    rclpy.init()
    node = RecordNode(args.name, args.duration, args.out, args.skip_start)
    node.run()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
