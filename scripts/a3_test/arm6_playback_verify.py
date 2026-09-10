#!/usr/bin/env python3
"""F38b 回放插值验证（通用 N 关节臂）：playback 前段应从当前位姿平滑插值到
首记录点，全程无阶跃、回放结束保持末位。

前置：can_bridge（generic gains）+ arm_controller（6j）运行中；teach6 轨迹已保存；
      臂停在任意位姿（与记录首点不同时插值效果明显）；mode=IDLE。

用法：
  python3 scripts/a3_test/arm6_playback_verify.py \
      [--traj ~/.a3/trajectories/teach6.yaml] [--ramp 2.5] \
      [--step-tol 0.06] [--pos-tol 0.15] [--start-tol 0.05]
"""

import argparse
import os
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
from common import Reporter, call_service  # noqa: E402
from a3_msgs.srv import PlaybackTrajectory  # noqa: E402
from a3_can_bridge.srv import MotorStop  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


class PlaybackVerifier(Node):
    def __init__(self, args):
        super().__init__("arm6_playback_verify")
        self.args = args
        self.latest = None  # {joint_name: pos}
        self.samples = []  # (t_rel, [pos_by_traj_names])
        # can_bridge 发布 /joint_states 为 BEST_EFFORT（对齐 F32 QoS 约定）
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, "/joint_states", self._on_js, qos)

    def _on_js(self, msg: JointState):
        self.latest = dict(zip(msg.name, msg.position))

    def _pos_by_names(self, names, data):
        return [float(data[n]) if n in data else float("nan") for n in names]

    def _emergency_stop(self):
        self.get_logger().error("emergency: motor/stop broadcast + arm/disable")
        call_service(self, MotorStop, "/a3/motor/stop",
                     MotorStop.Request(motor_id=0), timeout=3.0)
        call_service(self, Trigger, "/a3/arm/disable", Trigger.Request(), timeout=3.0)

    def run(self):
        rep = Reporter("F38b playback ramp verification")

        with open(self.args.traj, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        names = list(data["joint_names"])
        pts = [(float(p["time_from_start_sec"]), [float(v) for v in p["positions"]])
               for p in data["points"]]
        first_pt, last_pt, traj_dur = pts[0][1], pts[-1][1], pts[-1][0]
        self.get_logger().info(
            f"traj: {len(pts)} pts, {traj_dur:.1f}s, ramp={self.args.ramp}s")

        # 等 /joint_states
        deadline = time.time() + 10.0
        while self.latest is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        rep.check("joint_states live", self.latest is not None, "no /joint_states in 10s")
        if self.latest is None:
            rep.summary()
            return 1
        q_pre = self._pos_by_names(names, self.latest)
        rep.check("current pose known", all(map(lambda v: v == v, q_pre)),
                  f"q_pre={[round(v, 3) for v in q_pre]}")

        resp = call_service(self, PlaybackTrajectory, "/a3/arm/playback",
                            PlaybackTrajectory.Request(name=self.args.traj_name),
                            timeout=5.0)
        rep.check("playback service ok", resp is not None and resp.success,
                  "" if resp is None else resp.message)
        if resp is None or not resp.success:
            rep.summary()
            return 1

        t0 = time.monotonic()
        total_s = self.args.ramp + traj_dur + 2.0
        self.get_logger().info(f"sampling /joint_states for {total_s:.0f}s ...")
        try:
            while time.monotonic() - t0 < total_s:
                rclpy.spin_once(self, timeout_sec=0.02)
                if self.latest is not None:
                    self.samples.append((time.monotonic() - t0,
                                         self._pos_by_names(names, self.latest)))
        except KeyboardInterrupt:
            self._emergency_stop()
            raise
        self.get_logger().info(f"collected {len(self.samples)} samples")

        if len(self.samples) < 10:
            rep.check("samples collected", False, f"only {len(self.samples)}")
            rep.summary()
            return 1
        rep.check("samples collected", True, f"{len(self.samples)} samples")

        def max_delta(qa, qb):
            return max(abs(a - b) for a, b in zip(qa, qb))

        # 1) 无初始跳变：回放首样本 ≈ 回放前位姿
        rep.check("no initial jump", max_delta(self.samples[0][1], q_pre) < self.args.start_tol,
                  f"delta={max_delta(self.samples[0][1], q_pre):.4f} (< {self.args.start_tol})")

        # 2) ramp 期逐样本步长（20ms）无阶跃
        ramp_samples = [s for s in self.samples if s[0] <= self.args.ramp + 0.1]
        max_step = 0.0
        for (_, qa), (_, qb) in zip(ramp_samples, ramp_samples[1:]):
            max_step = max(max_step, max_delta(qa, qb))
        rep.check("ramp step smooth", max_step < self.args.step_tol,
                  f"max 20ms-step={max_step:.4f} (< {self.args.step_tol})")

        # 3) ramp 结束 ≈ 首记录点
        ramp_end = min(ramp_samples, key=lambda s: abs(s[0] - self.args.ramp))
        rep.check("ramp end ~= first point",
                  max_delta(ramp_end[1], first_pt) < self.args.pos_tol,
                  f"delta={max_delta(ramp_end[1], first_pt):.4f} (< {self.args.pos_tol})")

        # 4) 回放结束 ≈ 末记录点
        rep.check("final ~= last point", max_delta(self.samples[-1][1], last_pt) < self.args.pos_tol,
                  f"delta={max_delta(self.samples[-1][1], last_pt):.4f} (< {self.args.pos_tol})")

        # 5) 结束保持末位：最后 1s 漂移
        tail = [s for s in self.samples if s[0] >= self.samples[-1][0] - 1.0]
        drift = max(max_delta(s[1], last_pt) for s in tail) if tail else float("inf")
        rep.check("hold at end", drift < self.args.pos_tol, f"drift={drift:.4f}")

        # 首 3s 曲线（每 250ms）供人工复核
        print("\nramp curve (t, L1..L6):")
        for s in self.samples:
            if s[0] <= 3.0 and (s is self.samples[0] or abs(s[0] % 0.25) < 0.02):
                print(f"  t={s[0]:5.2f}  " +
                      "  ".join(f"{v:+7.3f}" for v in s[1]))
        print(f"  first pt:        " + "  ".join(f"{v:+7.3f}" for v in first_pt))

        return 0 if rep.summary() else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--traj", default=os.path.expanduser("~/.a3/trajectories/teach6.yaml"))
    ap.add_argument("--traj-name", default="teach6")
    ap.add_argument("--ramp", type=float, default=2.5)
    ap.add_argument("--step-tol", type=float, default=0.06)
    ap.add_argument("--pos-tol", type=float, default=0.15)
    ap.add_argument("--start-tol", type=float, default=0.05)
    args = ap.parse_args()

    rclpy.init()
    node = PlaybackVerifier(args)
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
