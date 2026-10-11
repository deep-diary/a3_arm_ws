#!/usr/bin/env python3
"""反馈健康度实时监控（CAN 掉包/陈旧定位）。

订阅 /joint_states，实时告警反馈到达间隔超阈值；用于巡游时后台跑，把「意外震动」
时刻与反馈掉包时刻一一对应，定位 can1 线束接触不良的具体丢包点。

用法（真机，先起栈）：
  python3 scripts/a3_test/feedback_gap_monitor.py --threshold 0.2 --warn 0.05
  # Ctrl-C 结束，输出统计摘要（丢包次数 + 时间戳列表）。

默认阈值 0.2s（= a3_mit_hardware_interface 的 fb_timeout），即反馈一旦超过 0.2s
未更新，硬件接口就会判定 stale；正常 /joint_states 周期约 20ms。
"""
from __future__ import annotations

import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class GapMonitor(Node):
    def __init__(self, threshold: float, warn: float):
        super().__init__("feedback_gap_monitor")
        self.threshold = threshold
        self.warn = warn
        self.times: list[float] = []
        self.gaps: list[tuple[float, float]] = []   # (相对 t, gap)
        self.warns: list[tuple[float, float]] = []  # (相对 t, gap)
        self.t0: float | None = None
        self.create_subscription(
            JointState, "/joint_states", self._cb, qos_profile_sensor_data)

    def _cb(self, _msg: JointState):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
            print(f"[feedback_gap_monitor] 开始监听 /joint_states "
                  f"阈值={self.threshold*1000:.0f}ms 警告={self.warn*1000:.0f}ms", flush=True)
            self.times.append(now)
            return
        dt = now - self.times[-1]
        rel = now - self.t0
        if dt >= self.threshold:
            self.gaps.append((rel, dt))
            print(f"[+{rel:8.2f}s] ⚠ GAP {dt*1000:6.0f}ms  (>= {self.threshold*1000:.0f}ms) "
                  f"— 反馈掉包/陈旧，若此刻有震动即实锤", flush=True)
        elif dt >= self.warn:
            self.warns.append((rel, dt))
            print(f"[+{rel:8.2f}s]   gap {dt*1000:6.0f}ms  (>= {self.warn*1000:.0f}ms)", flush=True)
        self.times.append(now)


def main():
    ap = argparse.ArgumentParser(description="反馈健康度实时监控（/joint_states 间隔告警）")
    ap.add_argument("--threshold", type=float, default=0.2,
                    help="超时阈值（s），默认 0.2（fb_timeout）")
    ap.add_argument("--warn", type=float, default=0.05,
                    help="警告阈值（s），默认 0.05（漏 2+ 帧）")
    args = ap.parse_args()

    rclpy.init()
    node = GapMonitor(args.threshold, args.warn)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # ---- 汇总 ----
    dts = [node.times[i] - node.times[i - 1] for i in range(1, len(node.times))]
    print("\n===== 反馈健康度汇总 =====")
    if not dts:
        print("  无样本")
    else:
        s = sorted(dts)
        n = len(s)
        def q(x):
            return s[min(int(x * n), n - 1)]
        print(f"  样本={n}  运行={node.times[-1] - node.t0:.1f}s")
        print(f"  间隔 mean={statistics.mean(dts)*1000:.1f}ms "
              f"p50={q(0.5)*1000:.1f} p95={q(0.95)*1000:.1f} "
              f"p99={q(0.99)*1000:.1f} max={s[-1]*1000:.1f}ms")
        print(f"  gap>{args.warn*1000:.0f}ms: {len(node.warns)} 次   "
              f"gap>{args.threshold*1000:.0f}ms(掉包): {len(node.gaps)} 次")
    if node.gaps:
        print(f"\n  ⚠ 掉包时刻（相对开始，s）：" + "  ".join(f"{t:.2f}" for t, _ in node.gaps))
    if node.warns:
        print(f"  警告时刻（相对开始，s）：" + "  ".join(f"{t:.2f}" for t, _ in node.warns))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
