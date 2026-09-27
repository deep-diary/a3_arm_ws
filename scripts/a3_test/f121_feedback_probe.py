#!/usr/bin/env python3
# F121 反馈新鲜度与解析实时性诊断探针（真机基线）。
#
# 量化「反馈来不及解析」是否成立，并验证 F122 失能态零增益保活是否让 gate 关闭期间
# /joint_states 与 /motor_feedback 持续更新。四层证据链：
#   L1 物理层  can_transport 堆日志「CAN bus counts(5s): rx_can0=.. rx_can1=..」增量（--log 交叉引用）
#   L2 网络层  /can_rx_frames (UInt8MultiArray, BEST_EFFORT depth5) 消息计数 —— DDS 丢包探测点
#   L3 解码层  /motor_feedback (String) 消息计数 + 按 motor 的 mode 分布 —— motor_protocol 解码吞吐
#   L4 发布层  /joint_states 到达间隔（应 20ms 周期）—— 上层消费者看到的"新鲜度"
#   L5 保活层  /a3/motor/tx_stats (TxStats, BEST_EFFORT) 每窗口 skip_power_gate / tx_refresh_total
#             —— skip_power_gate>0(即 gate 关闭)且 tx_refresh 增量>0 ⟹ F122 保活帧在流
#
# 用法（真机；断电/使能均可，建议先 75s 基线，期间手动开/关 gate）：
#   ros2 launch a3_bringup a3_bringup.launch.py
#   python3 scripts/a3_test/f121_feedback_probe.py --duration 75 --log <real_stack.log>
from __future__ import annotations

import argparse
import re
import statistics
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, UInt8MultiArray
from sensor_msgs.msg import JointState

# a3_can_bridge 生成的 TxStats（a3_can_bridge/msg/TxStats）
try:
    from a3_can_bridge.msg import TxStats
    HAVE_TXSTATS = True
except Exception:
    HAVE_TXSTATS = False

BE_QOS = QoSProfile(
    depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST)
REL_QOS = QoSProfile(
    depth=50, reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST)

RE_MODE = re.compile(r"motor=(\d+)\s+mode=(-?\d+)")
RE_RX_COUNTS = re.compile(r"CAN bus counts\(5s\):.*?rx_can0=(\d+)\s+rx_can1=(\d+)")
RE_TXWINDOW = re.compile(r"MP TX window\(5s\):.*?tx_traj=(\d+)[^t]*tx_refresh=(\d+)")
RE_GATE = re.compile(r"MP gate window\(5s\): gate_open=(-?\d+)")


def stamp_ns(ts: TimeMsg) -> int:
    return ts.sec * 1_000_000_000 + ts.nanosec


class Probe:
    def __init__(self):
        self.js_t: list[float] = []          # receipt time (monotonic s)
        self.fb_t: list[float] = []          # receipt time
        self.fb_mode: dict[int, int] = {}    # motor -> mode count
        self.fb_any: dict[int, int] = {}     # motor -> frame count
        self.can_t: list[float] = []         # receipt time
        self.tx_windows: list[dict] = []     # tx_stats snapshots

    # ---- callbacks ----
    def on_js(self, msg: JointState):
        self.js_t.append(time.monotonic())

    def on_fb(self, msg: String):
        self.fb_t.append(time.monotonic())
        m = RE_MODE.search(msg.data)
        if m:
            mid, mode = int(m.group(1)), int(m.group(2))
            self.fb_any[mid] = self.fb_any.get(mid, 0) + 1
            self.fb_mode[mid] = mode

    def on_can(self, msg: UInt8MultiArray):
        self.can_t.append(time.monotonic())

    def on_txstats(self, msg):
        if not HAVE_TXSTATS:
            return
        try:
            jn = list(msg.joint_names)
            hz = list(msg.tx_hz)
        except Exception:
            jn, hz = [], []
        self.tx_windows.append({
            "t": time.monotonic(),
            "window_s": float(msg.window_s),
            "refresh": int(msg.tx_refresh_total),
            "skip_gate": int(msg.skip_power_gate),
            "rate_ok": bool(msg.tx_rate_ok),
            "joints": jn,
            "hz": hz,
        })

    # ---- output helpers ----
    @staticmethod
    def pct_report(dts: list[float]):
        n = len(dts)
        if not n:
            return "  (no samples)"
        dts_s = sorted(dts)
        def q(x):
            return dts_s[min(int(x * n), n - 1)]
        over02 = sum(1 for d in dts_s if d > 0.02)   # 漏 1 帧（50Hz 周期 20ms）
        over05 = sum(1 for d in dts_s if d > 0.05)
        return (f"n={n} mean={statistics.mean(dts) * 1000:.1f}ms "
                f"p50={q(0.5) * 1000:.1f} p95={q(0.95) * 1000:.1f} "
                f"p99={q(0.99) * 1000:.1f} max={dts_s[-1] * 1000:.1f}ms "
                f"| gaps>20ms({over02}) >50ms({over05})")

    @staticmethod
    def per_sec(times: list[float], lo: float, hi: float) -> float:
        return sum(1 for t in times if lo <= t < hi)


def main():
    ap = argparse.ArgumentParser(description="F121 feedback-freshness / parse-realtime probe")
    ap.add_argument("--duration", type=float, default=75.0,
                    help="采集时长（s），建议 ≥ 2 个 5s 窗口 × 7 段，默认 75")
    ap.add_argument("--log", default=None,
                    help="真机栈日志路径：交叉引用 can_transport rx_can0/rx_can1 与 MP tx_refresh/gate 行")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("f121_feedback_probe")
    p = Probe()
    node.create_subscription(JointState, "/joint_states", p.on_js, REL_QOS)
    node.create_subscription(String, "/motor_feedback", p.on_fb, REL_QOS)
    if HAVE_TXSTATS:
        node.create_subscription(TxStats, "/a3/motor/tx_stats", p.on_txstats, BE_QOS)
    node.create_subscription(UInt8MultiArray, "/can_rx_frames", p.on_can, BE_QOS)

    # ---- L1: 骨架日志预解析（可以传输端实际收包为真值） ----
    log_rx = None
    gw = []          # (gate_open, tx_refresh_delta) 来自日志的 5s 窗口
    if args.log:
        log_rx = 0
        prev_rx = [0, 0]
        prev_tr = None
        for line in open(args.log, errors="ignore"):
            mc = RE_RX_COUNTS.search(line)
            if mc:
                r0, r1 = int(mc.group(1)), int(mc.group(2))
                log_rx += (r0 - prev_rx[0]) + (r1 - prev_rx[1])
                prev_rx = [r0, r1]
            mt = RE_TXWINDOW.search(line)
            mg = RE_GATE.search(line)
            gate = int(mg.group(1)) if mg else None
            if mt:
                tr = int(mt.group(2))
                if prev_tr is not None:
                    gw.append((gate, tr - prev_tr))
                prev_tr = tr

    print(f"[probe] F121 采集 {args.duration}s，订阅 /joint_states /motor_feedback "
          f"/can_rx_frames{'(TxStats)' if HAVE_TXSTATS else ''}"
          + (f"；日志交叉引用 {args.log}" if args.log else "；无日志（L1 缺失）"))
    print(f"[probe] L1 日志 can_transport rx 总量: {log_rx if args.log else 'N/A'} 帧\n")

    start = time.monotonic()
    spin = rclpy.executors.SingleThreadedExecutor()
    spin.add_node(node)
    t_last = start
    lo = start
    while True:
        spin.spin_once(timeout_sec=0.05)
        now = time.monotonic()
        if now % 5.0 < 0.2 and now - t_last >= 4.5:   # 每 ~5s 打印上一桶速率
            print(f"  +{now - start:6.1f}s  js={Probe.per_sec(p.js_t, lo, now)}/s "
                  f"fb={Probe.per_sec(p.fb_t, lo, now)}/s "
                  f"canrx={Probe.per_sec(p.can_t, lo, now)}/s")
            t_last, lo = now, now
        if now - start >= args.duration:
            break
    spin.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()
    t0 = start
    t1 = time.monotonic()
    dt_tot = t1 - t0

    # ---- L5: tx_stats 窗口（skip_gate>0 且 refresh 增量>0 ⟹ F122 保活观测） ----
    print("\n=== L5 /a3/motor/tx_stats 窗口（保活/门禁层） ===")
    keep_f121 = []
    for i in range(1, len(p.tx_windows)):
        w0, w1 = p.tx_windows[i - 1], p.tx_windows[i]
        d_ref = w1["refresh"] - w0["refresh"]
        d_skip = w1["skip_gate"] - w0["skip_gate"]
        tag = ""
        if d_skip > 0:
            tag = " [GATE-CLOSED]"
            if d_ref > 0:
                tag += " F122-keepalive-OK"
                keep_f121.append(d_ref)
        elif d_ref > 0:
            tag = " [gate-open refresh]"
        print(f"  t={w1['t'] - t0:7.1f}s window={w1['window_s']:.1f}s "
              f"skip_gate={d_skip}  tx_refreshΔ={d_ref}{tag}  tx_rate_ok={w1['rate_ok']}")
        if w1["joints"]:
            hzline = "  ".join(f"{j}:{h:.1f}" for j, h in zip(w1["joints"], w1["hz"]))
            print(f"    per-motor tx_hz → {hzline}")

    # ---- L3: 每电机 mode 分布 + 帧计数 ----
    print("\n=== L3 /motor_feedback 按电机（mode == 末次） ===")
    for mid in sorted(p.fb_any):
        print(f"  motor={mid}: frames={p.fb_any[mid]}  mode={p.fb_mode.get(mid, '?')}"
              f"  ({'使能' if p.fb_mode.get(mid) not in (0, -1) else '失能/未知'})")

    # ---- L4: /joint_states 到达间隔 ----
    print("\n=== L4 /joint_states 到达间隔（50Hz 期望 20ms 周期） ===")
    js_dt = [p.js_t[i] - p.js_t[i - 1] for i in range(1, len(p.js_t))]
    print("  " + Probe.pct_report(js_dt))

    # ---- L2 vs L3 loss（DDS 层 vs 解码层） ----
    n_can = len(p.can_t)
    n_fb = len(p.fb_t)
    print("\n=== L2/L3 计数与损耗 ===")
    print(f"  /can_rx_frames    L2: {n_can} 帧（{n_can / dt_tot:.1f}/s）")
    print(f"  /motor_feedback   L3: {n_fb} 条（{n_fb / dt_tot:.1f}/s）")
    if n_can > 0:
        l3_loss = max(0.0, 1.0 - n_fb / n_can)
        print(f"  L3帧/帧对L2载入 损耗: {l3_loss * 100:.1f}%  "
              f"({'解码≤收包，无解析瓶颈' if l3_loss < 0.05 else '⚠ 关注'})")
    if log_rx is not None and log_rx > 0:
        l2_loss = max(0.0, 1.0 - n_can / log_rx)
        print(f"  L2 对 L1 套接字 损耗: {l2_loss * 100:.1f}%  "
              f"(DDS BEST_EFFORT depth5 hop; {'正常' if l2_loss < 0.1 else '⚠ 关注'})")
    else:
        print("  L1（套接字真值）未提供：加 --log 指向 can_transport 日志可补损耗链")

    # ---- L1: 日志门禁/刷新交叉 ----
    if args.log:
        print("\n=== L1 日志交叉（MP门禁/刷新，can_transport rx） ===")
        print(f"  MP tx_refresh 增量窗口（(gate_open,Δrefresh)）: {gw}")
    else:
        print("\n（提示：未给 --log，缺 L1 套接字真值；建议真机跑时带栈日志）")

    # ---- F121/F122 判定 ----
    print("\n=== 判定 ===")
    ver = []
    if n_fb >= 7 * 3 and (not js_dt or statistics.mean(js_dt) < 0.03):
        ver.append("F121 基线：/motor_feedback 满 7 电机、/joint_states 平均间隔<30ms —— 解析实时、无饱和")
        verdict = 1
    elif n_fb < 7 * 3:
        ver.append("F121 基线：/motor_feedback 帧太少（疑似 0x18 未开或总线静默）—— 先查 0x18 配置")
        verdict = 0
    else:
        ver.append("F121 基线：/joint_states 平均间隔 ≥30ms —— 反馈存在降级，按 L1/L2/L3 定位瓶颈")
        verdict = 0.5
    if keep_f121:
        ver.append(f"F122 保活：gate 关闭期间 tx_refresh 增量均值 {statistics.mean(keep_f121):.0f} 帧/5s —— 失能零增益保活帧在流")
        verdict = min(verdict, 1)
    else:
        ver.append("F122 保活：未观测到 gate 关闭窗口，或保活未生效（确认 refresh_keepalive_when_gate_closed 与失能状态）")
        verdict = min(verdict, 0.5)
    for v in ver:
        print(f"  - {v}")
    if isinstance(verdict, float) and verdict == 1:
        verdict = 1
    print(f"\n[probe] 总判定: {'PASS' if (not isinstance(verdict, float) or verdict >= 1) else ('PARTIAL' if verdict >= 0.5 else 'FAIL')}")
    sys.exit(0 if (not isinstance(verdict, float) or verdict >= 1) else 1)


if __name__ == "__main__":
    main()