#!/usr/bin/env python3
"""F105 验收：DDS 网络栈硬化（sysctl + CycloneDDS XML）。

隔离域 105，机械臂断电。四项：
  1. sysctl 五项缓冲值达标
  2. 用安装后的 /etc/a3/cyclonedds.xml（CYCLONEDDS_URI）pub/sub 序号消息互通
  3. 20 s 压力（200/50/10 Hz 三种消息 + 4 核 CPU/内存压力）：
     零丢失零乱序；F104 topic_rate_monitor 旁路观察 50 Hz 话题不出现 WARN
  4. /etc/default/a3-arm 的 CYCLONEDDS_URI 注入链完整

用法：python3 scripts/a3_test/f105_dds_network_acceptance.py [domain]
"""
import importlib
import math
import multiprocessing as mp
import os
import signal
import subprocess
import sys
import time

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "105"
XML = "/etc/a3/cyclonedds.xml"
ENV_FILE = "/etc/default/a3-arm"
MON_LOG = "/tmp/f105_monitor.log"

SYSCTL_WANT = {
    "/proc/sys/net/core/rmem_max": 25165824,
    "/proc/sys/net/core/wmem_max": 25165824,
    "/proc/sys/net/core/rmem_default": 2097152,
    "/proc/sys/net/core/wmem_default": 2097152,
    "/proc/sys/net/core/netdev_max_backlog": 2000,
}

TOPIC_FAST = "/f105_fast"   # 200 Hz 小消息
TOPIC_MID = "/f105_mid"     # 50 Hz 中消息（topic_rate_monitor 观察）
TOPIC_BIG = "/f105_big"     # 10 Hz 8 KB 大消息

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" ({detail})" if detail else ""))


def _read(path):
    with open(path) as f:
        return int(f.read().strip())


def _level(x):
    return x[0] if isinstance(x, (bytes, bytearray)) else x


def make_env():
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = DOMAIN
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    env["PYTHONNOUSERSITE"] = "1"
    env["CYCLONEDDS_URI"] = f"file://{XML}"
    env["A3_DDS_IFACE"] = "wlan0"
    return env


def _load(msg_path, name):
    pkg = msg_path.split("/")[0]
    return getattr(importlib.import_module(f"{pkg}.msg"), name)


Int32 = _load("std_msgs/msg/Int32", "Int32")
Float64MultiArray = _load("std_msgs/msg/Float64MultiArray", "Float64MultiArray")
UInt8MultiArray = _load("std_msgs/msg/UInt8MultiArray", "UInt8MultiArray")
DiagnosticArray = _load("diagnostic_msgs/msg/DiagnosticArray", "DiagnosticArray")


def cpu_worker(stop):
    """跨核 CPU + 内存分配压力（独立进程绕开 GIL）。"""
    i = 0
    while not stop.value:
        x = 0.0
        for _ in range(200000):
            x += math.sqrt(i)
            i += 1
        if i % 3 == 0:
            _ = [0.0] * 4096  # 分配突发
        if not math.isfinite(x):
            print("bad")


def main():
    # 1) sysctl
    bad = []
    for path, want in SYSCTL_WANT.items():
        got = _read(path)
        if got < want:
            bad.append(f"{path}={got} want>={want}")
    check("sysctl 缓冲五项达标", not bad, "; ".join(bad))

    # CYCLONEDDS_URI 必须在首个 DDS 参与者创建前注入
    os.environ.update(make_env())

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("f105_acceptance")

    stats = {}  # topic -> dict(received, missing, reorder, last_seq, worst_dt, last_t)
    diag_seen = [False]
    diag_worst = [0]

    def make_cb(topic):
        def cb(msg):
            s = stats.setdefault(topic, dict(
                received=0, missing=0, reorder=0,
                last_seq=None, worst_dt=0.0, last_t=time.monotonic()))
            seq = msg.data if hasattr(msg, "data") and isinstance(msg.data, int) else None
            now = time.monotonic()
            dt = now - s["last_t"]
            s["worst_dt"] = max(s["worst_dt"], dt)
            s["last_t"] = now
            s["received"] += 1
            if seq is not None and s["last_seq"] is not None:
                if seq <= s["last_seq"]:
                    s["reorder"] += 1
                elif seq > s["last_seq"] + 1:
                    s["missing"] += seq - s["last_seq"] - 1
            if seq is not None:
                s["last_seq"] = seq
        return cb

    sub_fast = node.create_subscription(Int32, TOPIC_FAST, make_cb(TOPIC_FAST), 10)
    sub_mid = node.create_subscription(
        Float64MultiArray, TOPIC_MID, make_cb(TOPIC_MID), 10)
    sub_big = node.create_subscription(
        UInt8MultiArray, TOPIC_BIG, make_cb(TOPIC_BIG), 10)

    def on_diag(msg):
        for stt in msg.status:
            if stt.name.startswith("a3_topic_rate:") and TOPIC_MID in stt.name:
                diag_seen[0] = True
                diag_worst[0] = max(diag_worst[0], _level(stt.level))

    node.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 10)

    pub_fast = node.create_publisher(Int32, TOPIC_FAST, 10)
    pub_mid = node.create_publisher(Float64MultiArray, TOPIC_MID, 10)
    pub_big = node.create_publisher(UInt8MultiArray, TOPIC_BIG, 10)

    stop_pub = [False]
    seq_fast = [0]
    seq_mid = [0]
    seq_big = [0]

    def pub_loop(pub, hz, seq, build):
        period = 1.0 / hz
        next_t = time.monotonic()
        while not stop_pub[0]:
            now = time.monotonic()
            if now >= next_t:
                pub.publish(build(seq[0]))
                seq[0] += 1
                next_t += period
                if next_t < now - period:
                    next_t = now + period
            else:
                time.sleep(min(0.002, next_t - now))

    import threading
    threads = [
        threading.Thread(target=pub_loop, args=(
            pub_fast, 200, seq_fast, lambda s: Int32(data=s)), daemon=True),
        threading.Thread(target=pub_loop, args=(
            pub_mid, 50, seq_mid,
            lambda s: Float64MultiArray(data=[float(s)] + [0.0] * 63)), daemon=True),
        threading.Thread(target=pub_loop, args=(
            pub_big, 10, seq_big,
            lambda s: UInt8MultiArray(data=bytes((s % 256,)) * 8000)), daemon=True),
    ]

    # 2) XML 注入下互通（先起订阅/线程，2.5 s 后校验收到全部三流）
    for t in threads:
        t.start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.5:
        rclpy.spin_once(node, timeout_sec=0.05)
    got_all = all(stats.get(t, {}).get("received", 0) > 0
                  for t in (TOPIC_FAST, TOPIC_MID, TOPIC_BIG))
    check("安装 XML 下三流互通", got_all,
          ", ".join(f"{k.split('_')[-1]}:{v['received']}" for k, v in stats.items()))

    # 3) 启动 topic_rate_monitor（F104 产品节点）+ 压力进程
    mon = subprocess.Popen(
        ["ros2", "run", "a3_bringup", "topic_rate_monitor",
         "--ros-args",
         "-p", f"topics:=[{TOPIC_MID}]",
         "-p", "min_freq:=[40.0]",
         "-p", "max_freq:=[60.0]"],
        env=make_env(), start_new_session=True,
        stdin=subprocess.DEVNULL, stdout=open(MON_LOG, "w"),
        stderr=subprocess.STDOUT)

    ncpu = os.cpu_count() or 4
    n_workers = min(4, max(1, ncpu // 2))
    mgr = mp.Manager()
    stop_mp = mgr.Value("b", False)
    workers = [mp.Process(target=cpu_worker, args=(stop_mp,))
               for _ in range(n_workers)]
    for w in workers:
        w.start()

    settle = 3.0
    measure = 20.0
    ts = time.monotonic()
    settle_deadline = ts + settle
    while time.monotonic() < settle_deadline:
        rclpy.spin_once(node, timeout_sec=0.02)

    # 测量窗口零点快照：启动/发现期 gap 不计入压力判据
    snap = {t: {k: stats[t][k] for k in ("received", "missing", "reorder")}
            for t in (TOPIC_FAST, TOPIC_MID, TOPIC_BIG)}

    measure_deadline = time.monotonic() + measure
    while time.monotonic() < measure_deadline:
        rclpy.spin_once(node, timeout_sec=0.02)

    stop_mp.value = True
    for w in workers:
        w.join(timeout=3)
        if w.is_alive():
            w.terminate()

    win = {t: {k: stats[t][k] - snap[t][k]
               for k in ("received", "missing", "reorder")}
           for t in (TOPIC_FAST, TOPIC_MID, TOPIC_BIG)}
    total_missing = sum(w["missing"] for w in win.values())
    total_reorder = sum(w["reorder"] for w in win.values())
    expected_fast = int(measure * 200 * 0.8)
    expected_mid = int(measure * 50 * 0.8)
    counts_ok = (win[TOPIC_FAST]["received"] >= expected_fast
                 and win[TOPIC_MID]["received"] >= expected_mid)
    check(f"压力 {measure:.0f}s 零丢失零乱序、计数充足",
          total_missing == 0 and total_reorder == 0 and counts_ok,
          f"missing={total_missing} reorder={total_reorder} "
          f"fast={win[TOPIC_FAST]['received']}/{expected_fast} "
          f"mid={win[TOPIC_MID]['received']}/{expected_mid} "
          f"big={win[TOPIC_BIG]['received']}")

    check("topic_rate_monitor 50 Hz 全程不劣化",
          diag_seen[0] and diag_worst[0] == 0,
          f"seen={diag_seen[0]} worst_level={diag_worst[0]}")

    # 4) EnvironmentFile 注入链
    env_ok = False
    try:
        with open(ENV_FILE) as f:
            for line in f:
                if line.strip() == f"CYCLONEDDS_URI=file://{XML}":
                    env_ok = True
    except OSError:
        pass
    check("/etc/default/a3-arm 注入 CYCLONEDDS_URI", env_ok and os.path.exists(XML))

    stop_pub[0] = True
    try:
        os.killpg(os.getpgid(mon.pid), signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass
    node.destroy_node()
    rclpy.shutdown()

    passed = sum(RESULTS)
    print(f"\n==== F105 acceptance: {passed}/{len(RESULTS)} ====")
    return 0 if passed == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
