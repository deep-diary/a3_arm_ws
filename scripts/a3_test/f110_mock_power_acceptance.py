#!/usr/bin/env python3
"""F110 — mock 产品栈电源序列（sim_power_sequence）验收。

一条命令起产品栈（a3_bringup.launch.py hardware:=mock use_teleop:=true），
验证 mock 模式补齐电源序列后 L3 使能链路与 F61 灯效恢复正常：

  P1 锁存：gate_open=true / state=Running（TRANSIENT_LOCAL 晚订阅可达）
  P2 command 往返：shutdown → Idle/gate=false；start → Running/gate=true
  P3 L3 全链路：enable → FSM READY；/a3/ds4/feedback class=ready color=green
  P4 日志无 ps4_mapper L3 超时
  P5 launch 条件：sim_power_sequence 仅 hardware==mock 启动（源码静态核对）

隔离域 ROS_DOMAIN_ID=110。结束自动清栈，退出码反映 PASS/FAIL。
"""
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time

WS_DIR = "/home/cat/a3_arm_ws"
DOMAIN = "110"
LAUNCH_FILE = f"{WS_DIR}/src/a3_bringup/launch/a3_bringup.launch.py"

LAUNCH_PROC = None


def ensure_env() -> None:
    if os.environ.get("A3_F110_ENVED") == "1":
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
    env["ROS_DOMAIN_ID"] = DOMAIN
    env["A3_F110_ENVED"] = "1"
    os.execvpe(sys.executable,
               [sys.executable, os.path.abspath(__file__), *sys.argv[1:]], env)


class Report:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, name, ok, detail=""):
        mark = "PASS" if ok else "FAIL"
        line = f"  [{mark}] {name}"
        if detail:
            line += f"  ({detail})"
        print(line, flush=True)
        if ok:
            self.passed += 1
        else:
            self.failed += 1
        return ok

    def phase(self, title):
        print(f"\n=== {title} ===", flush=True)


def wait_future(fut, timeout):
    end = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < end:
        time.sleep(0.02)
    return fut.done()


def latch_qos(depth=1):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class PowerState:
    def __init__(self, node):
        from std_msgs.msg import Bool, String
        self.gate = None
        self.state = None
        node.create_subscription(Bool, "/power_sequence/gate_open",
                                  self._gate_cb, latch_qos())
        node.create_subscription(String, "/power_sequence/state",
                                  self._state_cb, latch_qos())

    def _gate_cb(self, msg):
        self.gate = bool(msg.data)

    def _state_cb(self, msg):
        self.state = msg.data

    def wait(self, gate, state, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.gate == gate and self.state == state:
                return True
            time.sleep(0.1)
        return False


class FsmState:
    def __init__(self, node):
        from a3_msgs.msg import ArmStatus
        self.state = ""
        node.create_subscription(ArmStatus, "/a3/arm_status", self._cb, 10)

    def _cb(self, msg):
        self.state = msg.state


class Feedback:
    def __init__(self, node):
        from std_msgs.msg import String
        self.latest = None
        node.create_subscription(String, "/a3/ds4/feedback", self._cb, 10)

    def _cb(self, msg):
        try:
            self.latest = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def wait(self, cls, color, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            d = self.latest
            if d and d.get("class") == cls and d.get("color") == color:
                return True
            time.sleep(0.2)
        return False


class CommandSender:
    """Publish reliably with a short re-send until ack (no latched publisher on node side)."""
    def __init__(self, node):
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._pub = node.create_publisher(String, "/power_sequence/command", qos)

    def send(self, command, hold=0.6):
        from std_msgs.msg import String
        end = time.monotonic() + hold
        while time.monotonic() < end:
            self._pub.publish(String(data=command))
            time.sleep(0.1)


def cleanup_launch(log_path):
    global LAUNCH_PROC
    proc = LAUNCH_PROC
    LAUNCH_PROC = None
    if proc is None:
        return
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=5.0)
    print(f"launch stopped; log: {log_path}", flush=True)


def main():
    global LAUNCH_PROC
    ensure_env()

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node("f110_mock_power_acceptance")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    power = PowerState(node)
    fsm = FsmState(node)
    feedback = Feedback(node)
    commands = CommandSender(node)
    rep = Report()
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_path = f"/tmp/f110_launch_{ts}.log"

    def signal_handler(signum, frame):
        print(f"\nsignal {signum} received, cleaning up...", flush=True)
        cleanup_launch(log_path)
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    launch_args = ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
                   "hardware:=mock", "use_rviz:=false", "use_mqtt:=false",
                   "use_teleop:=true"]
    logf = open(log_path, "w")
    LAUNCH_PROC = subprocess.Popen(
        launch_args, stdout=logf, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid)
    print(f"launch started (pid {LAUNCH_PROC.pid}): {' '.join(launch_args)}",
          flush=True)

    try:
        # ---------------- P1 latched power topics ----------------
        rep.phase("P1 锁存电源序列：gate_open=true / state=Running（TRANSIENT_LOCAL）")
        ok = power.wait(gate=True, state="Running", timeout=70.0)
        rep.check("latched gate_open=true & state=Running received", ok,
                 f"gate={power.gate} state={power.state!r}")

        # ---------------- P2 command roundtrip ----------------
        rep.phase("P2 command 往返：shutdown → Idle / start → Running")
        commands.send("shutdown")
        ok = power.wait(gate=False, state="Idle", timeout=6.0)
        rep.check("shutdown → gate=false / state=Idle", ok,
                 f"gate={power.gate} state={power.state!r}")
        commands.send("start")
        ok = power.wait(gate=True, state="Running", timeout=6.0)
        rep.check("start → gate=true / state=Running", ok,
                 f"gate={power.gate} state={power.state!r}")

        # ---------------- P3 L3 full chain ----------------
        rep.phase("P3 L3 语义：gate+Running 条件下 enable → READY / 灯效绿色")
        cond = power.gate is True and power.state == "Running"
        rep.check("mapper L3 gate condition satisfied", cond,
                 f"gate={power.gate} state={power.state!r}")
        cli = node.create_client(Trigger, "/a3/arm/enable")
        en = None
        # FSM 服务就绪时刻可能早于首帧 /joint_states（enable 前置位置检查会暂时
        # 拒绝）；重试该瞬时拒绝，其它拒绝立即判定失败。
        if cond and cli.wait_for_service(timeout_sec=20.0):
            end = time.monotonic() + 20.0
            while time.monotonic() < end:
                fut = cli.call_async(Trigger.Request())
                if wait_future(fut, 10.0):
                    res = fut.result()
                    if res.success or "no /joint_states" not in res.message:
                        en = res
                        break
                time.sleep(0.5)
        rep.check("/a3/arm/enable success", en is not None and en.success,
                 en.message if en else "enable failed/unreachable")
        end = time.monotonic() + 15.0
        ready = False
        while time.monotonic() < end:
            if fsm.state == "READY":
                ready = True
                break
            time.sleep(0.1)
        rep.check("FSM state READY", ready, f"state={fsm.state}")
        ok = feedback.wait("ready", "green", timeout=15.0)
        d = feedback.latest or {}
        rep.check("ds4 feedback class=ready / color=green (no red blink)", ok,
                 f"class={d.get('class')!r} color={d.get('color')!r} "
                 f"pattern={d.get('pattern')!r}")

        # ---------------- P4 no L3 timeout in log ----------------
        rep.phase("P4 ps4_mapper 日志无 L3 超时")
        time.sleep(2.0)
        log_text = open(log_path, errors="replace").read()
        timeout_hits = [l for l in log_text.splitlines()
                        if "timeout" in l.lower() and "power" in l.lower()]
        rep.check("no L3 power+enable timeout logged", not timeout_hits,
                 timeout_hits[-1].strip() if timeout_hits else "")

        # ---------------- P5 launch condition static check ----------------
        rep.phase("P5 launch 条件核对：sim_power_sequence 仅 mock 启动")
        src = open(LAUNCH_FILE).read()
        block = re.search(r"sim_power_sequence = Node\(.*?\n    \)", src, re.S)
        rep.check("sim_power_sequence Node defined", block is not None)
        cond_ok = bool(block and "== 'mock'" in block.group(0))
        rep.check("condition hardware == 'mock'", cond_ok)
        rep.check("node added to LaunchDescription items",
                 "sim_power_sequence," in src)

    finally:
        cleanup_launch(log_path)

    print(f"\n{'='*50}")
    print(f"TOTAL: {rep.passed} passed, {rep.failed} failed")
    if rep.failed:
        print("RESULT: FAIL")
        sys.exit(1)
    print("RESULT: PASS")
    sys.exit(0)


if __name__ == "__main__":
    main()
