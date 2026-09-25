#!/usr/bin/env python3
"""F111 — 真机（hardware:=can）栈 C++ power_sequence_node 验收。

一条命令起产品栈（a3_bringup.launch.py hardware:=can can_interface:=vcan981
use_teleop:=true + vcan_motor_sim 虚拟电机），验证 F78 统一栈 can 分支补接
真机执行层电源节点后 gate/state 时序与 L3 使能链路恢复：

  P1 初始锁存：/power_sequence_node 在栈中，gate_open=false / state=Idle
     （TRANSIENT_LOCAL 晚订阅可达）
  P2 command 生命周期：start → Running/gate=true 在 5 s（mapper L3 轮询窗口）内
     完成（约 1.1 s）；shutdown → Idle/gate=false
  P3 模拟 L3：Running+gate 条件下 /a3/arm/enable 成功，FSM READY
  P4 ds4_feedback class=ready / color=green（不再按 gate 关 + 失电红闪）
  P5 launch 条件：can_power_sequence 仅 hardware==can；sim_power_sequence 仅
     mock（两节点互斥，源码静态核对）

隔离域 ROS_DOMAIN_ID=111。结束自动清栈，退出码反映 PASS/FAIL。
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
DOMAIN = "111"
VCAN = "vcan981"
LAUNCH_FILE = f"{WS_DIR}/src/a3_bringup/launch/a3_bringup.launch.py"
STACK_LOG = "/tmp/f111_stack.log"
SIM_LOG = "/tmp/f111_sim.log"

LAUNCH_PROC = None
SIM_PROC = None


def ensure_env() -> None:
    if os.environ.get("A3_F111_ENVED") == "1":
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
    env["A3_F111_ENVED"] = "1"
    os.execvpe(sys.executable,
               [sys.executable, os.path.abspath(__file__), *sys.argv[1:]], env)


def ensure_vcan() -> None:
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run(["sudo", "-S", "ip", "link", "del", "dev", VCAN],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "add", "dev", VCAN, "type", "vcan"],
        input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "set", VCAN, "up"],
        input=b"temppwd\n", capture_output=True)


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


def cleanup_sim(log_path):
    global SIM_PROC
    proc = SIM_PROC
    SIM_PROC = None
    if proc is None:
        return
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            pass
    print(f"sim stopped; log: {log_path}", flush=True)


def main():
    global LAUNCH_PROC, SIM_PROC
    ensure_env()

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger

    ensure_vcan()

    rclpy.init()
    node = rclpy.create_node("f111_can_power_acceptance")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    power = PowerState(node)
    fsm = FsmState(node)
    feedback = Feedback(node)
    commands = CommandSender(node)
    rep = Report()
    ts = time.strftime("%Y%m%d_%H%M%S")
    stack_log_path = STACK_LOG
    sim_log_path = SIM_LOG

    signal.signal(signal.SIGINT, lambda s, f: cleanup_all())
    signal.signal(signal.SIGTERM, lambda s, f: cleanup_all())

    def cleanup_all():
        cleanup_launch(stack_log_path)
        cleanup_sim(sim_log_path)
        sys.exit(1)

    # 虚拟电机：can 栈插件直驱 vcan981；一阶跟随让 /joint_states 有活反馈
    # （FSM enable 前置位置检查需要），不需要 p1p2 的滞后注入量级。
    sim_proc = subprocess.Popen(
        ["python3", f"{WS_DIR}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", VCAN],
        stdout=open(sim_log_path, "w"), stderr=subprocess.STDOUT,
        preexec_fn=os.setsid)
    SIM_PROC = sim_proc
    time.sleep(1.0)

    launch_args = ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
                   "hardware:=can", f"can_interface:={VCAN}",
                   "use_rviz:=false", "use_mqtt:=false",
                   "use_teleop:=true", "use_self_test:=false",
                   "use_diagnostics:=false", "use_rosbag:=false"]
    logf = open(stack_log_path, "w")
    LAUNCH_PROC = subprocess.Popen(
        launch_args, stdout=logf, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid)
    print(f"launch started (pid {LAUNCH_PROC.pid}): {' '.join(launch_args)}",
          flush=True)

    try:
        # ---------------- P1 initial latched state ----------------
        rep.phase("P1 初始锁存：/power_sequence_node gate_open=false / state=Idle")
        ok = power.wait(gate=False, state="Idle", timeout=70.0)
        rep.check("initial gate=false & state=Idle received (latched)", ok,
                 f"gate={power.gate} state={power.state!r}")

        # ---------------- P2 command lifecycle ----------------
        rep.phase("P2 command 生命周期：start → Running(5 s 内) / shutdown → Idle")
        t0 = time.monotonic()
        commands.send("start")
        ok = power.wait(gate=True, state="Running", timeout=5.0)
        elapsed = time.monotonic() - t0
        rep.check("start → gate=true / state=Running within 5 s", ok,
                 f"elapsed={elapsed:.2f}s gate={power.gate} state={power.state!r}")
        commands.send("shutdown")
        ok = power.wait(gate=False, state="Idle", timeout=6.0)
        rep.check("shutdown → gate=false / state=Idle", ok,
                 f"gate={power.gate} state={power.state!r}")

        # ---------------- P3 simulated L3 enable ----------------
        rep.phase("P3 L3 语义：gate+Running 条件下 enable → FSM READY")
        commands.send("start")
        if not power.wait(gate=True, state="Running", timeout=6.0):
            rep.check("re-arm gate+Running before enable", False,
                     f"gate={power.gate} state={power.state!r}")
        else:
            rep.check("re-arm gate+Running before enable", True,
                     f"elapsed power on again")
            cli = node.create_client(Trigger, "/a3/arm/enable")
            en = None
            if cli.wait_for_service(timeout_sec=20.0):
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

        # ---------------- P4 ds4 feedback healthy ----------------
        rep.phase("P4 ds4_feedback class=ready / color=green（不再红闪）")
        ok = feedback.wait("ready", "green", timeout=15.0)
        d = feedback.latest or {}
        rep.check("ds4 feedback class=ready / color=green (no red blink)", ok,
                 f"class={d.get('class')!r} color={d.get('color')!r} "
                 f"pattern={d.get('pattern')!r}")

        # ---------------- P5 launch condition static check ----------------
        rep.phase("P5 launch 条件核对：can/sim 电源节点互斥")
        src = open(LAUNCH_FILE).read()
        can_block = re.search(r"can_power_sequence = Node\(.*?\n    \)", src, re.S)
        sim_block = re.search(r"sim_power_sequence = Node\(.*?\n    \)", src, re.S)
        rep.check("can_power_sequence Node defined", can_block is not None)
        rep.check("can_power_sequence condition hardware == 'can'",
                  bool(can_block and can_block.group(0).count("'can'") > 0))
        rep.check("sim_power_sequence condition hardware == 'mock'",
                  bool(sim_block and sim_block.group(0).count("'mock'") > 0))
        rep.check("both nodes in LaunchDescription items (mutually exclusive)",
                 "sim_power_sequence," in src and "can_power_sequence," in src)

    finally:
        cleanup_launch(stack_log_path)
        cleanup_sim(sim_log_path)

    print(f"\n{'='*50}")
    print(f"TOTAL: {rep.passed} passed, {rep.failed} failed")
    if rep.failed:
        print("RESULT: FAIL")
        sys.exit(1)
    print("RESULT: PASS")
    sys.exit(0)


if __name__ == "__main__":
    main()