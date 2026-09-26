#!/usr/bin/env python3
"""F113 — 失能/门禁时电机 0x18 主动上报保活（always-on active report）vcan 验收。

一条命令起产品栈（a3_bringup.launch.py hardware:=can can_interface:=vcan982
use_teleop:=false + vcan_motor_sim 虚拟电机），在一根独立 vcan 上同时嗅探
0x18 命令帧与电机流，验证插件在 on_activate/on_deactivate 每次 reset 后自
行重挂 0x18 all-ON（F113），门禁在全新实例 7/7 有反馈才放行：

  栈内 4 阶段共享一段 enable/disable 循环（闭环 C≥3 + 帧级 A/B 各 ≥3）：
  P1 使能：插件 0x18-ON 命令帧 ≥7 且 7 电机持续流式反馈（≥15fps/1s）+ READY
     帧级 B（on_activate 首启补发 0x18-ON ×7）
  P2 失能：on_deactivate 再次 0x18-ON ≥7（帧级 A），流不中断（≥15fps/1s），
     DISABLED 期间 /joint_states 持续跳动且不出现 ±12.49635 冻结标记
  P3 再使能/失能 ×2：每 cycle 使能 READY + 失能 DISABLED，双向 0x18-ON ≥7
  P4 暗电机失败态（回归 D 的 fresh 实例形态）：fresh 栈 + write_noar([4])
     注入（m4 全暗，无 0x18 流、无 type-2 应答）→ 使能必失败（日志
     "activate aborted: only 6/7"）且永不 READY（has_feedback 单实例内只置位
     不清除、锁存，fresh 实例才能复现 6/7 门）
  P5 清除注入 + fresh 栈：m4 恢复 → 使能 7/7 → READY，m4 恢复流

隔离域 ROS_DOMAIN_ID=113。结束自动清栈，退出码反映 PASS/FAIL。
"""
import json
import os
import signal
import struct
import subprocess
import sys
import threading
import time

WS_DIR = "/home/cat/a3_arm_ws"
DOMAIN = "113"
VCAN = "vcan982"
NOAR_FILE = "/tmp/f113_no_active_report.json"
LAUNCH_FILE = f"{WS_DIR}/src/a3_bringup/launch/a3_bringup.launch.py"
CAN_FORMAT = "=IB3x8s"
CMD_ACTIVE_REPORT = 0x18
MASTER_ID = 0xFD
MOTORS = list(range(1, 8))
# 固件「无有效位置」标记是 decode(0x7FFF) 得到的 ±12.49635。真机 /joint_states
# 关节域恒小于 URDF 限位（<5 rad），正常不可能接近它；但 sim 电机可合法跑
# 到 ±P_RANGE=±12.57（2 圈），这要放行。因此必须精确匹配 sentinel，不能用
# |p|>6 的宽判——否则误把 sim 饱和当成「冻结在无效位」。
POS_SENTINEL = 12.49635
POS_SENTINEL_EPS = 0.05

LAUNCH_PROC = None
SIM_PROC = None
STACK_LOG = "/tmp/f113_stack.log"
SIM_LOG = "/tmp/f113_sim.log"


def ensure_env() -> None:
    if os.environ.get("A3_F113_ENVED") == "1":
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
    env["A3_F113_ENVED"] = "1"
    os.execvpe(sys.executable,
               [sys.executable, os.path.abspath(__file__), *sys.argv[1:]], env)


def iface_up(iface: str) -> bool:
    return "" != subprocess.run(
        ["ip", "link", "show", iface], capture_output=True, text=True
    ).stdout


def ensure_vcan() -> None:
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=b"temppwd\n", capture_output=True)
    # Don't delete a pre-existing interface — reuse it. vcan is ephemeral,
    # so a leftover is either from a previous acceptance run (safe to reuse)
    # or belongs to something else (not ours to remove).
    if iface_up(VCAN):
        return
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


def write_noar(motors):
    with open(NOAR_FILE, "w") as f:
        json.dump({"motors": motors}, f)


class VcanSniffer:
    """Raw AF_CAN reader on the shared vcan: counts command-form 0x18-ON
    (bits8-15 == 0xFD -> motor in low byte) vs reply-stream 0x18
    (low byte == 0xFD -> motor in bits8-15)."""

    def __init__(self, iface):
        import socket
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.bind((iface,))
        self._sock.settimeout(0.1)
        self._lock = threading.Lock()
        self.cmd_on = {m: 0 for m in MOTORS}
        self.stream = {m: [] for m in MOTORS}  # monotonic timestamps
        self._stop = False
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _run(self):
        while not self._stop:
            try:
                frame = self._sock.recv(16)
            except Exception:
                continue
            if len(frame) < 16:
                continue
            can_id = struct.unpack("=I", frame[:4])[0] & 0x1FFFFFFF
            cmd = (can_id >> 24) & 0x1F
            dest = (can_id >> 8) & 0xFF
            low = can_id & 0xFF
            if cmd != CMD_ACTIVE_REPORT:
                continue
            now = time.monotonic()
            with self._lock:
                if dest == MASTER_ID and low in self.cmd_on:
                    # command-form 0x18-ON sent by the plugin: data[6]&1 == 1
                    data = frame[8:16]
                    if len(data) == 8 and data[6] & 0x01:
                        self.cmd_on[low] += 1
                elif low == MASTER_ID and dest in self.stream:
                    self.stream[dest].append(now)
                    tv = self.stream[dest]
                    while tv and tv[0] < now - 3.0:
                        tv.pop(0)

    def reset(self):
        with self._lock:
            for m in MOTORS:
                self.cmd_on[m] = 0
                self.stream[m] = []

    def cmd_on_total(self):
        with self._lock:
            return sum(self.cmd_on.values())

    def stream_count(self, motor):
        with self._lock:
            return len(self.stream.get(motor, []))

    def stream_rate(self, motor, window_s):
        now = time.monotonic()
        with self._lock:
            return sum(1 for t in self.stream.get(motor, []) if now - t <= window_s)

    def streams_all(self, min_rate, window_s):
        return all(self.stream_rate(m, window_s) >= min_rate for m in MOTORS)

    def stop(self):
        self._stop = True


class JointStates:
    def __init__(self, node):
        from sensor_msgs.msg import JointState
        self.count = 0
        self.positions = []
        node.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg):
        self.count += 1
        self.positions = list(msg.position)


class PowerState:
    def __init__(self, node):
        from std_msgs.msg import Bool, String
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.gate = None
        self.state = None
        node.create_subscription(Bool, "/power_sequence/gate_open", self._gate_cb, qos)
        node.create_subscription(String, "/power_sequence/state", self._state_cb, qos)

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

    def wait(self, state, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.state == state:
                return True
            time.sleep(0.1)
        return False


class CommandSender:
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


def _pids_by_pgid():
    out = subprocess.run(["ps", "-eo", "pid=,pgid="], capture_output=True,
                         text=True).stdout
    res = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
            res[int(parts[0])] = int(parts[1])
    return res


def cleanup_launch(log_path):
    global LAUNCH_PROC
    proc = LAUNCH_PROC
    LAUNCH_PROC = None
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        pgid = None
    if proc.poll() is None and pgid is not None:
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            pass
    if pgid is not None:
        # 正向清扫：launch 领导进程可能提前退出而子 ROS 节点仍驻留同 PG 存活，
        # 这些残留会跨 run 污染 domain 113 的 /joint_states（arm_controller 的
        # enable 预检读到多分钟前的 stale 时间戳）。逐个 SIGKILL 保证整 PG 清空。
        for pid, g in _pids_by_pgid().items():
            if g == pgid:
                try:
                    os.kill(pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass
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
    write_noar([])
    rclpy.init()
    node = rclpy.create_node("f113_keepalive_acceptance")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_th = threading.Thread(target=executor.spin, daemon=True)
    spin_th.start()

    power = PowerState(node)
    fsm = FsmState(node)
    js = JointStates(node)
    commands = CommandSender(node)
    rep = Report()
    sniff = VcanSniffer(VCAN)

    signal.signal(signal.SIGINT, lambda s, f: sys.exit(1))
    signal.signal(signal.SIGTERM, lambda s, f: sys.exit(1))

    def start_stack(tag):
        # start_stack/main 不是同一函数嵌套作用域：main() 开头那行 global 只管 main 自己，
        # 这里不声明就把 SIM_PROC/LAUNCH_PROC 存成 start_stack 局部，cleanup_launch/sim 读
        # 到的仍是模块级 None → 每轮 stop_stack 静默空转，整栈残留跨 run 累积（run6 enable
        # 就撞上上一轮的孤儿 arm_controller）。必须在此显式提级为模块全局。
        global LAUNCH_PROC, SIM_PROC
        # 多 sim 实例并发写 /tmp/f86_timeout_state.json（旧版共享 .tmp 名）曾 os.replace ENOENT
        # 互相吞进程；这里先清掉本 vcan 上残留的旧 sim/launch。注意：只杀 launch 主 pid 会
        # 让它子节点（controller_manager/arm_controller/servo_mode_bridge 等）成孤儿驻留同
        # domain，孤儿 arm_controller 仍注册 /a3/arm/enable，其背靠的 controller_manager 已
        # 死 → run6 enable 就死在 set_hardware_component_state 超时。所以按整 PG 清扫：
        # sim 与 launch 主进程的 cmdline 都带 vcan982，取它们的 PGID 再 killpg 连孤儿一起清。
        pgids = set()
        for pat in (f"vcan_motor_sim.*--interface {VCAN}",
                    f"a3_bringup.launch.py.*can_interface:={VCAN}"):
            out = subprocess.run(["pgrep", "-f", pat],
                                 capture_output=True, text=True).stdout
            for pid in (int(x) for x in out.split() if x.isdigit()):
                try:
                    pgids.add(os.getpgid(pid))
                except ProcessLookupError:
                    pass
        for pg in pgids:
            try:
                os.killpg(pg, signal.SIGKILL)
                print(f"[{tag}] killed leftover PG {pg}", flush=True)
            except ProcessLookupError:
                pass
        time.sleep(0.3)
        sim_out = f"/tmp/f113_sim_{tag}.log"
        sim_proc = subprocess.Popen(
            ["python3", f"{WS_DIR}/scripts/a3_test/vcan_motor_sim.py",
             "--interface", VCAN],
            stdout=open(sim_out, "w"), stderr=subprocess.STDOUT,
            preexec_fn=os.setsid)
        SIM_PROC = sim_proc
        time.sleep(1.0)
        # 真机电机 Type-18 主动上报是持久参数，上电即流（boot 即有 /joint_states）；
        # 全新 sim 实例默认不上报，需外部 0x18-ON 触发。统一栈（F111）开机不桥接
        # power_sequence 的 /can_tx_frames，插件的 0x18-ON 要等到 enable 才发——仿真侧
        # 这里在 spawn 后补发 7x 0x18-ON（data[6]=1），模拟"持久上报已开"，让
        # /joint_states 从 boot 就有得读，对齐真机 boot 语义。reset/on_activate 后的
        # 重挂仍由插件 0x18-ON 驱动（P1/P2 帧级校验不受影响）。
        for motor_id in range(1, 8):
            can_id = (0x18 << 24) | (0xFD << 8) | motor_id
            subprocess.run(
                ["cansend", VCAN, f"{can_id:08X}#0000000000000100"],
                check=True, capture_output=True)
        time.sleep(0.3)
        log = f"/tmp/f113_stack_{tag}.log"
        launch_args = ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
                       "hardware:=can", f"can_interface:={VCAN}",
                       "use_rviz:=false", "use_mqtt:=false",
                       "use_teleop:=false", "use_self_test:=false",
                       "use_diagnostics:=false", "use_rosbag:=false"]
        logf = open(log, "w")
        LAUNCH_PROC = subprocess.Popen(
            launch_args, stdout=logf, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid)
        print(f"[{tag}] launch started (pid {LAUNCH_PROC.pid})", flush=True)
        return log

    def stop_stack(log):
        cleanup_launch(log)
        cleanup_sim("")

    def wait_service(cli, timeout=90.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cli.service_is_ready():
                return True
            time.sleep(0.5)
        return False

    def call_named(cli_name, service, expect_ok, timeout=25.0):
        cli = node.create_client(Trigger, service)
        if not cli.wait_for_service(timeout_sec=timeout):
            return False, "service not up"
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            fut = cli.call_async(Trigger.Request())
            if wait_future(fut, 10.0):
                r = fut.result()
                if r.success == expect_ok:
                    return True, r.message
                if expect_ok and not r.success:
                    # transient contention (e.g. mid-switch): retry
                    time.sleep(0.5)
                    continue
                return False, f"result mismatch: success={r.success} msg={r.message!r}"
            time.sleep(0.5)
        return False, "timeout"

    def call_enable(timeout=60.0):
        """Call /a3/arm/enable once the service is up. Returns (success, msg)."""
        cli = node.create_client(Trigger, "/a3/arm/enable")
        if not wait_service(cli, 120.0):
            return None, "enable service never up within 120s"
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            fut = cli.call_async(Trigger.Request())
            if wait_future(fut, 15.0):
                r = fut.result()
                if r.success:
                    return True, r.message
                if "no /joint_states" not in r.message:
                    return r.success, r.message
            time.sleep(0.5)
        return False, "enable timeout"

    def boot_running(timeout=45.0):
        """Idempotent 'start' bursts until the power gate opens (Running)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if power.gate is True and power.state == "Running":
                return True
            commands.send("start", hold=0.15)
            time.sleep(0.1)
        return False

    def check_enable_ok(cycle_name, timeout=60.0):
        """Enable -> success + READY + 0x18-ON quota. Returns True if ok."""
        sniff.reset()
        ok, msg = call_enable(timeout=timeout)
        rep.check(f"{cycle_name}: enable success", bool(ok),
                  (msg or "")[:90])
        if not ok:
            return False
        ready = fsm.wait("READY", timeout=15.0)
        rep.check(f"{cycle_name}: FSM READY", ready, f"state={fsm.state}")
        if not ready:
            return False
        cmd0x = sniff.cmd_on_total()
        rep.check(f"{cycle_name}: on_activate 0x18-ON >= 7", cmd0x >= 7,
                  f"count={cmd0x}")
        if cmd0x < 7:
            return False
        time.sleep(1.2)
        rates = {m: sniff.stream_rate(m, 1.0) for m in MOTORS}
        ok_s = all(v >= 15 for v in rates.values())
        rep.check(f"{cycle_name}: 7 motors streaming 0x18 >= 15fps/1s", ok_s,
                  f"rates(1s)={rates}")
        return ok_s

    def check_disable_ok(cycle_name, timeout=25.0):
        """Disable -> DISABLED + 0x18-ON re-arm + stream/JS stays live."""
        js_before = js.count
        pos_before = list(js.positions)
        sniff.reset()
        ok_dis, dis_msg = call_named(f"dis_{cycle_name}", "/a3/arm/disable", True,
                                     timeout=timeout)
        rep.check(f"{cycle_name}: /a3/arm/disable success", ok_dis,
                  (dis_msg or "")[:90])
        if not ok_dis:
            return False
        disabled = fsm.wait("DISABLED", timeout=20.0)
        rep.check(f"{cycle_name}: FSM DISABLED", disabled, f"state={fsm.state}")
        if not disabled:
            return False
        cmd0x = sniff.cmd_on_total()
        rep.check(f"{cycle_name}: on_deactivate 0x18-ON >= 7", cmd0x >= 7,
                  f"count={cmd0x}")
        time.sleep(1.5)
        rates = {m: sniff.stream_rate(m, 1.0) for m in MOTORS}
        ok_s = all(v >= 15 for v in rates.values())
        rep.check(f"{cycle_name}: stream continues in DISABLED >= 15fps/1s", ok_s,
                  f"rates(1s)={rates}")
        ticks = js.count - js_before
        rep.check(f"{cycle_name}: /joint_states ticks during DISABLED", ticks > 0,
                  f"{ticks} msgs in DISABLED window")
        bad = sum(
            1 for p in js.positions
            if abs(abs(p) - POS_SENTINEL) < POS_SENTINEL_EPS)
        rep.check(f"{cycle_name}: no +/-12.49635 freeze marker", bad == 0,
                  f"positions={[round(p, 3) for p in js.positions][:7]}")
        return cmd0x >= 7 and ok_s and ticks > 0 and bad == 0

    def test():
        # ---------- P1+P2+P3: three enable/disable cycles on ONE stack ----
        rep.phase("P1-P3 同一栈 enable/disable x3（帧级 B/A + 闭环 C）")
        log = start_stack("p13")
        try:
            if not boot_running():
                rep.check("start bursts -> power Running", False,
                          f"gate={power.gate} state={power.state!r}")
                return
            rep.check("start bursts -> power Running (idempotent, no Idle latch)",
                      True, f"gate={power.gate} state={power.state!r}")

            if not check_enable_ok("cycle 1 enable"):
                return
            if not check_disable_ok("cycle 1 disable"):
                return
            if not check_enable_ok("cycle 2 enable"):
                return
            if not check_disable_ok("cycle 2 disable"):
                return
            if not check_enable_ok("cycle 3 enable"):
                return
            if not check_disable_ok("cycle 3 disable"):
                return
        finally:
            stop_stack(log)

        # ---------- P4: dark motor on a fresh stack -----------------------
        rep.phase("P4 暗电机（m4 no 0x18 / no type-2）：enable 必失败 only 6/7")
        write_noar([4])
        sniff.reset()
        log = start_stack("p4")
        try:
            if not boot_running():
                rep.check("P4 power Running (dark m4 boot)", False,
                          f"gate={power.gate} state={power.state!r}")
                return
            rep.check("P4 power Running before abort", True)
            sniff.reset()
            ok, msg = call_enable(timeout=60.0)
            rep.check("P4 enable MUST fail (dark m4 aborts on_activate)", not ok,
                      (msg or "")[:120])
            never_ready = not fsm.wait("READY", timeout=8.0)
            rep.check("P4 never reaches READY", never_ready, f"state={fsm.state}")
            time.sleep(2.0)
            src = open(log).read()
            aborted = "activate aborted: only 6/7" in src
            rep.check("P4 stack log shows 'activate aborted: only 6/7'", aborted)
            m4 = sniff.stream_count(4)
            s4 = {m: sniff.stream_count(m) for m in MOTORS}
            rep.check("P4 motor 4 emits zero 0x18 frames", m4 == 0,
                      f"per-motor={s4}")
        finally:
            stop_stack(log)

        # ---------- P5: clear injection, fresh stack, recovery -------------
        rep.phase("P5 清除注入 + fresh 栈：m4 恢复 7/7 -> READY")
        write_noar([])
        sniff.reset()
        log = start_stack("p5")
        try:
            if not boot_running():
                rep.check("P5 power Running", False,
                          f"gate={power.gate} state={power.state!r}")
                return
            rep.check("P5 power Running", True)
            if not check_enable_ok("P5 enable"):
                return
            m4b = sniff.stream_count(4)
            rep.check("P5 motor 4 streaming again after clear", m4b > 0,
                      f"m4 frames={m4b}")
        finally:
            stop_stack(log)

        # guarantee noar is cleared even if P4 crashed mid-phase
        write_noar([])

    try:
        test()
    finally:
        write_noar([])
        sniff.stop()
        executor.shutdown()
        spin_th.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    print(f"\n{'='*50}")
    print(f"TOTAL: {rep.passed} passed, {rep.failed} failed")
    if rep.failed:
        print("RESULT: FAIL")
        sys.exit(1)
    print("RESULT: PASS")
    sys.exit(0)


if __name__ == "__main__":
    main()