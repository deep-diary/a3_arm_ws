#!/usr/bin/env python3
"""F107 — 额定负载 1.5 kg 静态力矩门禁 + 滚动窗口占空比门禁 验收。

一条命令自动起栈（mock，产品入口 a3_bringup.launch.py hardware:=mock），
按四阶段验证并输出 PASS/FAIL 报告与退出码，结束自动清栈：

  P1 零负载回归  P2 额定负载静态门  P3 越限/回滚  P4 占空比门

所有门禁参数在控制器内实时读取，本脚本通过 FSM 的 set_parameters 服务
运行时调参，不依赖 launch 文件改动。隔离域 ROS_DOMAIN_ID=107。
"""
import os
import signal
import subprocess
import sys
import threading
import time

WS_DIR = "/home/cat/a3_arm_ws"
ARM_JOINTS = ["L1_joint", "L2_joint", "L3_joint",
              "L4_joint", "L5_joint", "L6_joint"]
ALL_JOINTS = ARM_JOINTS + ["L7_joint"]
DOMAIN = "107"
POSE_TOL = 0.06

LOW = [0.0, 1.57, 0.0, 0.0, 0.0, 0.0, 0.0]
ZERO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

LAUNCH_PROC = None


def ensure_env() -> None:
    """Re-exec under a3_shell_env.sh so rclpy/ament packages are importable."""
    if os.environ.get("A3_F107_ENVED") == "1":
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
    env["A3_F107_ENVED"] = "1"
    os.execvpe(sys.executable,
               [sys.executable, os.path.abspath(__file__), *sys.argv[1:]], env)


class JointRecorder:
    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self.msg = None
        from sensor_msgs.msg import JointState
        node.create_subscription(JointState, "/joint_states", self._cb, 20)

    def _cb(self, msg):
        with self._lock:
            self.msg = msg

    def latest(self):
        with self._lock:
            return self.msg

    def positions(self, joints):
        for _ in range(50):
            msg = self.latest()
            if msg and all(j in msg.name for j in joints):
                return [msg.position[msg.name.index(j)] for j in joints]
            time.sleep(0.1)
        raise RuntimeError(f"joint_states missing among {joints}")


class StateRecorder:
    def __init__(self, node):
        self._lock = threading.Lock()
        self.state = ""
        from a3_msgs.msg import ArmStatus
        node.create_subscription(ArmStatus, "/a3/arm_status", self._cb, 10)

    def _cb(self, msg):
        with self._lock:
            self.state = msg.state

    def get(self):
        with self._lock:
            return self.state


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


def call_service(node, srv_type, srv_name, request, timeout=5.0, wait=0.5):
    cli = node.srv_clients.get(srv_name)
    if cli is None:
        cli = node.create_client(srv_type, srv_name)
        node.srv_clients[srv_name] = cli
    if not cli.wait_for_service(timeout_sec=wait):
        raise RuntimeError(f"service unavailable: {srv_name}")
    fut = cli.call_async(request)
    if not wait_future(fut, timeout):
        raise RuntimeError(f"service timeout: {srv_name}")
    return fut.result()


def set_param(node, name, value):
    """Set a parameter on /a3_arm_controller via its set_parameters service."""
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue
    cli = node.srv_clients.get("fsm_set_parameters")
    if cli is None:
        cli = node.create_client(SetParameters,
                                 "/a3_arm_controller/set_parameters")
        node.srv_clients["fsm_set_parameters"] = cli
    if not cli.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("FSM set_parameters unavailable")
    pv = ParameterValue()
    if isinstance(value, bool):
        pv.type = 1  # ParameterType.PARAMETER_BOOL
        pv.bool_value = value
    else:
        pv.type = 3  # ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(value)
    p = Parameter(name=name, value=pv)
    req = SetParameters.Request(parameters=[p])
    fut = cli.call_async(req)
    if not wait_future(fut, 5.0):
        raise RuntimeError(f"set_parameters timeout: {name}")
    results = fut.result().results
    if not results or not results[0].successful:
        raise RuntimeError(
            f"set_parameters rejected {name}: "
            f"{results[0].reason if results else 'no result'}")


def wait_ready(state_rec, timeout=15.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and state_rec.get() != "READY":
        time.sleep(0.05)
    return state_rec.get() == "READY"


def jog(node, positions, duration):
    from a3_msgs.srv import SetJointPositions
    req = SetJointPositions.Request(positions=[float(x) for x in positions],
                                    duration=float(duration))
    return call_service(node, SetJointPositions,
                        "/a3/arm/set_joint_positions", req,
                        timeout=10.0, wait=10.0)


def move_to(node, positions, duration):
    from a3_msgs.srv import MoveToJointPositions
    req = MoveToJointPositions.Request(positions=[float(x) for x in positions],
                                       duration_s=float(duration))
    return call_service(node, MoveToJointPositions, "/a3/arm/move_to", req,
                        timeout=10.0, wait=10.0)


def set_payload(node, mass, com=(0.0, 0.0, 0.0)):
    from a3_msgs.srv import SetPayload
    req = SetPayload.Request(mass_kg=float(mass),
                             com_m=[float(v) for v in com])
    return call_service(node, SetPayload, "/a3/arm/set_payload", req,
                        timeout=10.0, wait=10.0)


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
            proc.wait(timeout=12.0)
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
    node = rclpy.create_node("f107_payload_duty_acceptance")
    node.srv_clients = {}
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    recorder = JointRecorder(node)
    state_rec = StateRecorder(node)
    rep = Report()
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_path = f"/tmp/f107_launch_{ts}.log"

    def signal_handler(signum, frame):
        print(f"\nsignal {signum} received, cleaning up...", flush=True)
        cleanup_launch(log_path)
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    launch_args = ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
                   "hardware:=mock", "use_rviz:=false", "use_mqtt:=false",
                   "use_teleop:=false"]
    logf = open(log_path, "w")
    LAUNCH_PROC = subprocess.Popen(
        launch_args, stdout=logf, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid)
    print(f"launch started (pid {LAUNCH_PROC.pid}): {' '.join(launch_args)}",
          flush=True)

    try:
        # ---------------- P1 zero-load regression ----------------
        rep.phase("P1 零负载回归：使能 + jog 运动路径不受门禁影响")
        end = time.monotonic() + 45.0
        en = None
        while time.monotonic() < end:
            try:
                en = call_service(node, Trigger, "/a3/arm/enable",
                                  Trigger.Request(), timeout=10.0, wait=15.0)
                if en.success or "joint_states" not in en.message:
                    break
            except Exception:
                pass
            time.sleep(1.0)
        rep.check("/a3/arm/enable success at zero pose (0 kg)",
                  en is not None and en.success,
                  en.message if en else "enable unreachable")
        rep.check("FSM state READY", wait_ready(state_rec),
                  f"state={state_rec.get()}")

        set_param(node, "goto_use_moveit", False)

        r1 = jog(node, [0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.3], 0.5)
        rep.check("zero-load jog (incl. L7) accepted", r1.success, r1.message)
        rep.check("FSM back to READY after jog", wait_ready(state_rec))
        p = recorder.positions(ALL_JOINTS)
        err = max(abs(p[i] - t) for i, t in
                  enumerate([0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.3]))
        rep.check(f"jog tracking err <{POSE_TOL}", err < POSE_TOL,
                  f"max_err={err:.4f}")

        r2 = jog(node, ZERO, 0.5)
        rep.check("zero-load return jog accepted", r2.success, r2.message)
        rep.check("FSM back to READY", wait_ready(state_rec))

        # ---------------- P2 rated payload static gate ----------------
        rep.phase("P2 额定负载 1.5 kg：可行位形放行 / 高重力位形拒绝")
        rm = move_to(node, LOW, 3.0)
        rep.check("zero-load move_to LOW accepted", rm.success, rm.message)
        rep.check("FSM READY at LOW", wait_ready(state_rec))
        p = recorder.positions(ALL_JOINTS)
        err = max(abs(a - b) for a, b in zip(p, LOW))
        rep.check(f"at LOW pose err <{POSE_TOL}", err < POSE_TOL,
                  f"max_err={err:.4f}")

        sp = set_payload(node, 1.5)
        rep.check("set_payload 1.5 kg at LOW succeeds", sp.success, sp.message)

        low_l1 = list(LOW)
        low_l1[0] = 0.2
        r3 = jog(node, low_l1, 0.5)
        rep.check("payload low-torque move (L1 rotate) accepted",
                  r3.success, r3.message)
        rep.check("FSM READY", wait_ready(state_rec))
        r4 = jog(node, LOW, 0.5)
        rep.check("payload return move accepted", r4.success, r4.message)
        rep.check("FSM READY", wait_ready(state_rec))

        r5 = jog(node, ZERO, 3.0)
        msg5 = r5.message
        rejected = (not r5.success and "static torque gate" in msg5
                    and "L3_joint" in msg5 and "-7.02" in msg5)
        rep.check("move toward zero rejected with L3 torque named",
                  rejected, msg5)

        # ---------------- P3 range / rollback ----------------
        rep.phase("P3 越限拒绝 + 当前位形回滚（旧负载值不变）")
        sp2 = set_payload(node, 2.0)
        rep.check("set_payload 2.0 kg (>rated) rejected",
                  not sp2.success and "out of range" in sp2.message,
                  sp2.message)

        r6 = jog(node, ZERO, 3.0)
        msg6 = r6.message
        rollback_ok = (not r6.success and "L3_joint" in msg6
                       and "-7.02" in msg6 and "-8.5" not in msg6)
        rep.check("old payload unchanged (gate still reports 1.5kg torque)",
                  rollback_ok, msg6)

        sp0 = set_payload(node, 0.0)
        rep.check("set_payload back to 0 kg", sp0.success, sp0.message)
        rm2 = move_to(node, ZERO, 3.0)
        rep.check("zero-load move_to zero accepted", rm2.success, rm2.message)
        rep.check("FSM READY at zero", wait_ready(state_rec))

        sp3 = set_payload(node, 1.5)
        rep.check("set_payload 1.5 kg at high-torque pose rejected",
                  not sp3.success and "current pose static gate" in sp3.message
                  and "L3_joint" in sp3.message, sp3.message)

        r7 = jog(node, [0.05, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0], 0.5)
        rep.check("zero-load jog after rollback accepted (old value 0 kept)",
                  r7.success, r7.message)
        rep.check("FSM READY", wait_ready(state_rec))
        r8 = jog(node, ZERO, 0.5)
        rep.check("return jog accepted", r8.success, r8.message)
        rep.check("FSM READY", wait_ready(state_rec))

        # ---------------- P4 duty gate ----------------
        rep.phase("P4 占空比门：窗口 20 s / 比例 0.2（预算 4 s）")
        set_param(node, "duty_window_s", 20.0)
        set_param(node, "duty_max_ratio", 0.2)
        # 等前序阶段的运动段全部滚出 20 s 窗口，保证占空比序列从空窗开始
        print("  waiting 21s for pre-existing motion to age out of window...",
              flush=True)
        time.sleep(21.0)

        target_a = [0.05, 0.0, -0.05, 0.0, 0.0, 0.0, 0.0]
        accepted_cnt = 0
        reject_msg = ""
        for i in range(14):
            tgt = target_a if i % 2 == 0 else ZERO
            rr = jog(node, tgt, 0.3)
            if rr.success:
                accepted_cnt += 1
                if not wait_ready(state_rec, timeout=5.0):
                    reject_msg = "state did not return READY"
                    break
            else:
                reject_msg = rr.message
                break

        rep.check("repeated short jogs eventually duty-blocked",
                  reject_msg != "" and "duty gate rejected" in reject_msg
                  and "wait 0s" not in reject_msg,
                  f"accepted={accepted_cnt}/14; {reject_msg}")
        rep.check("budget accounting realistic (10-13 x 0.3s in 4s budget)",
                  10 <= accepted_cnt <= 13, f"accepted={accepted_cnt}")

        time.sleep(2.0)
        set_param(node, "duty_window_s", 1.0)
        set_param(node, "duty_max_ratio", 1.0)
        r9 = jog(node, target_a, 0.3)
        rep.check("jog accepted after shrinking window (old records expired)",
                  r9.success, r9.message)
        rep.check("FSM READY", wait_ready(state_rec))
        r10 = jog(node, ZERO, 0.3)
        rep.check("return jog accepted", r10.success, r10.message)
        rep.check("FSM READY", wait_ready(state_rec))

        set_param(node, "duty_window_s", 600.0)
        set_param(node, "duty_max_ratio", 0.8)

    except Exception as exc:
        rep.check("phase execution", False, f"exception: {exc}")

    # ---------------- teardown ----------------
    try:
        call_service(node, Trigger, "/a3/arm/disable", Trigger.Request(),
                     timeout=8.0)
    except Exception:
        pass
    cleanup_launch(log_path)
    time.sleep(3.0)

    resid = []
    for _ in range(15):
        chk = subprocess.run(
            ["bash", "-c",
             f"source {WS_DIR}/scripts/a3_shell_env.sh >/dev/null 2>&1 && "
             f"ROS_DOMAIN_ID={DOMAIN} ros2 node list"],
            capture_output=True, text=True, timeout=10.0)
        resid = [ln for ln in chk.stdout.splitlines()
                 if ln.strip() and ln.strip() != "/f107_payload_duty_acceptance"]
        if not resid:
            break
        time.sleep(1.0)
    rep.check("no residual nodes on isolated domain after teardown",
              not resid, ",".join(resid) if resid else "clean")

    print(f"\n==== F107 RESULT: {rep.passed} PASS / {rep.failed} FAIL ====",
          flush=True)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if rep.failed == 0 else 1)


if __name__ == "__main__":
    main()
