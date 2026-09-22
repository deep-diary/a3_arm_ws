#!/usr/bin/env python3
"""F84 acceptance: temperature/fault-word decode + thermal FSM gating.

Self-contained: launches vcan_motor_sim (health injection via JSON file)
and the full CAN stack (a3_bringup.launch.py hardware:=can), runs the
checks, tears everything down, then runs a mock-stack regression.

  python3 scripts/a3_test/f84_thermal_fault_acceptance.py [DOMAIN_ID]

Acceptance (docs/edge/REQUIREMENTS.md F84):
  1. list_hardware_interfaces shows 7 <joint>/temperature state interfaces;
     at 30 C /a3/motor/states has 7 fresh entries (temp~30, fault_mask=0,
     enabled=true); a3_hardware:motor_health diagnostic OK
  2. motor 3 at 92 C -> motor_health WARN + ArmStatus.temp_warn=true, but a
     quintic FJT still returns error_code=0
  3. motor 3 at 96 C -> F44 path READY->SAFE_PARK->disable->COOLING; enable
     refused while hot; at 80 C enable succeeds -> READY
  4. motor 5 fault word=4 -> emergency reset -> FAULT; motor_health ERROR
  5. mock stack (no temperature interface) regression: enable -> READY

Exit code 0 = all checks passed.
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from a3_can_bridge.msg import MotorStates
from a3_msgs.msg import ArmStatus
from controller_manager_msgs.srv import ListHardwareInterfaces
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# NOTE: in this Humble/genpy combo DiagnosticStatus.OK/.WARN/.ERROR are bytes
# (b'\x00'..b'\x03') while message fields may arrive as either int or bytes;
# on_diag normalizes fields to int, so compare against plain ints.
LEVEL_OK, LEVEL_WARN, LEVEL_ERROR = 0, 1, 2

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "84"
DOMAIN_MOCK = str(int(DOMAIN) + 2)
IFACE = os.environ.get("F84_CAN_IF", "vcan4")
STACK_LOG = "/tmp/f84_stack.log"
SIM_LOG = "/tmp/f84_sim.log"
MOCK_LOG = "/tmp/f84_mock.log"
HEALTH_FILE = os.environ.get("F84_HEALTH_FILE", "/tmp/f84_health.json")
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


class ProcessGroup:
    def __init__(self, argv, log_path, env=None):
        self.log = open(log_path, "wb")
        self.proc = subprocess.Popen(
            argv, stdout=self.log, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True, env=env)

    def kill(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        self.log.close()


def make_env(domain):
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["PYTHONNOUSERSITE"] = "1"
    return env


def ensure_vcan(interface):
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "add", "dev", interface,
         "type", "vcan"], input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "set", interface, "up"],
        input=b"temppwd\n", capture_output=True)


def spin(node, t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(node, cli, request, timeout=30.0):
    fut = cli.call_async(request)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    raise RuntimeError(f"service timeout: {cli.srv_name}")


def write_health(payload):
    tmp = HEALTH_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(payload)
    os.replace(tmp, HEALTH_FILE)
    time.sleep(0.12)  # sim polls mtime every 50 ms


def wait_state(node, holder, states, timeout_s):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        spin(node, 0.1)
        if holder.get("state") in states:
            return True
    return False


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    ensure_vcan(IFACE)
    write_health("{}")
    env = make_env(DOMAIN)

    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", IFACE, "--health-file", HEALTH_FILE],
        SIM_LOG, env=env)
    time.sleep(0.8)
    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={IFACE}",
         "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"],
        STACK_LOG, env=env)

    rclpy.init()
    node = Node("f84_acceptance")
    hw_cli = node.create_client(
        ListHardwareInterfaces,
        "/controller_manager/list_hardware_interfaces")
    enable_cli = node.create_client(Trigger, "/a3/arm/enable")
    disable_cli = node.create_client(Trigger, "/a3/arm/disable")

    status_holder = {"state": None, "temp_warn": None}
    states_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    motor_holder = {"msg": None}
    diag_holder = {"node": node, "diag": {}}

    def on_status(m):
        status_holder["state"] = m.state
        status_holder["temp_warn"] = m.temp_warn

    def on_motor(m):
        motor_holder["msg"] = m

    def on_diag(m):
        for st in m.status:
            level = st.level
            if isinstance(level, (bytes, bytearray)):
                level = level[0]
            diag_holder["diag"][st.name] = level

    js_holder = {"n": 0}
    node.create_subscription(ArmStatus, "/a3/arm_status", on_status, 10)
    node.create_subscription(MotorStates, "/a3/motor/states",
                             on_motor, states_qos)
    node.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 10)
    node.create_subscription(JointState, "/joint_states",
                             lambda m: js_holder.__setitem__("n", js_holder["n"] + 1),
                             10)

    fatal = False
    try:
        if not hw_cli.wait_for_service(timeout_sec=30):
            raise RuntimeError("controller_manager absent in 30s")
        spin(node, 3.0)

        # ---- check 1a: temperature state interfaces, available ----
        res = call(node, hw_cli, ListHardwareInterfaces.Request())
        names = [i.name for i in res.state_interfaces]
        temp_ifaces = [n for n in names if n.endswith("/temperature")]
        check("1a 7 <joint>/temperature state interfaces present",
              len(temp_ifaces) == 7, f"{sorted(temp_ifaces)}")

        # Wait for joint_state_broadcaster data so the enable position-limit
        # gate doesn't read empty positions (F83 harness does the same).
        t0 = time.monotonic()
        while js_holder["n"] == 0 and time.monotonic() - t0 < 25:
            spin(node, 0.1)
        spin(node, 0.5)
        assert js_holder["n"] > 0, "no /joint_states before enable"

        # enable (worst-case FSM ~21 s on real; sim faster)
        assert enable_cli.wait_for_service(timeout_sec=20), "enable service absent"
        en_resp = call(node, enable_cli, Trigger.Request(), timeout=30)
        if not en_resp.success:
            raise RuntimeError(f"enable rejected: {en_resp.message}")
        if not wait_state(node, status_holder, {"READY"}, 30):
            raise RuntimeError(f"not READY after enable: {status_holder}")
        spin(node, 0.5)

        # ---- check 1b: MotorStates fresh / 30 C / fault 0 / enabled ----
        msg = motor_holder["msg"]
        ok_states = (msg is not None and len(msg.states) == 7
                     and all(s.fresh for s in msg.states)
                     and all(25.0 <= s.temperature_c <= 35.0
                             for s in msg.states)
                     and all(s.fault_mask == 0 for s in msg.states)
                     and all(s.enabled for s in msg.states))
        detail = ""
        if msg is not None and msg.states:
            s0 = msg.states[0]
            detail = (f"n={len(msg.states)} fresh={s0.fresh} "
                      f"t={s0.temperature_c:.1f} fault={s0.fault_mask} "
                      f"en={s0.enabled}")
        check("1b MotorStates: 7 fresh, ~30 C, fault_mask=0, enabled=true",
              ok_states, detail)

        # ---- check 1c: motor_health OK ----
        mh = diag_holder["diag"].get("a3_hardware:motor_health")
        check("1c a3_hardware:motor_health OK at 30 C", mh == LEVEL_OK,
              f"level={mh}")

        # ---- check 2: 92 C -> WARN + temp_warn, motion still fine ----
        write_health('{"motor": 3, "temp_c": 92.0}')
        t0 = time.monotonic()
        warn_ok = False
        while time.monotonic() - t0 < 8:
            spin(node, 0.1)
            m = motor_holder["msg"]
            m3 = next((s for s in m.states if s.motor_id == 3), None) \
                if m else None
            if (m3 is not None and m3.temperature_c >= 91.0
                    and diag_holder["diag"].get(
                        "a3_hardware:motor_health") == LEVEL_WARN
                    and status_holder["temp_warn"]):
                warn_ok = True
                break
        check("2a 92 C: motor_health WARN + ArmStatus.temp_warn=true",
              warn_ok)

        err_arm, err_grip = move_small(node)
        check("2b quintic FJT error_code=0 during thermal WARN",
              err_arm == 0 and err_grip == 0, f"err={err_arm}/{err_grip}")

        # ---- check 3: 96 C -> SAFE_PARK -> COOLING; enable refused; 80 C OK
        write_health('{"motor": 3, "temp_c": 96.0}')
        parked = wait_state(
            node, status_holder, {"COOLING"}, 25)
        check("3a 96 C: F44 safe-park -> COOLING", parked,
              f"state={status_holder['state']}")

        refused = call(node, enable_cli, Trigger.Request(), timeout=15)
        refusal_ok = not refused.success
        check("3b enable refused while >= 90 C", refusal_ok,
              f"success={refused.success} msg={refused.message}")

        write_health('{"motor": 3, "temp_c": 80.0}')
        spin(node, 1.0)  # let cooling check see fresh 80 C frames
        call(node, enable_cli, Trigger.Request(), timeout=30)
        cooled_ready = wait_state(
            node, status_holder, {"READY"}, 30)
        check("3c at 80 C: enable succeeds -> READY", cooled_ready,
              f"state={status_holder['state']}")

        # ---- check 4: fault word on motor 5 -> FAULT + motor_health ERROR
        write_health('{"motor": 5, "fault": 4}')
        faulted = wait_state(
            node, status_holder, {"FAULT"}, 15)
        spin(node, 0.4)
        mh_level = diag_holder["diag"].get("a3_hardware:motor_health")
        check("4 fault=4 on motor 5: emergency reset -> FAULT, motor_health ERROR",
              faulted and mh_level == LEVEL_ERROR,
              f"state={status_holder['state']} diag={mh_level}")

    except Exception:
        import traceback
        traceback.print_exc()
        fatal = True
    finally:
        write_health("{}")
        stack.kill()
        sim.kill()

    # ---- check 5: mock stack regression (no temperature interface) ----
    if not fatal:
        env_mock = make_env(DOMAIN_MOCK)
        mock_stack = ProcessGroup(
            ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
             "hardware:=mock",
             "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"],
            MOCK_LOG, env=env_mock)
        try:
            # Harness node stays on DOMAIN; use a second node for the mock
            # domain — rclpy nodes cannot change domain, so shell out the
            # enable via ros2 service on DOMAIN_MOCK.
            t0 = time.monotonic()
            ready = False
            while time.monotonic() - t0 < 60:
                time.sleep(2)
                en = subprocess.run(
                    ["ros2", "service", "call", "/a3/arm/enable",
                     "std_srvs/srv/Trigger", "{}"],
                    capture_output=True, text=True,
                    env={**env_mock, "ROS_DOMAIN_ID": DOMAIN_MOCK},
                    timeout=20)
                if "success=True" in en.stdout or "success=true" in en.stdout:
                    q = subprocess.run(
                        ["ros2", "topic", "echo", "/a3/arm_status",
                         "a3_msgs/msg/ArmStatus", "--once", "--field",
                         "state"],
                        capture_output=True, text=True,
                        env={**env_mock, "ROS_DOMAIN_ID": DOMAIN_MOCK},
                        timeout=15)
                    if "READY" in q.stdout:
                        ready = True
                        break
            check("5 mock stack regression: enable -> READY", ready)
        finally:
            subprocess.run(
                ["ros2", "service", "call", "/a3/arm/disable",
                 "std_srvs/srv/Trigger", "{}"],
                capture_output=True,
                env={**env_mock, "ROS_DOMAIN_ID": DOMAIN_MOCK},
                timeout=20)
            mock_stack.kill()

    node.destroy_node()
    rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F84 acceptance: {passed}/{total} ====", flush=True)
    if fatal:
        return 2
    return 0 if passed == total else 1


# Imported here to keep the top of file readable; reuse F83 motion helper.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from f83_enable_choreography_acceptance import move_small  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
