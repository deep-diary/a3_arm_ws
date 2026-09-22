#!/usr/bin/env python3
"""F89b acceptance: FSM free-drive via standard atomic switch_controller.

On the standard ros2_control product stack the FSM teach services
(start_teach / stop_teach — the same services PS4 Share/Options call) must
swap arm_controller <-> zero_torque_controller with ONE atomic STRICT
switch_controller request; the legacy /a3/zero_torque/start|stop services
do not exist on that stack.

All simulation-only (real arm stays powered off): vcan_motor_sim with the
URDF gravity truth (unit scales) + product stack a3_bringup hardware:=can.

Sequence:
  1. Rejections before enable: start_teach while DISABLED; stop_teach while
     not TEACH.
  2. enable -> READY.
  3. start_teach -> TEACH; list_controllers: zero_torque active, arm
     inactive; ~/gravity_torque keeps publishing; settle 1 s then 1 s drift
     <= 0.02 rad.
  4. Duplicate start_teach while TEACH rejected.
  5. Exit-switch fail-safe: point freedrive_arm_controller at a bogus name
     at runtime -> STRICT rejects the exit request -> response success=false
     and state stays TEACH (gravity comp still active). Restore param.
  6. stop_teach -> READY; arm active, zero_torque inactive; message has
     auto-saved; /a3/control_mode back to READY.

python3 scripts/a3_test/f89b_freedrive_switch_acceptance.py [DOMAIN]

Exit code 0 = all checks passed.
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.msg import Parameter as RclParam
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from a3_msgs.msg import ArmStatus

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "92"
VCAN = os.environ.get("F89B_VCAN", "vcan89b")
SIM_LOG = "/tmp/f89b_accept_sim.log"
STACK_LOG = "/tmp/f89b_accept_stack.log"
RESULTS = []

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
BOGUS_ARM = "nonexistent_arm_controller"


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


class ProcessGroup:
    def __init__(self, argv, log_path, env=None):
        self.log = open(log_path, "wb")
        self.proc = subprocess.Popen(
            argv, stdout=self.log, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True, env=env)

    def terminate(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.hard_kill()

    def hard_kill(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc.wait(timeout=10)


def make_env():
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = DOMAIN
    env["PYTHONNOUSERSITE"] = "1"
    return env


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


def ensure_vcan(interface):
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "add", "dev", interface, "type", "vcan"],
        input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "set", interface, "up"],
        input=b"temppwd\n", capture_output=True)


def clear_injection_files():
    # Leftover dynamics/push/fault/silence files would otherwise leak into
    # this acceptance (the sim watches them continuously).
    for path in ("/tmp/f87_dynamics.json", "/tmp/f73_ext.json",
                 "/tmp/f81_silence.json", "/tmp/f84_health.json"):
        with open(path, "w") as f:
            f.write("{}")


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f89b_acceptance")
        self.js = None
        self.js_stamp = None
        self.status = None
        self.status_stamp = None
        self.grav = None
        self.grav_stamp = None
        self.mode = None
        self.mode_stamp = None
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        for qos in (
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        ):
            self.create_subscription(
                JointState, "/joint_states", self._on_js, qos)
        self.create_subscription(
            ArmStatus, "/a3/arm_status", self._on_status, 10)
        self.create_subscription(
            JointState, "/zero_torque_controller/gravity_torque",
            self._on_grav, 10)
        self.create_subscription(
            String, "/a3/control_mode", self._on_mode, 10)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")
        self.teach_start_cli = self.create_client(
            Trigger, "/a3/arm/start_teach")
        self.teach_stop_cli = self.create_client(
            Trigger, "/a3/arm/stop_teach")
        self.list_cli = self.create_client(
            ListControllers, "/controller_manager/list_controllers")
        self.set_param_cli = self.create_client(
            SetParametersAtomically,
            "/a3_arm_controller/set_parameters_atomically")
        self.get_param_cli = self.create_client(
            GetParameters, "/a3_arm_controller/get_parameters")

    def _on_js(self, msg):
        self.js = msg
        self.js_stamp = time.time()

    def _on_status(self, msg):
        self.status = msg
        self.status_stamp = time.time()

    def _on_grav(self, msg):
        self.grav = msg
        self.grav_stamp = time.time()

    def _on_mode(self, msg):
        if msg.data:
            self.mode = msg.data
            self.mode_stamp = time.time()

    def arm_positions(self):
        return [self.js.position[self.js.name.index(n)] for n in ARM_JOINTS]

    def wait_state(self, expected, timeout=40):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            spin(self, 0.1)
            if self.status is not None and self.status.state == expected:
                return True
        return False

    def enable_to_ready(self, timeout=40):
        assert self.enable_cli.wait_for_service(timeout_sec=20), \
            "enable service absent"
        spin(self, 2.0)
        try:
            resp = call(self, self.enable_cli, Trigger.Request(),
                        timeout=timeout)
        except RuntimeError:
            spin(self, 2.0)
            resp = call(self, self.enable_cli, Trigger.Request(),
                        timeout=timeout)
        if not resp.success and "no /joint_states" in resp.message:
            for _ in range(8):
                spin(self, 1.0)
                resp = call(self, self.enable_cli, Trigger.Request(),
                            timeout=timeout)
                if resp.success or "no /joint_states" not in resp.message:
                    break
        if not resp.success:
            raise RuntimeError(f"enable rejected: {resp.message}")
        if not self.wait_state("READY", timeout=timeout):
            raise RuntimeError("state did not reach READY after enable")

    def controller_states(self):
        resp = call(self, self.list_cli, ListControllers.Request(),
                    timeout=10)
        return {c.name: c.state for c in resp.controller}

    def set_fsm_string_param(self, name, value):
        req = SetParametersAtomically.Request()
        req.parameters = [RclParam(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                 string_value=value))]
        resp = call(self, self.set_param_cli, req, timeout=10)
        if not resp.result.successful:
            raise RuntimeError(
                f"set {name} rejected: {resp.result.reason}")
        # LL-096: set alone is not proof — read it back.
        for _ in range(20):
            got = call(
                self, self.get_param_cli,
                GetParameters.Request(names=[name]), timeout=5)
            if got.values and got.values[0].string_value == value:
                return
            spin(self, 0.1)
        raise RuntimeError(f"param {name} readback mismatch")


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    env = make_env()
    rclpy.init()
    node = HarnessNode()
    ensure_vcan(VCAN)
    clear_injection_files()

    urdf = os.path.join(
        WS, "install/a3_description/share/a3_description/urdf/el_a3.urdf")
    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", VCAN, "--gravity-model", "urdf",
         "--gravity-scales", "1.0", "1.0", "1.0", "1.0", "1.0", "1.0",
         "--gravity-noise", "0.01", "--gravity-urdf", urdf],
        SIM_LOG, env=env)
    time.sleep(1.5)
    stack = ProcessGroup(
        [
            "ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
            "hardware:=can", f"can_interface:={VCAN}",
            "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false",
        ],
        STACK_LOG, env=env)

    try:
        # Wait for the full stack + FSM services.
        for cli in (node.teach_start_cli, node.teach_stop_cli,
                    node.set_param_cli, node.get_param_cli,
                    node.list_cli):
            assert cli.wait_for_service(timeout_sec=40), \
                f"service absent: {cli.srv_name}"

        print("\n==== 1. rejections before enable ====", flush=True)
        r = call(node, node.teach_start_cli, Trigger.Request())
        check("start_teach rejected while DISABLED",
              not r.success and "READY" in r.message, r.message)
        r = call(node, node.teach_stop_cli, Trigger.Request())
        check("stop_teach rejected while not TEACH",
              not r.success and "not teaching" in r.message, r.message)

        print("\n==== 2. enable -> READY ====", flush=True)
        node.js_stamp = None
        t0 = time.monotonic()
        while node.js_stamp is None and time.monotonic() - t0 < 40:
            spin(node, 0.1)
        if node.js_stamp is None:
            raise RuntimeError("no /joint_states in 40 s")
        spin(node, 1.0)
        node.enable_to_ready()
        states = node.controller_states()
        check("arm_controller active at READY",
              states.get("arm_controller") == "active", str(states))
        check("zero_torque_controller inactive at READY",
              states.get("zero_torque_controller") == "inactive",
              str(states))

        print("\n==== 3. start_teach -> free drive ====", flush=True)
        r = call(node, node.teach_start_cli, Trigger.Request())
        check("start_teach success", r.success, r.message)
        if not node.wait_state("TEACH", timeout=10):
            raise RuntimeError("state did not reach TEACH")
        spin(node, 0.5)
        states = node.controller_states()
        check("zero_torque_controller active in TEACH",
              states.get("zero_torque_controller") == "active", str(states))
        check("arm_controller inactive in TEACH",
              states.get("arm_controller") == "inactive", str(states))

        t0 = time.monotonic()
        while node.grav_stamp is None and time.monotonic() - t0 < 3.0:
            spin(node, 0.05)
        if node.grav_stamp is None:
            raise RuntimeError("no gravity_torque after start_teach")
        spin(node, 0.5)
        age = time.time() - node.grav_stamp
        check("~/gravity_torque keeps publishing (stamp fresh)",
              age < 0.3, f"age={age:.2f}s")

        # Mode echo is what the BLOCKED_MODES gates read.
        check("/a3/control_mode == ZERO_TORQUE",
              node.mode == "ZERO_TORQUE", str(node.mode))

        # F89/F89b steady-state rule: settle 1 s (mode-switch transient),
        # then judge drift over 1 s.
        spin(node, 1.0)
        p_start = {n: node.js.position[node.js.name.index(n)]
                   for n in ARM_JOINTS}
        spin(node, 1.0)
        p_end = {n: node.js.position[node.js.name.index(n)]
                 for n in ARM_JOINTS}
        per_joint = {n: abs(p_end[n] - p_start[n]) for n in ARM_JOINTS}
        worst_name = max(per_joint, key=per_joint.get)
        drift = per_joint[worst_name]
        check("steady free-drive drift <= 0.02 rad over 1 s",
              drift <= 0.02, f"worst={worst_name} drift={drift:.4f}")

        print("\n==== 4. duplicate start rejected ====", flush=True)
        r = call(node, node.teach_start_cli, Trigger.Request())
        check("start_teach rejected while TEACH",
              not r.success and "READY" in r.message, r.message)

        print("\n==== 5. exit-switch fail-safe ====", flush=True)
        node.set_fsm_string_param("freedrive_arm_controller", BOGUS_ARM)
        r = call(node, node.teach_stop_cli, Trigger.Request())
        check("stop_teach success=false when STRICT exit rejected",
              not r.success and "free-drive exit failed" in r.message,
              r.message)
        spin(node, 0.5)
        check("state stays TEACH after failed exit",
              node.status is not None and node.status.state == "TEACH",
              node.status.state if node.status else "no status")
        states = node.controller_states()
        check("zero_torque still active after failed exit",
              states.get("zero_torque_controller") == "active", str(states))
        check("arm still inactive after failed exit",
              states.get("arm_controller") == "inactive", str(states))
        node.set_fsm_string_param("freedrive_arm_controller",
                                  "arm_controller")

        print("\n==== 6. stop_teach -> READY ====", flush=True)
        node.mode_stamp = None
        r = call(node, node.teach_stop_cli, Trigger.Request())
        check("stop_teach success", r.success, r.message)
        check("message reports auto-save",
              "auto-saved latest.yaml" in r.message, r.message)
        if not node.wait_state("READY", timeout=10):
            raise RuntimeError("state did not return to READY")
        states = node.controller_states()
        check("arm_controller active after stop",
              states.get("arm_controller") == "active", str(states))
        check("zero_torque_controller inactive after stop",
              states.get("zero_torque_controller") == "inactive",
              str(states))
        # /a3/control_mode is latched-on-change; wait for the READY echo.
        t0 = time.monotonic()
        while node.mode != "READY" and time.monotonic() - t0 < 5.0:
            spin(node, 0.05)
        check("/a3/control_mode back to READY", node.mode == "READY",
              str(node.mode))

    except Exception:
        import traceback
        traceback.print_exc()
        with open(STACK_LOG, errors="replace") as f:
            txt = f.read()
        print("\n---- tail stack log ----\n", txt[-3000:], flush=True)
    finally:
        if stack.proc.poll() is None:
            stack.hard_kill()
        sim.terminate()
        node.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F89b acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
