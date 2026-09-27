#!/usr/bin/env python3
"""F127 acceptance: teach free-drive releases the gripper joint L7.

F87 put L7 on the standard GripperActionController (effort, claim
[L7_joint/effort]); F73's zero_torque_controller claimed L1-L6 only. F127
extends zero_torque to also claim L7, so during teach free-drive the gripper
is torque-free alongside the arm. arm_controller + gripper_controller both
activating at enable means Humble STRICT switch_controller will REFUSE "arm
out + zero_torque in" while gripper still claims L7 — so _cm_freedrive_switch
must atomically swap gripper_controller out (enter) and back (exit) within the
same request (LL-136).

All simulation-only (real arm stays powered off): vcan_motor_sim + product
stack a3_bringup hardware:=can (same skeleton as F89b, domain 92 /
vcan89b).

Sequence:
  1. enable -> READY; list_controllers: arm active, zero_torque inactive,
     gripper active (L7 position-controlled at rest).
  2. start_teach -> TEACH; zero_torque active, arm inactive, gripper inactive
     (released); get_parameters on zero_torque_controller confirms joints
     include L7_joint; settle 1 s then 1 s drift <= 0.02 rad across all 7
     joints (L7 held by gravity comp too).
  3. stop_teach -> READY; arm active, gripper active (restored), zero_torque
     inactive; message reports auto-saved latest.yaml.

python3 scripts/a3_test/f127_l7_teach_release_acceptance.py [DOMAIN]

Exit code 0 = all checks passed.
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from a3_msgs.msg import ArmStatus

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "92"
VCAN = os.environ.get("F127_VCAN", "vcan89b")
SIM_LOG = "/tmp/f127_accept_sim.log"
STACK_LOG = "/tmp/f127_accept_stack.log"
RESULTS = []

ARM = "arm_controller"
GRIPPER = "gripper_controller"
ZERO = "zero_torque_controller"
ALL_JOINTS = [f"L{i}_joint" for i in range(1, 8)]


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
        super().__init__("f127_acceptance")
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
        # zero_torque_controller 的 joints 参数（F127 须含 L7_joint）
        self.zt_get_param_cli = self.create_client(
            GetParameters, "/zero_torque_controller/get_parameters")

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

    def joint_positions(self):
        if self.js is None:
            return None
        idx = self.js.name
        return [self.js.position[idx.index(n)] for n in ALL_JOINTS]

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

    def dump(self, label=""):
        p = self.joint_positions()
        s = " ".join(f"{n}={v:.3f}" for n, v in zip(ALL_JOINTS, p)) if p else "no-js"
        print(f"    pos[{label}] {s}", flush=True)

    def controller_states(self):
        resp = call(self, self.list_cli, ListControllers.Request(),
                    timeout=10)
        return {c.name: c.state for c in resp.controller}

    def zero_torque_joints(self):
        """读 zero_torque_controller 运行时 joints 数组（string_array）。"""
        resp = call(self, self.zt_get_param_cli,
                    GetParameters.Request(names=["joints"]), timeout=10)
        if not resp.values:
            return []
        pv = resp.values[0]
        if pv.type != ParameterType.PARAMETER_STRING_ARRAY:
            return []
        return list(pv.string_array_value)


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
        for cli in (node.teach_start_cli, node.teach_stop_cli,
                    node.list_cli, node.zt_get_param_cli):
            assert cli.wait_for_service(timeout_sec=40), \
                f"service absent: {cli.srv_name}"

        print("\n==== 1. enable -> READY（arm + gripper 均 active）====", flush=True)
        node.js_stamp = None
        t0 = time.monotonic()
        while node.js_stamp is None and time.monotonic() - t0 < 40:
            spin(node, 0.1)
        if node.js_stamp is None:
            raise RuntimeError("no /joint_states in 40 s")
        spin(node, 1.0)
        node.dump("pre-enable")
        node.enable_to_ready()
        node.dump("READY")
        states = node.controller_states()
        check(f"arm active at READY ({ARM})",
              states.get(ARM) == "active", str(states))
        check(f"gripper active at READY ({GRIPPER})",
              states.get(GRIPPER) == "active", str(states))
        check(f"zero_torque inactive at READY ({ZERO})",
              states.get(ZERO) == "inactive", str(states))

        print("\n==== 2. start_teach -> L7 随 arm 一并释放 ====", flush=True)
        r = call(node, node.teach_start_cli, Trigger.Request())
        check("start_teach success", r.success, r.message)
        if not node.wait_state("TEACH", timeout=10):
            raise RuntimeError("state did not reach TEACH")
        node.dump("TEACH")
        if os.environ.get("F127_TRACE"):
            t0 = time.monotonic()
            for _ in range(20):
                spin(node, 0.25)
                p = node.joint_positions()
                v = None
                if node.js is not None:
                    idx = node.js.name
                    v = [node.js.velocity[idx.index(n)] for n in ALL_JOINTS]
                ps = " ".join(f"{x:.3f}" for x in p) if p else "na"
                vs = " ".join(f"{x:.3f}" for x in v) if v else "na"
                print(f"    TRACE t={time.monotonic()-t0:5.2f} pos=[{ps}] vel=[{vs}]", flush=True)
        spin(node, 0.5)
        states = node.controller_states()
        check("zero_torque active in TEACH", states.get(ZERO) == "active",
              str(states))
        check("arm inactive in TEACH", states.get(ARM) == "inactive",
              str(states))
        check("gripper RELEASED in TEACH (F127)",
              states.get(GRIPPER) == "inactive", str(states))

        zj = node.zero_torque_joints()
        check("zero_torque_controller.joints 含 L7_joint (F127)",
              "L7_joint" in zj, f"joints={zj}")
        check("zero_torque_controller.joints 含 L1_joint",
              "L1_joint" in zj, f"joints={zj}")

        # f89b: wait until ~/gravity_torque compensator is confirmed live, then
        # check control_mode echo (that's what the BLOCKED_MODES gates read).
        t0 = time.monotonic()
        while node.grav_stamp is None and time.monotonic() - t0 < 3.0:
            spin(node, 0.05)
        if node.grav_stamp is None:
            raise RuntimeError("no gravity_torque after start_teach")
        spin(node, 0.5)
        age = time.time() - node.grav_stamp
        check("~/gravity_torque keeps publishing (stamp fresh)",
              age < 0.3, f"age={age:.2f}s")
        check("/a3/control_mode == ZERO_TORQUE",
              node.mode == "ZERO_TORQUE", str(node.mode))

        # F127 稳态规则（同 F89b）：settle 1 s 跳模式瞬态 → 1 s 判漂移，
        # 但 d 覆盖全部 7 关节（L7 也由重力补偿托住不动）。
        spin(node, 1.0)
        node.dump("settle-end")
        p_start = node.joint_positions()
        if p_start is None:
            raise RuntimeError("no /joint_states during teach")
        spin(node, 1.0)
        p_end = node.joint_positions()
        per_joint = {}
        for i, n in enumerate(ALL_JOINTS):
            a = p_start[i]
            b = p_end[i]
            d = abs(a - b)
            per_joint[n] = d
            check(f"drift {n} <= 0.02 rad over 1 s",
                  d <= 0.02, f"Δ={d:.4f} ({a:.4f}->{b:.4f})")

        print("\n==== 3. stop_teach -> gripper 恢复 active ====", flush=True)
        r = call(node, node.teach_stop_cli, Trigger.Request())
        check("stop_teach success", r.success, r.message)
        check("message reports auto-save",
              "auto-saved latest.yaml" in r.message, r.message)
        if not node.wait_state("READY", timeout=10):
            raise RuntimeError("state did not return to READY")
        states = node.controller_states()
        check("arm active after stop", states.get(ARM) == "active",
              str(states))
        check("gripper RESTORED active after stop (F127)",
              states.get(GRIPPER) == "active", str(states))
        check("zero_torque inactive after stop", states.get(ZERO) == "inactive",
              str(states))

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
    print(f"\n==== F127 L7 release acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())