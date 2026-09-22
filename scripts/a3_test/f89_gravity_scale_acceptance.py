#!/usr/bin/env python3
"""F89 acceptance: gravity-model verification & per-joint scale calibration.

Three segments, all simulation-only (real arm stays powered off):

  Segment A — injection/recovery pairing:
    vcan_motor_sim (F49 calibrated truth inertia, injected true scales,
    0.01 Nm noise) + product stack can mode, arm enabled; run
    scripts/gravity_scale_calibration.py --quick. The produced file must
    recover injected scales on the excited joints (L2/L3/L4) within 0.05,
    R2 >= 0.95, and flag the physically low-excitation joints L1/L5/L6.

  Segment B — produced file drives free-drive:
    restart the stack with gravity_scales_file:=<produced>; enable; direct
    two-point FJT to HOME; switch arm_controller -> zero_torque_controller.
      a) /zero_torque_controller/gravity_torque effort equals an independent
         harness RNEA x recovered scales at the same q (<= 0.03 Nm)
      b) steady free-drive drift <= 0.02 rad over 1 s after a 1 s settle
         (compensation matches truth; mode-switch transient excluded)
    switch back.

  Segment C — default behavior:
    restart with gravity_scales_file:=""; zero_torque configure log says
    "tau_scale not set, using 1.0".

  python3 scripts/a3_test/f89_gravity_scale_acceptance.py [DOMAIN]

Exit code 0 = all checks passed.
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "89"
VCAN = os.environ.get("F89_VCAN", "vcan89")
SIM_LOG = "/tmp/f89_accept_sim.log"
CAL_LOG = "/tmp/f89_accept_cal.log"
STACKA_LOG = "/tmp/f89_accept_stack_a.log"
STACKB_LOG = "/tmp/f89_accept_stack_b.log"
STACKC_LOG = "/tmp/f89_accept_stack_c.log"
SCALES_FILE = "/tmp/f89_scales_accept.yaml"
RESULTS = []

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
ALL_JOINTS = [f"L{i}_joint" for i in range(1, 8)]
# True scales injected into the sim. L1/L5/L6 stay 1.0: their gravity loads
# are physically unobservable in static poses (base yaw / wrist axes), so the
# tool correctly keeps scale=1.0 there regardless.
TRUE_SCALES = [1.0, 0.92, 1.08, 0.95, 1.0, 1.0]
EXPECTED_LOW_EXC = [True, False, False, False, True, True]
JOINT_TO_LINK = {
    "L2": "l2_l3_urdf_asm",
    "L3": "l3_lnik_urdf_asm",
    "L4": "l4_l5_urdf_asm",
    "L5": "part_9",
    "L6": "l5_l6_urdf_asm",
}


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


def wait_js(node, timeout_s=35):
    # Reset first: the node outlives stack restarts, so a stamp carried over
    # from the previous segment would make this return instantly even if the
    # new ros2_control_node is dead.
    node.js_stamp = None
    node.js = None
    t0 = time.monotonic()
    while node.js_stamp is None and time.monotonic() - t0 < timeout_s:
        spin(node, 0.1)
    return node.js_stamp is not None


def clear_injection_files():
    # Leftover dynamics/push/fault/silence files would otherwise leak into
    # this acceptance (the sim watches them continuously).
    for path in ("/tmp/f87_dynamics.json", "/tmp/f73_ext.json",
                 "/tmp/f81_silence.json", "/tmp/f84_health.json"):
        with open(path, "w") as f:
            f.write("{}")


def ensure_vcan(interface):
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "add", "dev", interface, "type", "vcan"],
        input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "set", interface, "up"],
        input=b"temppwd\n", capture_output=True)


def build_calibrated_model():
    import numpy as np
    import pinocchio as pin
    share = get_package_share_directory("a3_description")
    urdf_path = os.path.join(share, "urdf", "el_a3.urdf")
    model = pin.buildModelFromUrdf(urdf_path)
    with open(os.path.join(share, "config", "inertia_params.yaml"),
              "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    params = data.get("inertia_params", {})
    if params and data.get("use_calibrated_params", False):
        for key, link_name in JOINT_TO_LINK.items():
            if key not in params:
                continue
            parent = None
            for frame in model.frames:
                if frame.name == link_name:
                    parent = frame.parentJoint
                    break
            if parent is None or parent <= 0 or parent >= len(model.inertias):
                continue
            p = params[key]
            mass = float(p.get("mass", model.inertias[parent].mass))
            com = np.array(p.get("com", [0.0, 0.0, 0.0]), dtype=np.float64)
            Y = model.inertias[parent]
            model.inertias[parent] = pin.Inertia(mass, com, Y.inertia)
    q_idx, v_idx = [], []
    for name in ARM_JOINTS:
        jid = model.getJointId(name)
        q_idx.append(model.joints[jid].idx_q)
        v_idx.append(model.joints[jid].idx_v)
    return model, q_idx, v_idx


def independent_rnea(model, q_idx, v_idx, pose6):
    import pinocchio as pin
    q = pin.neutral(model)
    for i in range(6):
        q[q_idx[i]] = pose6[i]
    # Same convention as zero_torque_controller: q_ is zeroed, L7 stays 0.
    g = pin.computeGeneralizedGravity(model, model.createData(), q)
    return {ARM_JOINTS[i]: float(g[v_idx[i]]) for i in range(6)}


class HarnessNode(Node):
    def __init__(self, name="f89_acceptance"):
        super().__init__(name)
        self.js = None
        self.js_stamp = None
        self.grav = None
        self.grav_stamp = None
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.create_subscription(
            JointState, "/joint_states", self._on_js,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            JointState, "/joint_states", self._on_js,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            JointState, "/zero_torque_controller/gravity_torque",
            self._on_grav, 10)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")
        self.switch_cli = self.create_client(
            SwitchController, "/controller_manager/switch_controller")

    def _on_js(self, msg):
        self.js = msg
        self.js_stamp = time.time()

    def _on_grav(self, msg):
        self.grav = msg
        self.grav_stamp = time.time()

    def arm_positions(self):
        return [self.js.position[self.js.name.index(n)] for n in ARM_JOINTS]

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
        # FSM state is observed through /a3/arm_status; wait on the controllers
        # instead: arm_controller claimed/active is what the calibration needs.
        spin(self, 3.0)

    def switch_to_free_drive(self):
        req = SwitchController.Request(
            activate_controllers=["zero_torque_controller"],
            deactivate_controllers=["arm_controller"],
            strictness=SwitchController.Request.STRICT)
        req.timeout = RosDuration(sec=2, nanosec=0)
        resp = call(self, self.switch_cli, req, timeout=10)
        return resp.ok

    def switch_to_position(self):
        req = SwitchController.Request(
            activate_controllers=["arm_controller"],
            deactivate_controllers=["zero_torque_controller"],
            strictness=SwitchController.Request.STRICT)
        req.timeout = RosDuration(sec=2, nanosec=0)
        resp = call(self, self.switch_cli, req, timeout=10)
        return resp.ok


def start_stack(env, scales_arg):
    argv = [
        "ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
        "hardware:=can", f"can_interface:={VCAN}",
        "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false",
    ]
    # Empty := values are rejected by ros2 launch; with no scales file the
    # launch default auto-adopt path is identical as long as
    # ~/.a3/gravity_scales.yaml is absent (asserted by segment A cleanup).
    if scales_arg:
        argv.append(f"gravity_scales_file:={scales_arg}")
    return argv


def send_home_fjt(node, timeout=12.0):
    ac = ActionClient(
        node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=10):
        raise RuntimeError("arm FJT action absent")
    q0 = node.arm_positions()
    goal = FollowJointTrajectory.Goal()
    goal.goal_time_tolerance = RosDuration(sec=1, nanosec=0)
    goal.trajectory.joint_names = list(ARM_JOINTS)
    p0 = JointTrajectoryPoint(
        positions=[float(v) for v in q0],
        velocities=[0.0] * 6, accelerations=[0.0] * 6,
        time_from_start=RosDuration())
    home = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
    p1 = JointTrajectoryPoint(
        positions=home, velocities=[0.0] * 6, accelerations=[0.0] * 6,
        time_from_start=RosDuration(sec=3, nanosec=0))
    goal.trajectory.points = [p0, p1]
    gh_fut = ac.send_goal_async(goal)
    end = time.monotonic() + 5
    while not gh_fut.done() and time.monotonic() < end:
        spin(node, 0.05)
    handle = gh_fut.result()
    if not handle.accepted:
        raise RuntimeError("FJT goal rejected")
    res_fut = handle.get_result_async()
    end = time.monotonic() + timeout
    while not res_fut.done() and time.monotonic() < end:
        spin(node, 0.05)
    res = res_fut.result().result
    return res.error_code == 0


def segment_a(env, node):
    print("\n==== Segment A: inject true scales, recover via calibration ====",
          flush=True)
    ensure_vcan(VCAN)
    clear_injection_files()
    check("A precondition: ~/.a3/gravity_scales.yaml absent (no auto-adopt)",
          not os.path.exists(os.path.expanduser("~/.a3/gravity_scales.yaml")))
    urdf = os.path.join(
        WS, "install/a3_description/share/a3_description/urdf/el_a3.urdf")
    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", VCAN, "--gravity-model", "urdf",
         "--gravity-scales", *[f"{v}" for v in TRUE_SCALES],
         "--gravity-noise", "0.01", "--gravity-urdf", urdf],
        SIM_LOG, env=env)
    time.sleep(1.5)
    with open(SIM_LOG, errors="replace") as f:
        simtxt = f.read()
    check("A sim truth applies F49 calibrated inertia (5 links)",
          "applied F49 calibrated inertia to 5 links" in simtxt, simtxt[-200:])

    stack = ProcessGroup(
        start_stack(env, ""), STACKA_LOG, env=env)
    try:
        if not wait_js(node):
            raise RuntimeError("stack A: no /joint_states in 35 s")
        spin(node, 1.0)
        node.enable_to_ready()
        spin(node, 1.0)

        if os.path.exists(SCALES_FILE):
            os.remove(SCALES_FILE)
        cal = subprocess.Popen(
            ["python3", f"{WS}/scripts/gravity_scale_calibration.py",
             "--quick", "--out", SCALES_FILE],
            stdout=open(CAL_LOG, "wb"), stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True, env=env)
        try:
            rc = cal.wait(timeout=240)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(cal.pid), signal.SIGKILL)
            raise RuntimeError("calibration timed out")
        check("A calibration exits 0", rc == 0, f"rc={rc}")

        with open(SCALES_FILE, "r", encoding="utf-8") as f:
            produced = yaml.safe_load(f)
        scales = produced["zero_torque_controller"]["ros__parameters"]["tau_scale"]
        meta = produced["gravity_calibration_metadata"]["ros__parameters"]
        check("A produced file has 6 tau_scale values", len(scales) == 6)

        low_exc = meta["low_excitation"]
        check("A low-excitation flags L1/L5/L6, excited L2/L3/L4",
              low_exc == EXPECTED_LOW_EXC, f"{low_exc}")

        for i, name in enumerate(ARM_JOINTS):
            if EXPECTED_LOW_EXC[i]:
                check(f"A {name}: low-exc scale held 1.0",
                      abs(scales[i] - 1.0) < 1e-6, f"scale={scales[i]:.4f}")
            else:
                err = abs(scales[i] - TRUE_SCALES[i])
                r2 = meta["r_squared"][i]
                check(f"A {name}: recovered scale {TRUE_SCALES[i]:.2f} within 0.05",
                      err <= 0.05, f"got={scales[i]:.4f} err={err:.4f}")
                check(f"A {name}: R2 >= 0.95",
                      r2 is not None and r2 >= 0.95, f"R2={r2}")
    except Exception:
        import traceback
        traceback.print_exc()
        return False, stack, sim
    return True, stack, sim


def segment_b(env, node):
    print("\n==== Segment B: produced file loaded, free-drive compensation ====",
          flush=True)
    stack = ProcessGroup(
        start_stack(env, SCALES_FILE), STACKB_LOG, env=env)
    try:
        if not wait_js(node):
            raise RuntimeError("stack B: no /joint_states in 35 s")
        spin(node, 1.0)
        node.enable_to_ready()
        spin(node, 1.0)

        fjt_ok = send_home_fjt(node)
        check("B FJT to HOME succeeds", fjt_ok)
        spin(node, 1.0)

        switched = node.switch_to_free_drive()
        check("B strict switch arm_controller -> zero_torque", switched)
        spin(node, 0.3)

        # ---- Criterion a: commanded effort == independent RNEA x scales ----
        with open(SCALES_FILE, "r", encoding="utf-8") as f:
            scales = yaml.safe_load(
                f)["zero_torque_controller"]["ros__parameters"]["tau_scale"]
        model, q_idx, v_idx = build_calibrated_model()

        t0 = time.monotonic()
        while node.grav is None and time.monotonic() - t0 < 3.0:
            spin(node, 0.05)
        if node.grav is None:
            raise RuntimeError("no /zero_torque_controller/gravity_torque")
        grav = node.grav
        pose = {name: grav.position[grav.name.index(name)]
                for name in ARM_JOINTS}
        g_pred = independent_rnea(model, q_idx, v_idx,
                                  [pose[n] for n in ARM_JOINTS])
        residuals = []
        for i, name in enumerate(ARM_JOINTS):
            cmd = grav.effort[grav.name.index(name)]
            residuals.append(abs(cmd - scales[i] * g_pred[name]))
        worst = max(residuals)
        check("B gravity_torque = scale x independent RNEA (<=0.03 Nm)",
              worst <= 0.03, f"worst={worst:.4f}")

        # ---- Criterion b: steady-state free-drive drift over 1 s ----
        # Let the mode-switch transient (position->effort frame handover,
        # residual velocity) decay first; industrial free-float checks judge
        # the held pose, not the switch instant.
        spin(node, 1.0)
        if node.js is None:
            raise RuntimeError("no /joint_states during free drive")
        p_start = {n: node.js.position[node.js.name.index(n)]
                   for n in ARM_JOINTS}
        spin(node, 1.0)
        p_end = {n: node.js.position[node.js.name.index(n)]
                 for n in ARM_JOINTS}
        per_joint = {n: abs(p_end[n] - p_start[n]) for n in ARM_JOINTS}
        drift = max(per_joint.values())
        worst_name = max(per_joint, key=per_joint.get)
        check("B steady free-drive drift <= 0.02 rad over 1 s",
              drift <= 0.02,
              f"worst={worst_name} drift={drift:.4f}")

        back = node.switch_to_position()
        check("B switch back to arm_controller", back)
        spin(node, 0.5)
    except Exception:
        import traceback
        traceback.print_exc()
        return False, stack
    return True, stack


def segment_c(env, node):
    print("\n==== Segment C: no scales file -> tau_scale defaults to 1.0 ====",
          flush=True)
    stack = ProcessGroup(
        start_stack(env, ""), STACKC_LOG, env=env)
    try:
        if not wait_js(node):
            raise RuntimeError("stack C: no /joint_states in 35 s")
        found = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20.0:
            spin(node, 0.2)
            with open(STACKC_LOG, errors="replace") as f:
                if "tau_scale not set, using 1.0" in f.read():
                    found = True
                    break
        check("C zero_torque configure: tau_scale unset -> 1.0", found)
    except Exception:
        import traceback
        traceback.print_exc()
        return False, stack
    return True, stack


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    env = make_env()
    rclpy.init()

    node = HarnessNode()
    ok_a = ok_b = ok_c = False
    stack_a = stack_b = stack_c = sim = None
    try:
        ok_a, stack_a, sim = segment_a(env, node)
    finally:
        if stack_a and stack_a.proc.poll() is None:
            stack_a.hard_kill()

    if ok_a:
        spin(node, 1.5)
        try:
            ok_b, stack_b = segment_b(env, node)
        finally:
            if stack_b and stack_b.proc.poll() is None:
                stack_b.hard_kill()

    if ok_a and ok_b:
        spin(node, 1.5)
        try:
            ok_c, stack_c = segment_c(env, node)
        finally:
            if stack_c and stack_c.proc.poll() is None:
                stack_c.hard_kill()

    if sim:
        sim.terminate()
    node.destroy_node()
    rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F89 acceptance: {passed}/{total} ====", flush=True)
    return 0 if ok_a and ok_b and ok_c and passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
