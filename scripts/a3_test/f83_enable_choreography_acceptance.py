#!/usr/bin/env python3
"""F83 acceptance: vendor-standard enable choreography + soft-start damping.

Self-contained: launches vcan_motor_sim and the full CAN stack
(a3_bringup.launch.py hardware:=can can_interface:=vcan0), sniffs raw CAN
frames, runs the checks, tears everything down.

  python3 scripts/a3_test/f83_enable_choreography_acceptance.py [DOMAIN_ID]

Acceptance (docs/edge/REQUIREMENTS.md F83):
  0. zero frames on the CAN bus between controller_manager boot and the
     operator enable (hardware boots INACTIVE, LL-086)
  1. per motor 1-7, in order with >=20 ms gaps: Type 4 data[0]=1 (clear
     fault) -> Type 18 param 0x7005 data[4]=0 (RUN_MODE=MOTION_CONTROL)
     -> Type 3 (enable); the F81 reset gate precedes the choreography
  2. after each motor's Type 3, the first >=3 Type 1 frames are pure
     damping (kp<1, kd 3.5-4.5), then within 0.5 s normal gains
     (kp 70-90, kd ~2)
  3. post-soft-start motion: small quintic FJT error_code=0 on both JTCs,
     position span visible on the bus; disable is clean (FSM DISABLED)
  4. dark motor at enable: whole choreography is skipped (no Type 3/18/
     clear-fault frames), on_activate returns ERROR (F81 semantics kept)

Exit code 0 = all checks passed.
"""

import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import rclpy
import rclpy.action
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "86"
IFACE = os.environ.get("F83_CAN_IF", "vcan0")
IFACE_B = os.environ.get("F83_CAN_IF_B", "vcan2")
STACK_LOG = "/tmp/f83_stack.log"
SIM_LOG = "/tmp/f83_sim.log"
LOG_B = "/tmp/f83_phaseB.log"
JOINTS = [f"L{i}_joint" for i in range(1, 8)]

CAN_FORMAT = "=IB3x8s"
CMD_CONTROL = 0x01
CMD_ENABLE = 0x03
CMD_RESET = 0x04
CMD_SETPARAM = 0x12
KP_MAX = 500.0
KD_MAX = 5.0
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def u16f(raw, hi):
    return raw / 65535.0 * hi


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


class CanSniffer(threading.Thread):
    def __init__(self, interface):
        super().__init__(daemon=True)
        self.interface = interface
        self.stop_ev = threading.Event()
        self.lock = threading.Lock()
        self.frames = []   # (t, cmd_type, motor_id, data bytes)

    def run(self):
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((self.interface,))
        sock.settimeout(0.2)
        while not self.stop_ev.is_set():
            try:
                raw = sock.recv(72)
            except socket.timeout:
                continue
            except OSError:
                break
            can_id, _dlc, data = struct.unpack(CAN_FORMAT, raw)
            can_id &= 0x1FFFFFFF
            cmd_type = (can_id >> 24) & 0x1F
            motor_id = can_id & 0xFF
            if not (1 <= motor_id <= 7):
                continue
            with self.lock:
                self.frames.append(
                    (time.monotonic(), cmd_type, motor_id, bytes(data)))
        sock.close()

    def since(self, t0):
        with self.lock:
            return [f for f in self.frames if f[0] >= t0]

    def clear(self):
        with self.lock:
            self.frames.clear()

    def stop(self):
        self.stop_ev.set()


def make_env(domain):
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["PYTHONNOUSERSITE"] = "1"
    return env


def ensure_vcan(interface):
    base = interface.replace("vcan", "")
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


def call(node, cli, request, timeout=8.0):
    fut = cli.call_async(request)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    raise RuntimeError(f"service timeout: {cli.srv_name}")


def wait_jsb(node, list_cli, deadline_s=90):
    t0 = time.monotonic()
    while time.monotonic() - t0 < deadline_s:
        spin(node, 0.5)
        states = {c.name: c.state for c in call(
            node, list_cli, ListControllers.Request(), timeout=5.0).controller}
        if states.get("joint_state_broadcaster") == "active":
            return states
    return None


def quintic_goal(joint_names, delta, duration_s=3.0):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(joint_names)
    n = 31
    for k in range(n + 1):
        s = k / n
        s = s * s * s * (10 * s * s - 15 * s + 6)
        pt = JointTrajectoryPoint()
        pt.positions = [s * d for d in delta]
        pt.time_from_start.sec = int(duration_s * k / n)
        pt.time_from_start.nanosec = int(
            (duration_s * k / n % 1.0) * 1e9)
        goal.trajectory.points.append(pt)
    return goal


def move_small(node):
    """Relative quintic move: L1-L6 +0.10 rad, L7 +0.05 rad."""
    ac_arm = rclpy.action.ActionClient(
        node, FollowJointTrajectory,
        "/arm_controller/follow_joint_trajectory")
    ac_grip = rclpy.action.ActionClient(
        node, FollowJointTrajectory,
        "/gripper_controller/follow_joint_trajectory")
    if (not ac_arm.wait_for_server(timeout_sec=8)
            or not ac_grip.wait_for_server(timeout_sec=8)):
        return None, None
    gh1 = ac_arm.send_goal_async(
        quintic_goal(JOINTS[:6], [0.10] * 6))
    gh2 = ac_grip.send_goal_async(
        quintic_goal(JOINTS[6:], [0.05]))
    end = time.monotonic() + 10
    while (not gh1.done() or not gh2.done()) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not gh1.done() or not gh2.done():
        return None, None
    if not (gh1.result().accepted and gh2.result().accepted):
        return None, None
    r1 = gh1.result().get_result_async()
    r2 = gh2.result().get_result_async()
    end = time.monotonic() + 12
    while (not r1.done() or not r2.done()) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not r1.done() or not r2.done():
        return "pending", "pending"
    return r1.result().result.error_code, r2.result().result.error_code


def decode_gains(data):
    kp = u16f((data[4] << 8) | data[5], KP_MAX)
    kd = u16f((data[6] << 8) | data[7], KD_MAX)
    return kp, kd


MINIMAL_LAUNCH = "/tmp/f83_minimal.launch.py"

MINIMAL_LAUNCH_BODY = '''"""F83 phase-B minimal stack.

Humble switch_controller does not activate INACTIVE-boot hardware (LL-086),
so after spawning arm_controller active we explicitly drive the component
to ACTIVE via set_hardware_component_state; used to verify a dark motor
blocks on_activate. Interface selected by F83_CAN_IF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    iface = os.environ.get("F83_CAN_IF", "vcan2")
    desc = get_package_share_directory("a3_description")
    xacro = os.path.join(desc, "urdf", "el_a3.urdf.xacro")
    ctrls = os.path.join(desc, "config", "el_a3_controllers.yaml")

    robot_description = Command([
        "xacro ", xacro,
        " use_real_hardware:=true can_interface:=", iface])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": ParameterValue(robot_description, value_type=str)}, ctrls],
        output="screen",
    )
    spawn_arm = TimerAction(period=3.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager",
                   "/controller_manager"],
    )])
    arm_hw = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/controller_manager/set_hardware_component_state",
            "controller_manager_msgs/srv/SetHardwareComponentState",
            "{name: RsA3System, target_state: {id: 3}}",
        ],
        output="screen",
    )])
    return LaunchDescription([control_node, spawn_arm, arm_hw])
'''


def write_minimal_launch():
    with open(MINIMAL_LAUNCH, "w") as f:
        f.write(MINIMAL_LAUNCH_BODY)


def main():
    # The harness node must share the stack's domain; rclpy reads
    # ROS_DOMAIN_ID at init, so force it before rclpy.init().
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    ensure_vcan(IFACE)
    env = make_env(DOMAIN)
    # Sniffer starts BEFORE the stack: check 0 is total CAN silence between
    # controller_manager boot and the operator enable call.
    sniffer = CanSniffer(IFACE)
    sniffer.start()
    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", IFACE], SIM_LOG, env=env)
    time.sleep(0.8)
    t_boot = time.monotonic()
    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={IFACE}",
         "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"],
        STACK_LOG, env=env)

    rclpy.init()
    node = Node("f83_acceptance")
    list_cli = node.create_client(
        ListControllers, "/controller_manager/list_controllers")
    enable_cli = node.create_client(Trigger, "/a3/arm/enable")
    disable_cli = node.create_client(Trigger, "/a3/arm/disable")
    fatal = False
    try:
        if not list_cli.wait_for_service(timeout_sec=30):
            raise RuntimeError("controller_manager absent in 30s")
        if wait_jsb(node, list_cli) is None:
            raise RuntimeError("JSB not active in 90s")

        # ---- check 0 + checks 1+2: enable -> choreography + soft-start ----
        # Hardware boots INACTIVE (hardware_components_initial_state, LL-086):
        # the F81 reset gate and vendor choreography all run inside
        # on_activate, triggered by THIS operator enable call.
        t_enable = time.monotonic()
        # Worst-case FSM path ~21 s (wait_service + set_hw 10 s + switch 5 s);
        # the component on_activate alone blocks CM ~1.2 s.
        assert enable_cli.wait_for_service(timeout_sec=20), "enable service absent"
        call(node, enable_cli, Trigger.Request(), timeout=30)
        pre = [f for f in sniffer.since(t_boot) if f[0] < t_enable]
        check("0 zero frames on bus: CM boot -> operator enable",
              len(pre) == 0, f"frames={len(pre)}")

        # Wait until controllers active (choreography + soft-start done by then).
        t0 = time.monotonic()
        st = {}
        while time.monotonic() - t0 < 20:
            spin(node, 0.1)
            st = {c.name: c.state for c in call(
                node, list_cli, ListControllers.Request(),
                timeout=5.0).controller}
            if (st.get("arm_controller") == "active"
                    and st.get("gripper_controller") == "active"):
                break
        check("controllers activated", st.get("arm_controller") == "active"
              and st.get("gripper_controller") == "active", f"states={st}")
        spin(node, 0.6)  # collect normal-gain frames past soft-start
        frames = sniffer.since(0)

        # check 1: per-motor ordered choreography with >=20 ms gaps
        choreo_ok = True
        choreo_detail = []
        for motor in range(1, 8):
            seq = [(t, ct, d) for t, ct, m, d in frames
                   if m == motor and ct in (CMD_RESET, CMD_SETPARAM, CMD_ENABLE)]
            cf = next(((t, d) for t, ct, d in seq
                       if ct == CMD_RESET and d[0] == 0x01), None)
            sp = next(((t, d) for t, ct, d in seq if ct == CMD_SETPARAM), None)
            en = next(((t, d) for t, ct, d in seq if ct == CMD_ENABLE), None)
            ok = (cf is not None and sp is not None and en is not None
                  and cf[0] < sp[0] < en[0]
                  and sp[1][0] == 0x05 and sp[1][1] == 0x70
                  and sp[1][4] == 0
                  and sp[0] - cf[0] >= 0.02 and en[0] - sp[0] >= 0.02)
            if not ok:
                choreo_ok = False
                choreo_detail.append(
                    f"m{motor}: cf={cf is not None} sp={sp is not None} "
                    f"en={en is not None}")
        # F81 gate (plain reset, data[1]=0xC0) must precede first clear-fault
        first_gate = next(((t, d) for t, ct, m, d in frames
                           if ct == CMD_RESET and d[1] == 0xC0), None)
        first_cf = next((t for t, ct, m, d in frames
                         if ct == CMD_RESET and d[0] == 0x01), None)
        gate_ok = first_gate is not None and first_cf is not None \
            and first_gate[0] < first_cf
        check("1 per-motor clear-fault -> RUN_MODE int write -> enable, gaps >=20ms",
              choreo_ok, "; ".join(choreo_detail))
        check("1b F81 reset gate precedes choreography", gate_ok,
              f"gate={None if first_gate is None else round(first_gate[0]-t_enable,2)} "
              f"cf={None if first_cf is None else round(first_cf-t_enable,2)}")

        # check 2: damping frames then normal gains per motor
        damping_ok = True
        transition_ok = True
        detail = []
        for motor in range(1, 8):
            en_t = next((t for t, ct, m, d in frames
                         if m == motor and ct == CMD_ENABLE), None)
            if en_t is None:
                damping_ok = False
                transition_ok = False
                detail.append(f"m{motor} no enable frame")
                continue
            ctrls = [(t, d) for t, ct, m, d in frames
                     if m == motor and ct == CMD_CONTROL and t >= en_t]
            first3 = ctrls[:3]
            gains = [decode_gains(d) for _t, d in first3]
            if (len(gains) < 3
                    or not all(kp < 1 and 3.5 <= kd <= 4.5
                               for kp, kd in gains)):
                damping_ok = False
                detail.append(f"m{motor} damp={[(round(k,1),round(d,1)) for k,d in gains]}")
                continue
            # write() only starts after the full per-motor choreography
            # finishes, so measure the hand-off from the FIRST Type-1 frame
            # (soft-start 10 cycles @200Hz ≈ 50 ms), not from Type 3.
            first_ctrl_t = ctrls[0][0]
            normal_t = next(
                (t for t, d in ctrls[3:]
                 if 70 <= decode_gains(d)[0] <= 90
                 and abs(decode_gains(d)[1] - 2.0) < 0.3), None)
            if normal_t is None or normal_t - first_ctrl_t > 0.5:
                transition_ok = False
                detail.append(f"m{motor} no normal gains in 0.5s")
        check("2 first >=3 frames pure damping (kp<1, kd 3.5-4.5)",
              damping_ok, "; ".join(detail))
        check("2b transition to normal gains (kp~80, kd~2) within 0.5s",
              transition_ok, "; ".join(detail))

        # ---- check 3: small quintic move + clean disable ----
        t_move = time.monotonic()
        err_arm, err_grip = move_small(node)
        spin(node, 0.3)
        move_frames = sniffer.since(t_move)
        spans = {}
        for motor in range(1, 8):
            vals = [((d[0] << 8) | d[1]) for t, ct, m, d in move_frames
                    if m == motor and ct == CMD_CONTROL]
            if vals:
                spans[motor] = (max(vals) - min(vals)) * 25.14 / 65535
        span_ok = len(spans) == 7 and min(spans.values()) > 0.02
        check("3 post-soft-start quintic FJT error_code=0 + position span",
              err_arm == 0 and err_grip == 0 and span_ok,
              f"err={err_arm}/{err_grip} min_span="
              f"{min(spans.values()) if spans else None}")

        assert disable_cli.wait_for_service(timeout_sec=20), "disable service absent"
        call(node, disable_cli, Trigger.Request(), timeout=30)
        t0 = time.monotonic()
        fsm_state = {}
        from a3_msgs.msg import ArmStatus
        node.create_subscription(
            ArmStatus, "/a3/arm_status",
            lambda m: fsm_state.update(v=m.state), 10)
        while time.monotonic() - t0 < 20:
            spin(node, 0.1)
            st = {c.name: c.state for c in call(
                node, list_cli, ListControllers.Request(),
                timeout=5.0).controller}
            if (fsm_state.get("v") == "DISABLED"
                    and st.get("arm_controller") == "inactive"):
                break
        check("3b clean disable -> DISABLED, controllers inactive",
              fsm_state.get("v") == "DISABLED"
              and st.get("arm_controller") == "inactive",
              f"fsm={fsm_state.get('v')} arm={st.get('arm_controller')}")
    except Exception:
        import traceback
        traceback.print_exc()
        fatal = True
    finally:
        sniffer.stop()
        stack.kill()
        sim.kill()

    # ---- check 4: dark motor at startup skips the whole choreography ----
    if not fatal:
        ensure_vcan(IFACE_B)
        write_minimal_launch()
        silence_b = "/tmp/f83_silence_b.json"
        with open(silence_b, "w") as f:
            f.write('{"motor": 4}')
        domain_b = str(int(DOMAIN) + 3)
        env_b = make_env(domain_b)
        env_b["F83_CAN_IF"] = IFACE_B
        sim2 = ProcessGroup(
            ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
             "--interface", IFACE_B, "--silence-file", silence_b],
            "/tmp/f83_sim_b.log", env=env_b)
        time.sleep(0.8)
        stack2 = ProcessGroup(
            ["ros2", "launch", MINIMAL_LAUNCH], LOG_B, env=env_b)
        sniffer2 = CanSniffer(IFACE_B)
        sniffer2.start()
        try:
            # Minimal launch auto-spawns arm_controller active at t+3 s,
            # which triggers hardware on_activate without a service call.
            t0 = time.monotonic()
            while time.monotonic() - t0 < 25:
                spin(node, 0.5)
                with open(LOG_B, errors="ignore") as lf:
                    text = lf.read()
                if ("no enable frames sent" in text
                        and ("Failed to 'activate' hardware" in text
                             or "process has died" in text
                             or "Could not contact service" in text
                             or "activate aborted" in text)):
                    break
            frames_b = sniffer2.since(0)
            any_enable = any(ct == CMD_ENABLE for _, ct, _m, _d in frames_b)
            any_setparam = any(ct == CMD_SETPARAM for _, ct, _m, _d in frames_b)
            any_clearfault = any(ct == CMD_RESET and d[0] == 0x01
                                 for _, ct, _m, d in frames_b)
            with open(LOG_B, errors="ignore") as lf:
                text = lf.read()
            gate_log = "no enable frames sent" in text
            cm_failed = ("Failed to 'activate' hardware" in text
                         or "process has died" in text
                         or "Could not contact service" in text
                         or "activate aborted" in text)
            check("4 dark motor: choreography skipped, on_activate ERROR",
                  not any_enable and not any_setparam and not any_clearfault
                  and gate_log and cm_failed,
                  f"enable={any_enable} setparam={any_setparam} "
                  f"clearfault={any_clearfault} gate_log={gate_log} "
                  f"cm_failed={cm_failed}")
        finally:
            sniffer2.stop()
            stack2.kill()
            sim2.kill()
            if os.path.exists(silence_b):
                os.remove(silence_b)

    node.destroy_node()
    rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F83 acceptance: {passed}/{total} ====", flush=True)
    if fatal:
        return 2
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
