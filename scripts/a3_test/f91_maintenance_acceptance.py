#!/usr/bin/env python3
"""F91 acceptance: motor maintenance productization (SetZero/SaveParam).

The product SystemInterface plugin cannot host services; F91 adds a
standalone motor_maintenance node (exclusive CAN socket) reusing the
verified codec builders, interlocked against active controllers.

All simulation-only (real arm stays powered off).

Phase A (standalone, stack down):
  1. set_zero single motor -> success, count +1, angle rebased to 0
  2. save_parameters single -> flash count +1
  3. set_zero broadcast 255 -> all 7 counts, every angle 0
  4. save_parameters broadcast -> all 7 counts
  5. motor_id out of range (9) -> success=false, no counts change
Phase B (product stack running):
  6. enable -> READY; set_zero refused (success=false), no counts
  7. save_parameters refused, no counts
  8. stack stopped; set_zero works again, count lands

python3 scripts/a3_test/f91_maintenance_acceptance.py [DOMAIN]

Exit code 0 = all checks passed.
"""

import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from a3_msgs.srv import MotorIdCommand
from a3_msgs.msg import ArmStatus

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "91"
VCAN = os.environ.get("F91_VCAN", "vcan91")
STATE_A = "/tmp/f91_state_a.json"
STATE_B = "/tmp/f91_state_b.json"
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


def call(node, cli, request, timeout=20.0):
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


def read_state(path):
    for _ in range(30):
        try:
            with open(path) as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            time.sleep(0.1)
    raise RuntimeError(f"state file unreadable: {path}")


def wait_counts(path, expected_zeros, expected_saves, timeout=8):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        st = read_state(path)
        motors = st["motors"]
        zeros = {int(k): v["set_zero_count"] for k, v in motors.items()}
        saves = {int(k): v["save_param_count"] for k, v in motors.items()}
        if zeros == expected_zeros and saves == expected_saves:
            return motors
        time.sleep(0.15)
    st = read_state(path)
    return st["motors"]


def start_sim(state_file, tag, gravity=False):
    argv = ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
            "--interface", VCAN, "--state-file", state_file]
    if gravity:
        urdf = os.path.join(
            WS, "install/a3_description/share/a3_description/urdf/el_a3.urdf")
        argv += ["--gravity-model", "urdf",
                 "--gravity-scales", "1.0", "1.0", "1.0", "1.0",
                 "1.0", "1.0",
                 "--gravity-noise", "0.01", "--gravity-urdf", urdf]
    return ProcessGroup(argv, f"/tmp/f91_sim_{tag}.log", env=make_env())


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f91_acceptance")
        self.status = None
        self.create_subscription(
            ArmStatus, "/a3/arm_status", self._on_status, 10)
        self.set_zero_cli = self.create_client(
            MotorIdCommand, "/a3/maintenance/set_zero")
        self.save_cli = self.create_client(
            MotorIdCommand, "/a3/maintenance/save_parameters")
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")

    def _on_status(self, msg):
        self.status = msg

    def cmd(self, cli, motor_id):
        req = MotorIdCommand.Request()
        req.motor_id = motor_id
        return call(self, cli, req)

    def enable_to_ready(self, timeout=40):
        assert self.enable_cli.wait_for_service(timeout_sec=30), \
            "enable service absent"
        spin(self, 2.0)
        resp = call(self, self.enable_cli, Trigger.Request(), timeout=timeout)
        if not resp.success and "no /joint_states" in resp.message:
            for _ in range(8):
                spin(self, 1.0)
                resp = call(self, self.enable_cli, Trigger.Request(),
                            timeout=timeout)
                if resp.success or "no /joint_states" not in resp.message:
                    break
        if not resp.success:
            raise RuntimeError(f"enable rejected: {resp.message}")
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            spin(self, 0.1)
            if self.status is not None and self.status.state == "READY":
                return
        raise RuntimeError("state did not reach READY")


def start_maintenance(env):
    exe = os.path.join(
        WS, "install/a3_hardware_interface/lib/a3_hardware_interface/"
            "motor_maintenance")
    return ProcessGroup(
        [exe, "--ros-args", "-p", f"can_interface:={VCAN}"],
        "/tmp/f91_maintenance.log", env=env)


def start_stack():
    return ProcessGroup(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={VCAN}",
         "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false",
         "use_rosbag:=false"],
        "/tmp/f91_stack.log", env=make_env())


def phase_a(node):
    print("\n==== Phase A: standalone maintenance ====", flush=True)
    for p in (STATE_A, STATE_B):
        try:
            os.remove(p)
        except FileNotFoundError:
            pass
    sim = start_sim(STATE_A, "a")
    try:
        time.sleep(1.5)
        assert node.set_zero_cli.wait_for_service(timeout_sec=20), \
            "set_zero service absent"

        # 1. set_zero single (motor 3 starts at angle 0.785)
        resp = node.cmd(node.set_zero_cli, 3)
        exp_z = {i: 0 for i in range(1, 8)}
        exp_z[3] = 1
        exp_s = {i: 0 for i in range(1, 8)}
        motors = wait_counts(STATE_A, exp_z, exp_s)
        check("set_zero single accepted", resp.success, resp.message)
        check("set_zero single: count +1, angle rebased",
              motors["3"]["set_zero_count"] == 1 and
              abs(motors["3"]["angle"]) < 1e-6)

        # 2. save_parameters single
        resp = node.cmd(node.save_cli, 5)
        exp_s[5] = 1
        motors = wait_counts(STATE_A, exp_z, exp_s)
        check("save_parameters single accepted", resp.success, resp.message)
        check("save_parameters single: flash count +1",
              motors["5"]["save_param_count"] == 1)

        # 3. set_zero broadcast: m3 gets its 2nd, motors 2 (init 0.785) rebased
        resp = node.cmd(node.set_zero_cli, 255)
        exp_z = {i: 1 for i in range(1, 8)}
        exp_z[3] = 2
        motors = wait_counts(STATE_A, exp_z, exp_s)
        check("set_zero broadcast accepted", resp.success, resp.message)
        check("set_zero broadcast: all 7 counts",
              all(motors[str(i)]["set_zero_count"] == exp_z[i]
                  for i in range(1, 8)))
        check("set_zero broadcast: every angle 0",
              all(abs(motors[str(i)]["angle"]) < 1e-6
                  for i in range(1, 8)))

        # 4. save_parameters broadcast
        resp = node.cmd(node.save_cli, 255)
        exp_s = {i: 1 for i in range(1, 8)}
        exp_s[5] = 2
        motors = wait_counts(STATE_A, exp_z, exp_s)
        check("save_parameters broadcast accepted", resp.success,
              resp.message)
        check("save_parameters broadcast: all 7 counts",
              all(motors[str(i)]["save_param_count"] == exp_s[i]
                  for i in range(1, 8)))

        # 5. invalid id: refused, nothing changes
        resp = node.cmd(node.set_zero_cli, 9)
        time.sleep(0.8)
        motors = read_state(STATE_A)["motors"]
        check("motor_id out of range refused", not resp.success, resp.message)
        check("invalid motor_id: no counts changed",
              all(motors[str(i)]["set_zero_count"] == exp_z[i] and
                  motors[str(i)]["save_param_count"] == exp_s[i]
                  for i in range(1, 8)))
    finally:
        sim.terminate()


def phase_b(node, maintenance):
    print("\n==== Phase B: controller interlock ====", flush=True)
    sim = start_sim(STATE_B, "b", gravity=True)
    stack = None
    try:
        time.sleep(1.5)
        stack = start_stack()
        node.enable_to_ready()

        # 6/7. active controllers: refused before any CAN frame
        rz = node.cmd(node.set_zero_cli, 255)
        rs = node.cmd(node.save_cli, 255)
        time.sleep(1.0)
        motors = read_state(STATE_B)["motors"]
        no_counts = all(
            motors[str(i)]["set_zero_count"] == 0 and
            motors[str(i)]["save_param_count"] == 0
            for i in range(1, 8))
        check("set_zero refused while controllers active",
              not rz.success and "active" in rz.message, rz.message)
        check("set_zero refusal sent no frames", no_counts)
        check("save_parameters refused while controllers active",
              not rs.success and "active" in rs.message, rs.message)

        # 8. stop stack; maintenance allowed again (sim keeps running)
        stack.hard_kill()
        stack = None
        time.sleep(4.0)  # let controller_manager discovery drop
        resp = node.cmd(node.set_zero_cli, 3)
        exp_z = {i: 0 for i in range(1, 8)}
        exp_z[3] = 1
        motors = wait_counts(STATE_B, exp_z,
                             {i: 0 for i in range(1, 8)})
        check("maintenance works again after stack stopped",
              resp.success, resp.message)
        check("post-shutdown set_zero count landed",
              motors["3"]["set_zero_count"] == 1 and
              abs(motors["3"]["angle"]) < 1e-6)
    except Exception:
        import traceback
        traceback.print_exc()
        if os.path.exists("/tmp/f91_stack.log"):
            with open("/tmp/f91_stack.log", errors="replace") as f:
                print("\n---- tail stack log ----\n", f.read()[-3000:],
                      flush=True)
        raise
    finally:
        if stack is not None:
            stack.hard_kill()
        sim.terminate()


def main():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    env = make_env()
    rclpy.init()
    node = HarnessNode()
    ensure_vcan(VCAN)
    maintenance = start_maintenance(env)

    try:
        phase_a(node)
        phase_b(node, maintenance)
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        maintenance.terminate()

    node.destroy_node()
    rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F91 acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
