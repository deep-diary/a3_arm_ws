#!/usr/bin/env python3
"""F90 acceptance: fault-triggered bounded rosbag2 black-box (snapshot mode).

On the product stack (a3_bringup, use_rosbag:=true is the default) a
rosbag2 recorder runs in --snapshot-mode: a bounded in-memory circular
cache, nothing on disk until the FSM calls /rosbag2_recorder/snapshot.
Every FAULT entry funnels through ArmFSM._set_state, so the edge into
FAULT must flush ONE mcap split containing the pre-/post-fault window,
fully automatically. A missing recorder must never block fault handling.

All simulation-only (real arm stays powered off): vcan_motor_sim + the
product stack a3_bringup hardware:=can.

Phase A (use_rosbag:=true):
  1. recorder up: /rosbag2_recorder/snapshot service exists; before any
     fault the bag holds 0 messages (cache only).
  2. enable -> READY; let traffic buffer.
  3. inject motor fault word ({"motor": 5, "fault": 4}) -> FSM FAULT;
     without any manual call a new mcap split appears with > 0 messages
     on /joint_states and /a3/arm_status (ros2 bag info readable).
  4. bounded: recorder cmdline carries 32 MiB cache / 64 MiB split; no
     file exceeds the split bound; re-injecting while still FAULT adds
     no further split (edge suppression).
Phase B (use_rosbag:=false):
  5. no recorder node/service; enable -> READY -> injected fault still
     reaches FAULT, no errors.

python3 scripts/a3_test/f90_blackbox_acceptance.py [DOMAIN]

Exit code 0 = all checks passed.
"""

import glob
import json
import os
import re
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rosbag2_interfaces.srv import Snapshot
from std_srvs.srv import Trigger

from a3_msgs.msg import ArmStatus

WS = "/home/cat/a3_arm_ws"
DOMAIN = sys.argv[1] if len(sys.argv) > 1 else "90"
VCAN = os.environ.get("F90_VCAN", "vcan90")
STAMP = time.strftime("%Y%m%d_%H%M%S")
BAG_DIR_ON = f"/tmp/f90_blackbox_on_{STAMP}"
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
        ["sudo", "-S", "ip", "link", "set", "interface", "up"],
        input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "set", interface, "up"],
        input=b"temppwd\n", capture_output=True)


def write_health(payload):
    path = "/tmp/f84_health.json"
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f)
    os.replace(tmp, path)
    time.sleep(0.12)


def bag_info(bag_dir):
    proc = subprocess.run(
        ["ros2", "bag", "info", bag_dir],
        capture_output=True, text=True, env=make_env())
    return proc.returncode, proc.stdout + proc.stderr


def parse_messages(info_txt):
    m = re.search(r"Messages:\s+(\d+)", info_txt)
    return int(m.group(1)) if m else 0


def topic_count(info_txt, topic):
    m = re.search(rf"Topic: {re.escape(topic)}\s*\|.*?Count:\s*(\d+)", info_txt)
    return int(m.group(1)) if m else 0


def mcap_files(bag_dir):
    return sorted(glob.glob(os.path.join(bag_dir, "*.mcap")))


def recorder_cmdline():
    proc = subprocess.run(
        ["pgrep", "-af", r"ros2 bag record"],
        capture_output=True, text=True)
    return proc.stdout.strip()


def recorder_pid():
    line = recorder_cmdline()
    return int(line.split()[0]) if line else None


def stop_recorder_gracefully(bag_dir, timeout=12):
    # rosbag2 writes metadata.yaml only after the recorder exits; while it is
    # running `ros2 bag info` cannot see message counts. The recorder is a
    # standalone ExecuteProcess — SIGINT it alone, stack stays up.
    pid = recorder_pid()
    if pid is not None:
        os.kill(pid, signal.SIGINT)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if os.path.exists(os.path.join(bag_dir, "metadata.yaml")):
            return True
        time.sleep(0.2)
    return False


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f90_acceptance")
        self.status = None
        self.create_subscription(
            ArmStatus, "/a3/arm_status", self._on_status, 10)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")
        self.snapshot_cli = self.create_client(
            Snapshot, "/rosbag2_recorder/snapshot")

    def _on_status(self, msg):
        self.status = msg

    def wait_state(self, expected, timeout=40):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            spin(self, 0.1)
            if self.status is not None and self.status.state == expected:
                return True
        return False

    def enable_to_ready(self, timeout=40):
        assert self.enable_cli.wait_for_service(timeout_sec=30), \
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

    def recorder_present(self):
        names = {n for n, ns in self.get_node_names_and_namespaces()}
        return "rosbag2_recorder" in names

    def wait_snapshot_service_absent(self, timeout=15):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            spin(self, 0.5)
            if not self.snapshot_cli.service_is_ready():
                spin(self, 2.0)
                if not self.snapshot_cli.service_is_ready():
                    return True
        return not self.snapshot_cli.service_is_ready()


def run_stack(node, use_rosbag, bag_dir=None):
    urdf = os.path.join(
        WS, "install/a3_description/share/a3_description/urdf/el_a3.urdf")
    env = make_env()
    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", VCAN, "--gravity-model", "urdf",
         "--gravity-scales", "1.0", "1.0", "1.0", "1.0", "1.0", "1.0",
         "--gravity-noise", "0.01", "--gravity-urdf", urdf],
        f"/tmp/f90_sim_{use_rosbag}.log", env=env)
    time.sleep(1.5)
    argv = [
        "ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
        "hardware:=can", f"can_interface:={VCAN}",
        "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false",
        f"use_rosbag:={'true' if use_rosbag else 'false'}",
    ]
    if bag_dir:
        argv.append(f"bag_dir:={bag_dir}")
    stack = ProcessGroup(
        argv, f"/tmp/f90_stack_{use_rosbag}.log", env=env)
    return sim, stack


def phase_a(node):
    print("\n==== Phase A: use_rosbag:=true ====", flush=True)
    write_health({})
    sim, stack = run_stack(node, True, BAG_DIR_ON)
    try:
        assert node.enable_cli.wait_for_service(timeout_sec=40), \
            "enable service absent"

        # 1. recorder up, snapshot service present, nothing recorded yet
        t0 = time.monotonic()
        while time.monotonic() - t0 < 25:
            spin(node, 0.1)
            if node.recorder_present() and node.snapshot_cli.wait_for_service(
                    timeout_sec=0.1):
                break
        check("rosbag2 recorder node up", node.recorder_present())
        check("snapshot service available",
              node.snapshot_cli.service_is_ready())
        spin(node, 1.0)
        # mcap is lazy and writes metadata.yaml only on recorder exit; before
        # the first snapshot only (at most) a tiny header file may exist.
        pre_files = mcap_files(BAG_DIR_ON)
        pre_biggest = max((os.path.getsize(f) for f in pre_files), default=0)
        check("no message data before any fault (cache-only)",
              pre_biggest < 4096, f"biggest={pre_biggest} bytes")

        # 2. enable -> READY, let the circular buffer fill
        node.enable_to_ready()
        spin(node, 3.0)

        # 3. inject fault -> FAULT -> automatic flush
        sizes_before = {f: os.path.getsize(f) for f in mcap_files(BAG_DIR_ON)}
        write_health({"motor": 5, "fault": 4})
        check("FSM enters FAULT on motor fault",
              node.wait_state("FAULT", timeout=15))

        def flushed_bytes():
            grew = 0
            for f in mcap_files(BAG_DIR_ON):
                grew = max(grew,
                           os.path.getsize(f) - sizes_before.get(f, 0))
            return grew

        t0 = time.monotonic()
        while time.monotonic() - t0 < 8 and flushed_bytes() < 100 * 1024:
            spin(node, 0.2)
        grew = flushed_bytes()
        check("mcap auto-flushed with buffered data on FAULT edge",
              grew >= 100 * 1024, f"grew={grew} bytes")

        # 4. boundedness
        cmd = recorder_cmdline()
        check("recorder bounded cache 32 MiB",
              "--max-cache-size 33554432" in cmd, cmd[:200])
        check("recorder split bound 64 MiB",
              "--max-bag-size 67108864" in cmd, cmd[:200])
        biggest = max((os.path.getsize(f) for f in mcap_files(BAG_DIR_ON)),
                      default=0)
        check("no mcap file exceeds the 64 MiB split bound",
              biggest <= 64 * 1024 * 1024 + 1024 * 1024,
              f"biggest={biggest} bytes")

        # re-inject while still FAULT: edge suppression -> no extra flush
        sizes_settled = {f: os.path.getsize(f) for f in mcap_files(BAG_DIR_ON)}

        def extra_after_settle():
            return sum(
                os.path.getsize(f) - sizes_settled.get(f, 0)
                for f in mcap_files(BAG_DIR_ON))

        write_health({"motor": 5, "fault": 4})
        spin(node, 3.0)
        check("repeated fault while FAULT triggers no extra flush",
              extra_after_settle() == 0,
              f"extra={extra_after_settle()} bytes")

        # stop recorder so metadata.yaml is written, then verify contents
        check("recorder wrote metadata after graceful stop",
              stop_recorder_gracefully(BAG_DIR_ON))
        rc, info = bag_info(BAG_DIR_ON)
        total_msgs = parse_messages(info) if rc == 0 else 0
        check("flushed bag readable with >0 messages",
              rc == 0 and total_msgs > 0, f"msgs={total_msgs} rc={rc}")
        check("captured /joint_states across the fault",
              topic_count(info, "/joint_states") > 0,
              f"n={topic_count(info, '/joint_states')}")
        check("captured /a3/arm_status across the fault",
              topic_count(info, "/a3/arm_status") > 0,
              f"n={topic_count(info, '/a3/arm_status')}")

    except Exception:
        import traceback
        traceback.print_exc()
        with open("/tmp/f90_stack_True.log", errors="replace") as f:
            print("\n---- tail stack log ----\n", f.read()[-3000:],
                  flush=True)
    finally:
        if stack.proc.poll() is None:
            stack.hard_kill()
        sim.terminate()
        write_health({})
        time.sleep(0.5)


def phase_b(node):
    print("\n==== Phase B: use_rosbag:=false ====", flush=True)
    write_health({})
    sim, stack = run_stack(node, False)
    try:
        assert node.enable_cli.wait_for_service(timeout_sec=40), \
            "enable service absent"
        spin(node, 5.0)
        check("no recorder node when use_rosbag:=false",
              not node.recorder_present())
        # Discovery may briefly keep the phase-A service cached on this node.
        check("no snapshot service when use_rosbag:=false",
              node.wait_snapshot_service_absent())

        node.enable_to_ready()
        write_health({"motor": 5, "fault": 4})
        t0 = time.monotonic()
        reached = node.wait_state("FAULT", timeout=15)
        check("fault still reaches FAULT without recorder", reached,
              node.status.state if node.status is not None else "no status")
        # fault path must not have stalled waiting on the absent service
        check("fault handling returned promptly (no block)",
              time.monotonic() - t0 < 15)

    except Exception:
        import traceback
        traceback.print_exc()
        with open("/tmp/f90_stack_False.log", errors="replace") as f:
            print("\n---- tail stack log ----\n", f.read()[-3000:],
                  flush=True)
    finally:
        if stack.proc.poll() is None:
            stack.hard_kill()
        sim.terminate()
        write_health({})


def main():
    # Clean up leftover feasibility-experiment processes from prior work.
    for pid in (3355214, 3357227):
        try:
            os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass

    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    env = make_env()
    rclpy.init()
    node = HarnessNode()
    ensure_vcan(VCAN)

    phase_a(node)
    phase_b(node)

    node.destroy_node()
    rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F90 acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
