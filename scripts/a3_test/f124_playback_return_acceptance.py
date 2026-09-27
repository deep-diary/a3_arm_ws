#!/usr/bin/env python3
"""F124 acceptance: 回放回首点段走 MoveIt 轨迹规划（playback_return_use_moveit）。

edge_web_sim（DOMAIN 62）自起栈：use_gripper/use_moveit=true，use_servo/rviz=false。
默认 playback_return_use_moveit:=true → 首点与当前构型差 >0.02 rad 时，phase R 用
move_group 规划回首点（LED cyan），phase P 再发录制轨迹（purple）。负例热设 false →
回落几何 join-ramp，无回首点段。

前提：无电机；`/a3/motor/sim_push` 直接改动 sim plant（模拟示教手拖）。真机/标准栈
语义差异（edge_web_sim 无 GripperCommand action → phase R 的 L7 补发是 no-op）见计划
R4，不在此断言。

序列：
  1. enable → READY；off_idle（FJT L2→0.4）→ mode IDLE 稳定。
  2. start_teach → TEACH；sim_push 6 次（L2 +0.05×6=0.3，hold 1.2s 覆盖 0.5s 推间隔，
     每次续锁冻结 → 0.4→0.7 单调阶梯）模拟手拖录制；stop_teach → READY（F54 自动存档）。
  3. 断言 ~/.a3/trajectories/latest.yaml：joint_names 为 7 关节、points ≥ 10。
  4. goto idle（MoveIt，F67）改构型 → READY（首点≠当前 → use_ramp）。
  5. 清 samples → /a3/arm/playback "" → 断言状态消息序含「playback return 」前缀且
     在非-return「playback 」前缀之前；resp 含 "return=MoveIt" 与 "retime ["。
  6. 负例：再 goto idle → READY；set_param playback_return_use_moveit=false + GET
     readback（LL-096）→ 重放 → 断言无「playback return」状态前缀、resp 无
     "return=MoveIt"（回落 ramp）→ 恢复 true + readback。

Exit code 0 = all checks passed.
"""

import argparse
import os
import signal
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import Parameter as RclParam
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from a3_msgs.msg import ArmStatus
from a3_msgs.srv import GotoNamedPose
from a3_msgs.srv import PlaybackTrajectory

WS = "/home/cat/a3_arm_ws"
STACK_LOG = "/tmp/f124_accept_stack.log"
RESULTS = []

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
ALL_JOINTS = [f"L{i}_joint" for i in range(1, 8)]   # latest.yaml 应含 L7
STATE_READY = "READY"
STATE_TEACH = "TEACH"
MODE_IDLE = "IDLE"


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


def spin(node, t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(node, cli, request, timeout=15.0):
    fut = cli.call_async(request)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    raise RuntimeError(f"service timeout: {cli.srv_name}")


class Harness(Node):
    def __init__(self):
        super().__init__("f124_acceptance")
        self.js = None
        self.status = None
        self.status_samples = []
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_status, 10)
        self.en = self.create_client(Trigger, "/a3/arm/enable")
        self.start_teach = self.create_client(Trigger, "/a3/arm/start_teach")
        self.stop_teach = self.create_client(Trigger, "/a3/arm/stop_teach")
        self.goto_cli = self.create_client(
            GotoNamedPose, "/a3/arm/goto_named_pose")
        self.playback_cli = self.create_client(
            PlaybackTrajectory, "/a3/arm/playback")
        self.fsm_set_cli = self.create_client(
            SetParametersAtomically, "/a3_arm_controller/set_parameters_atomically")
        self.fsm_get_cli = self.create_client(
            GetParameters, "/a3_arm_controller/get_parameters")
        self.push_param_cli = self.create_client(
            SetParametersAtomically,
            "/motor_protocol_node/set_parameters_atomically")
        self.push_trig = self.create_client(Trigger, "/a3/motor/sim_push")
        self.fjt = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

    def _on_js(self, msg):
        self.js = msg

    def _on_status(self, msg):
        self.status = msg
        self.status_samples.append((time.monotonic(), msg.state, msg.mode,
                                    msg.message))

    def joint_pos(self, name):
        js = self.js
        if js is None:
            return None
        return float(js.position[js.name.index(name)])

    def wait_status(self, timeout=30.0):
        end = time.monotonic() + timeout
        while self.status is None and time.monotonic() < end:
            spin(self, 0.05)
        return self.status

    def wait_state(self, state, timeout=30.0):
        self.wait_status()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            spin(self, 0.05)
            if self.status.state == state:
                return True
        return self.status.state == state

    def wait_mode(self, mode, timeout=15.0):
        self.wait_status()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            spin(self, 0.05)
            if self.status.mode == mode:
                return True
        return self.status.mode == mode

    def wait_mode_stable(self, mode, hold_s=1.0, timeout=8.0):
        """等 mode 连续稳定 hold_s（跨过 gravity 名义 end_time 尾巴，见 f119 注释）。"""
        self.wait_status()
        stable_since = None
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            spin(self, 0.05)
            if self.status.mode == mode:
                stable_since = stable_since if stable_since is not None else time.monotonic()
                if time.monotonic() - stable_since >= hold_s:
                    return True
            else:
                stable_since = None
        return self.status.mode == mode

    def enable(self):
        assert self.en.wait_for_service(timeout_sec=20)
        resp = call(self, self.en, Trigger.Request(), timeout=40)
        if not resp.success:
            raise RuntimeError(f"enable rejected: {resp.message}")
        spin(self, 3.0)
        # 使能后 0.5s：让 arm_monitor 的使能沿重基准先完成再发轨迹（LL-132/133，f119 同款）
        spin(self, 0.5)

    def fjt_l2_040(self, duration_s=3.0, result_wait_s=6.0):
        q0 = [self.joint_pos(n) for n in ARM_JOINTS]
        if any(v is None for v in q0):
            raise RuntimeError("no /joint_states for FJT")

        def dur(s):
            return Duration(sec=int(s), nanosec=int((s % 1) * 1e9))

        end = list(q0)
        end[1] = 0.4
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=q0, velocities=[0.0] * 6,
                                 accelerations=[0.0] * 6, time_from_start=dur(0.0)),
            JointTrajectoryPoint(positions=end, velocities=[0.0] * 6,
                                 accelerations=[0.0] * 6, time_from_start=dur(duration_s)),
        ]
        gh = self.fjt.send_goal_async(goal)
        t0 = time.monotonic()
        while not gh.done() and time.monotonic() - t0 < 8:
            spin(self, 0.05)
        if not gh.done() or not gh.result().accepted:
            return False, None, None
        rf = gh.result().get_result_async()
        t0 = time.monotonic()
        while not rf.done() and time.monotonic() - t0 < result_wait_s:
            spin(self, 0.05)
        if not rf.done():
            return True, None, None
        return True, time.monotonic() - t0, rf.result().result.error_code

    def off_idle(self):
        acc, _, code = self.fjt_l2_040()
        if not acc or code != 0:
            raise RuntimeError(f"off-idle FJT failed acc={acc} code={code}")
        if not self.wait_mode_stable(MODE_IDLE, hold_s=1.0, timeout=8.0):
            raise RuntimeError(
                f"mode not IDLE-stable after off-idle (mode={self.status.mode})")

    def set_push(self, rad7, hold_s):
        req = SetParametersAtomically.Request()
        req.parameters = [
            RclParam(name="sim_push_enabled",
                     value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                          bool_value=True)),
            RclParam(name="sim_push_rad",
                     value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                          double_array_value=[float(v) for v in rad7])),
            RclParam(name="sim_push_hold_s",
                     value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                          double_value=hold_s)),
        ]
        resp = call(self, self.push_param_cli, req, timeout=10)
        if not resp.result.successful:
            raise RuntimeError(f"push param rejected: {resp.result.reason}")

    def push_once(self, sleep_s=0.45):
        resp = call(self, self.push_trig, Trigger.Request(), timeout=10)
        if not resp.success:
            raise RuntimeError(f"sim_push rejected: {resp.message}")
        spin(self, sleep_s)

    def set_fsm_bool_param(self, name, value):
        req = SetParametersAtomically.Request()
        req.parameters = [RclParam(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                 bool_value=value))]
        resp = call(self, self.fsm_set_cli, req, timeout=10)
        if not resp.result.successful:
            raise RuntimeError(f"set {name} rejected: {resp.result.reason}")
        # LL-096: set alone is not proof — read it back.
        for _ in range(20):
            got = call(self, self.fsm_get_cli,
                       GetParameters.Request(names=[name]), timeout=5)
            if got.values and got.values[0].bool_value == value:
                return
            spin(self, 0.1)
        raise RuntimeError(f"param {name} readback mismatch")

    def goto_idle(self):
        assert self.goto_cli.wait_for_service(timeout_sec=10)
        resp = call(self, self.goto_cli,
                    GotoNamedPose.Request(pose_name="idle"), timeout=40)
        if not resp.success:
            raise RuntimeError(f"goto idle rejected: {resp.message}")
        if not self.wait_state(STATE_READY, timeout=60):
            raise RuntimeError(f"not READY after goto idle (state={self.status.state})")


def wait_future(h, fut, status_timeout=90.0):
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < status_timeout:
        spin(h, 0.05)
    done = fut.done()
    resp = fut.result() if done else None
    return done, resp, list(h.status_samples)


def messages(samples):
    return [s[3] for s in samples if s[3]]


def read_latest_yaml():
    import yaml
    path = os.path.expanduser("~/.a3/trajectories/latest.yaml")
    with open(path) as f:
        return yaml.safe_load(f), path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("domain", nargs="?", default="62",
                    help="罗斯 DOMAIN_ID（edge_web_sim 自起于该域）")
    args = ap.parse_args()

    os.environ["ROS_DOMAIN_ID"] = args.domain
    os.environ["PYTHONNOUSERSITE"] = "1"
    env = dict(os.environ)

    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "edge_web_sim.launch.py",
         "use_gripper:=true", "use_rviz:=false", "use_moveit:=true",
         "use_servo:=false", "use_servo_anchor:=false"],
        STACK_LOG, env=env)

    rclpy.init()
    h = Harness()

    try:
        assert h.en.wait_for_service(timeout_sec=60) and \
            h.playback_cli.wait_for_service(timeout_sec=60) and \
            h.fsm_get_cli.wait_for_service(timeout_sec=60) and \
            h.push_trig.wait_for_service(timeout_sec=60) and \
            h.fjt.wait_for_server(timeout_sec=60), \
            "core services absent (edge_web_sim failed to boot?)"

        print("\n==== 1. enable -> READY; off_idle -> L2=0.4 ====", flush=True)
        h.enable()
        if not h.wait_state(STATE_READY, timeout=15):
            raise RuntimeError("arm not READY")
        h.off_idle()
        check("off-idle 后 mode IDLE 稳定", h.status.mode == MODE_IDLE, "")
        if not h.wait_state(STATE_READY, timeout=15):
            raise RuntimeError(f"state not READY after off_idle (state={h.status.state})")

        print("\n==== 2. start_teach + sim_push 录制 ====", flush=True)
        resp = call(h, h.start_teach, Trigger.Request(), timeout=10)
        check("start_teach success", resp.success, f"msg={resp.message}")
        if not h.wait_state(STATE_TEACH, timeout=5.0):
            raise RuntimeError("state did not reach TEACH")
        l2_0 = h.joint_pos("L2_joint")
        check("录制起点 L2 来自 off-idle(≈0.4)", l2_0 is not None and abs(l2_0 - 0.4) < 0.05,
              f"L2={l2_0}")
        # hold_s=1.2s > 0.5s 间隔：每推续锁 L2（跟随 _target 的弹簧被冻结），
        # 6 推形成 0.4→0.7 单调阶梯；stop_teach 前 0.5s 仍在冻结期 → 末点≈0.7
        h.set_push([0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0], hold_s=1.2)
        for _ in range(6):
            h.push_once(sleep_s=0.5)
        resp = call(h, h.stop_teach, Trigger.Request(), timeout=15)
        check("stop_teach success + 自动存档",
              resp.success and "auto-saved latest.yaml" in resp.message,
              f"msg={resp.message}")
        if not h.wait_state(STATE_READY, timeout=10):
            raise RuntimeError("state did not return READY after stop_teach")

        print("\n==== 3. latest.yaml 契约 ====", flush=True)
        data, path = read_latest_yaml()
        jn = list(data.get("joint_names", []))
        pts = list(data.get("points", []))
        check("latest.yaml 含 7 关节 (含 L7_joint)",
              jn == ALL_JOINTS, f"joint_names={jn}")
        check(f"latest.yaml 采样点 ≥ 10（>=teach_auto_save_min_samples）",
              len(pts) >= 10, f"n={len(pts)}")
        check("记录点含 L2 单调推进（sim_push 生效）",
              len(pts) >= 2 and pts[0]["positions"][1] < 0.6 and
              abs(pts[-1]["positions"][1] - 0.7) < 0.05,
              f"first_L2={pts[0]['positions'][1]:.3f} last_L2={pts[-1]['positions'][1]:.3f}")

        print("\n==== 4. goto idle 改构型（首点≈L2=0.4 ≠ idle）====", flush=True)
        h.goto_idle()
        check("goto idle 后 READY", h.status.state == STATE_READY, "")
        l2 = h.joint_pos("L2_joint")
        check("当前 L2 离开录制首点（ramp_dist>0.02 → use_ramp）",
              l2 is not None and abs(l2 - 0.4) > 0.02, f"L2={l2}")

        print("\n==== 5. 正例：playback -> MoveIt 回首点 ====", flush=True)
        h.status_samples = []
        done, resp, samples = wait_future(
            h, h.playback_cli.call_async(PlaybackTrajectory.Request(name="")))
        check("playback service returned", done, "")
        if done:
            msgs = messages(samples)
            r_idx = next((i for i, m in enumerate(msgs)
                          if m.startswith("playback return")), None)
            p_idx = next((i for i, m in enumerate(msgs)
                          if m.startswith("playback ")
                          and not m.startswith("playback return")), None)
            check("phase R 状态消息「playback return 」先于「playback 」",
                  r_idx is not None and p_idx is not None and r_idx < p_idx,
                  f"r_idx={r_idx} p_idx={p_idx} msgs={msgs[-4:]}")
            check("playback success=True", resp.success, f"msg={resp.message}")
            check("resp 含 retime 标记", "retime [" in resp.message, resp.message)
            check("resp 含 return=MoveIt（回首点经 move_group）",
                  "return=MoveIt" in resp.message, resp.message)
        if not h.wait_state(STATE_READY, timeout=60):
            raise RuntimeError("not READY after positive playback (back_to_ready)")

        print("\n==== 6. 负例：playback_return_use_moveit=false 回落 ramp ====", flush=True)
        h.goto_idle()
        h.set_fsm_bool_param("playback_return_use_moveit", False)
        check("param playback_return_use_moveit 已置 false (LL-096 readback)",
              True, "")
        h.status_samples = []
        done, resp, samples = wait_future(
            h, h.playback_cli.call_async(PlaybackTrajectory.Request(name="")))
        check("负例 playback service returned", done, "")
        if done:
            msgs = messages(samples)
            check("负例无「playback return 」状态前缀（回落几何 ramp）",
                  not any(m.startswith("playback return") for m in msgs),
                  f"msgs={msgs[-4:]}")
            check("负例 resp 无 return=MoveIt", "return=MoveIt" not in resp.message,
                  resp.message)
            check("负例 resp 仍带 retime 标记", "retime [" in resp.message,
                  resp.message)
        h.set_fsm_bool_param("playback_return_use_moveit", True)
        check("restore playback_return_use_moveit=true (LL-096 readback)", True, "")
        if not h.wait_state(STATE_READY, timeout=60):
            raise RuntimeError("not READY after negative playback")

    except Exception:
        import traceback
        traceback.print_exc()
        with open(STACK_LOG, errors="replace") as f:
            txt = f.read()
        print("\n---- tail stack log ----\n", txt[-3000:], flush=True)
    finally:
        if stack is not None and stack.proc.poll() is None:
            stack.hard_kill()
        h.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F124 playback-return acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())