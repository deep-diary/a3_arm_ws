#!/usr/bin/env python3
"""F117 acceptance: JTC trajectory tolerance 0.30/0.05 survives the degraded lag.

2026-09-25 真机实跑：CAN 反馈降级（/joint_states ~12.3 Hz、最大空洞 0.392 s、
`CAN write failed: Resource temporarily unavailable`），goto 回 home 途中 JTC
容差 abort（`Position Error: 0.3073 > 0.300000`）→ FSM 震动 → R3 无法回 home
→ SAFE_PARK 也 abort 进 FAULT。根因是 CAN 降级导致的**零跟踪**（3 条独立证据），
不是容差过窄——error 在 0.32 s 冲到 0.3073 且等于「恰好指令距离」，0.30/0.15
都挡不住它会一直冲到 L3 折叠幅 ~1.6 rad。故 0.30 是**临时放宽的良性滞后余量**，
本质是卡滞/零跟踪检测器（非防撞），真正的修复是 CAN 健康。

本验收用 vcan 栈（真插件 + 真 JTC + 真容差引擎）复现「可承受的降级滞后」量级：
  vcan_motor_sim.py --alpha 0.012 --alpha-hz 200 → 命令环实际刷新率 ≈133 Hz
  （非假定 200 Hz），实测 TC_eff ≈0.42 s；滞后模型 err ≈ 2·TC_eff·v（0112 校准：
  v=1.575/11.5=0.137 → 实测 0.124，与 2·0.42·0.137=0.115 吻合）。idle<->ready
  相对位移 Δ 的两点轨迹时长 7.0 s → v=0.225 → 峰值 |ref-fbk| ≈0.20 rad ——
  落在 (0.15, 0.30) 窗口中部：
  * Case A：配置恒定 0.30/0.05 → 双向 FJT 都 SUCCESSFUL（error 0），且实测
    峰值误差 ∈ (0.15, 0.30) → 证明正好在 F117 覆盖的「会 abort 0.15」的降级
    滞后区，而 0.30 不再误杀。
  * Case B：运行时 hot-set 各关节 trajectory=0.15 → 同一运动 abort
    （error != 0，PATH_TOLERANCE_VIOLATED）→ 证明容差是活的、能探测到回归。
  * Case E：--alpha 0.002（TC≈3.8 s，滞后 ≈1.9 rad）→ 0.30 也 abort →
    证明 0.30 不是「无限容差」，极端滞后仍会被拦。

与 p1p2 相同：A/B 以「使能后的活锚点 q0」为基准做相对 Δ 位移（免疫 sim wrap
artifact）；JTC 容差引擎只需 err 量级不关心 (TC,v) 拆分。

  python3 scripts/a3_test/f117_jtc_tolerance_deg_acceptance.py \
      [DOMAIN_ID] [--alpha X] [--duration D]

Acceptance（docs/edge/REQUIREMENTS.md F117）：
  A. 带降级滞后量级，从活锚点按 idle->ready 的相对位移 Δ 运动 -> SUCCESSFUL
  B. 反向回锚点 -> SUCCESSFUL
  band. 摆程 |ref-fbk| 峰值落入 (0.15, 0.30)（=JTC trajectory 容差引擎实测量）
  D. 热设置 trajectory=0.15 -> 同一运动 abort（error != 0）
  E. --alpha 0.002 重跑（滞后 ~1.7 rad）-> 0.30 也 abort（error != 0）

Exit code 0 = all checks passed.
"""

import argparse
import os
import signal
import subprocess
import sys
import time

from builtin_interfaces.msg import Duration

import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

WS = "/home/cat/a3_arm_ws"
VCAN = "vcan58"
SIM_LOG_R1 = "/tmp/f117_sim_r1.log"
STACK_LOG_R1 = "/tmp/f117_stack_r1.log"
SIM_LOG_R2 = "/tmp/f117_sim_r2.log"
STACK_LOG_R2 = "/tmp/f117_stack_r2.log"

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
# idle<->ready 相对位移（ready = [0, 1.05, -1.575, 0, 0, 0]），用作 Δ
DELTA = [0.0, 1.05, -1.575, 0.0, 0.0, 0.0]
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
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            self.proc.wait(timeout=10)


def make_env(domain):
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = domain
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    env["PYTHONNOUSERSITE"] = "1"
    return env


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


class HarnessNode(Node):
    def __init__(self):
        super().__init__("f117_acceptance")
        self.js = None
        self.js_stamp = None
        self.max_track_err = 0.0
        self.sampling = False
        self._dbg_t0 = None
        self._dbg_hits = 0
        self._dbg_hi_e = 0.0
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(
            JointTrajectoryControllerState, "/arm_controller/controller_state",
            self._on_ctrl_state, 10)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")
        self.fjt = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory")

    def _on_js(self, msg):
        self.js = msg
        self.js_stamp = time.time()

    def _on_ctrl_state(self, msg):
        # 与 JTC trajectory-tolerance 引擎同量：|reference - feedback|（窗口采样，
        # 排除 enable/settle 瞬态）
        ref = msg.reference.positions
        fbk = msg.feedback.positions
        n = min(len(ref), len(fbk))
        if self.sampling:
            end = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self._dbg_t0 is None:
                self._dbg_t0 = end
            for i in range(n):
                e = abs(float(fbk[i]) - float(ref[i]))
                if e > self.max_track_err:
                    self.max_track_err = e
                    if e > 0.05 and (self._dbg_hits < 8 or e > self._dbg_hi_e):
                        self._dbg_hi_e = e
                        self._dbg_hits += 1
                        rel = end - self._dbg_t0
                        print(f"[dbg] t+{rel:5.1f}s j{i} ref={ref[i]:+.3f} "
                              f"fbk={fbk[i]:+.3f} err={e:.4f}", flush=True)

    def arm_positions(self):
        return [self.js.position[self.js.name.index(n)] for n in ARM_JOINTS]

    def wait_fresh_js(self, timeout=40):
        self.js_stamp = None
        t0 = time.monotonic()
        while self.js_stamp is None and time.monotonic() - t0 < timeout:
            spin(self, 0.1)
        return self.js_stamp is not None

    def enable(self):
        assert self.enable_cli.wait_for_service(timeout_sec=20)
        spin(self, 2.0)
        resp = call(self, self.enable_cli, Trigger.Request(), timeout=40)
        if not resp.success:
            raise RuntimeError(f"enable rejected: {resp.message}")
        spin(self, 3.0)

    def run_trajectory(self, q_target, duration_s, result_wait_s):
        self.sampling = False
        q0 = self.arm_positions()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)

        def dur(s):
            return Duration(sec=int(s), nanosec=int((s % 1) * 1e9))

        p0 = JointTrajectoryPoint(
            positions=[float(v) for v in q0],
            velocities=[0.0] * 6, accelerations=[0.0] * 6,
            time_from_start=dur(0.0))
        p1 = JointTrajectoryPoint(
            positions=[float(v) for v in q_target],
            velocities=[0.0] * 6, accelerations=[0.0] * 6,
            time_from_start=dur(duration_s))
        goal.trajectory.points = [p0, p1]

        gh_fut = self.fjt.send_goal_async(goal)
        end = time.monotonic() + 8
        while not gh_fut.done() and time.monotonic() < end:
            spin(self, 0.05)
        if not gh_fut.done():
            return False, None, None
        handle = gh_fut.result()
        if not handle.accepted:
            return False, None, None

        self.sampling = True
        res_fut = handle.get_result_async()
        t0 = time.monotonic()
        end = t0 + result_wait_s
        while not res_fut.done() and time.monotonic() < end:
            spin(self, 0.05)
        self.sampling = False
        if not res_fut.done():
            return True, None, None
        return True, time.monotonic() - t0, res_fut.result().result.error_code


def ensure_vcan():
    for cmd in (["sudo", "-S", "modprobe", "vcan"],
                ["sudo", "-S", "ip", "link", "del", "dev", VCAN],
                ["sudo", "-S", "ip", "link", "add", "dev", VCAN, "type", "vcan"],
                ["sudo", "-S", "ip", "link", "set", VCAN, "up"]):
        subprocess.run(cmd, input=b"temppwd\n", capture_output=True)


def set_jtc_trajectory_tolerance(node, value):
    """hot-set 各关节 constraints.L{i}_joint.trajectory（F117 同 JTC 运行时参数）。"""
    cli = node.create_client(SetParameters, "/arm_controller/set_parameters")
    if not cli.wait_for_service(timeout_sec=10):
        return False, "set_parameters service unavailable"
    req = SetParameters.Request()
    req.parameters = [
        RosParameter(
            name=f"constraints.L{i}_joint.trajectory",
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(value)),
        )
        for i in range(1, 7)
    ]
    resp = call(node, cli, req, timeout=10)
    ok = all(r.successful for r in resp.results)
    return ok, [r.successful for r in resp.results]


def launch_stack(env, sim_log, stack_log, alpha):
    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", VCAN, "--alpha", str(alpha), "--alpha-hz", "200"],
        sim_log, env=env)
    time.sleep(1.0)
    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={VCAN}",
         "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"],
        stack_log, env=env)
    return sim, stack


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("domain", nargs="?", default="58")
    ap.add_argument("--alpha", type=float, default=0.012,
                    help="vcan first-order follow gain: err≈TC*v. 实测命令环刷新率"
                         "≈133 Hz → 0.012→TC≈0.42 s → peak ~0.22 rad "
                         "(in 0.15..0.30 degraded window); 0.002→TC≈3.8 s → "
                         "~1.9 rad (extreme lag).")
    ap.add_argument("--duration", type=float, default=7.0,
                    help="idle<->ready two-point duration. err≈2*TC_eff*v, "
                         "TC_eff≈0.42 s（实测校准）; v=Δ/t, L3 Δ=1.575 rad. "
                         "t=11.5 实测 lag≈0.124；t=7.0 → v=0.225 → lag≈0.20 in "
                         "(0.15, 0.30)。")
    args = ap.parse_args()

    os.environ["ROS_DOMAIN_ID"] = args.domain
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["PYTHONNOUSERSITE"] = "1"

    ensure_vcan()
    env = make_env(args.domain)
    rclpy.init()
    node = HarnessNode()
    rw = args.duration + 6.0

    # =================== Run 1: 配置 0.30/0.05, alpha = args.alpha ===========
    sim1 = stack1 = None
    try:
        sim1, stack1 = launch_stack(env, SIM_LOG_R1, STACK_LOG_R1, args.alpha)
        if not node.wait_fresh_js():
            raise RuntimeError("run1: no /joint_states in 40 s")
        node.enable()
        assert node.fjt.wait_for_server(timeout_sec=10)

        anchor = node.arm_positions()
        node.max_track_err = 0.0  # Case band 只看 Run1 的 A/B 摆程

        acc, elapsed, code = node.run_trajectory(
            [anchor[i] + DELTA[i] for i in range(6)], args.duration, rw)
        check("A degraded-Δ idle->ready accepted/terminated",
              acc, f"code={code} elapsed={elapsed}")
        check("A degraded-Δ idle->ready SUCCESSFUL (0.30 tolerated)",
              acc and code == 0, f"code={code}")

        acc, elapsed, code = node.run_trajectory(anchor, args.duration, rw)
        check("B degraded-Δ ready->idle accepted/terminated",
              acc, f"code={code} elapsed={elapsed}")
        check("B degraded-Δ ready->idle SUCCESSFUL (0.30 tolerated)",
              acc and code == 0, f"code={code}")

        e = node.max_track_err
        check("band peak |ref-fbk| in (0.15, 0.30) degraded window",
              0.15 < e < 0.30,
              f"observed={e:.3f} 期望~0.20（err≈2·TC_eff·v, TC_eff≈0.42 s 实测校准）")

        ok, detail = set_jtc_trajectory_tolerance(node, 0.15)
        check("D hot-set trajectory=0.15 applied", ok, f"results={detail}")
        # 回锚点位置本轮已回到 anchor；再跑同一 A 运动，err~0.20 > 0.15 → abort
        acc, elapsed, code = node.run_trajectory(
            [anchor[i] + DELTA[i] for i in range(6)], args.duration, rw)
        check("D hot-set 0.15 aborts the same degraded sweep (regression probe)",
              acc and code != 0, f"code={code} elapsed={elapsed}")
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        if stack1:
            stack1.terminate()
        if sim1:
            sim1.terminate()
        time.sleep(2.0)

    # ============== Run 2: alpha = 0.002（TC=2.5 s, 滞后 ~0.9 rad）==============
    sim2 = stack2 = None
    try:
        sim2, stack2 = launch_stack(env, SIM_LOG_R2, STACK_LOG_R2, 0.002)
        if not node.wait_fresh_js():
            raise RuntimeError("run2: no /joint_states in 40 s")
        node.enable()
        assert node.fjt.wait_for_server(timeout_sec=10)

        anchor = node.arm_positions()
        acc, elapsed, code = node.run_trajectory(
            [anchor[i] + DELTA[i] for i in range(6)], args.duration, rw)
        check("E extreme lag (alpha=0.002, ~1.9 rad) aborts at 0.30",
              acc and code != 0,
              f"code={code} elapsed={elapsed} (0.30 is bounded, not unlimited)")
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        if stack2:
            stack2.terminate()
        if sim2:
            sim2.terminate()
        node.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F117 JTC tolerance acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())