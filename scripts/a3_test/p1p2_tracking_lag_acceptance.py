#!/usr/bin/env python3
"""P1/P2 acceptance: JTC tracking-tolerance must survive the real-arm lag.

2026-09-24 实测（/tmp/a3_real_stack.log）：真机 goto 起跑约 0.2 s 内 L2 的
|Position Error| 已达 0.0508，越过 F97 的 trajectory tolerance 0.05 rad →
`Aborted due to state tolerance violation`（error -4）→ FSM 中途停住（P2）+
重复按键起跑-急冻-反冲（P1）。由 err ≈ TC*v_ref 反推巡航 v_ref≈0.46 rad/s、
跟随时间常数 TC≈0.11 s。

本验收用 vcan 栈（真插件 + 真 JTC + 真容差引擎）复现同样的滞后量级：
  vcan_motor_sim.py --alpha 0.023 --alpha-hz 200 给虚拟电机注入一阶滞后
  TC = 1/(alpha*200) = 0.22 s；以 home<->ready 的相对位移 Δ（L3=-1.575）为
  两点轨迹时长 11.5 s，实测 JTC spline 峰值参考速度 ≈0.35 rad/s（时长越长
  spline 越缓，3.5 s 两点会峰到 ≈2.2 rad/s = 4.8× 真机巡航而过度加载）。于是
  峰值 |ref-fbk| err ≈ TC*v ≈0.07 rad —— 落在实测 0.05 量级、新旧容差
  （0.05 旧 / 0.15 新）正中的复现窗口。

  A/B 以「使能后的活锚点 q0」为基准，运动量固定为 home<->ready 的相对位移
  Δ（A: q0→q0+Δ，B: q0+Δ→q0），而不是绝对目标。原因是 sim 冷启动自带一个
  启动 artifact：vcan_motor_sim 首个控制帧到达时 dt≈栈启动的 15 s，effort
  物理分支在一个大步里积分出大速度；随后 position-mode 重锚定经常落在被
  ±2π 环绕的 wrapped 角度（L2: +0.785→-4.831、L4: ±7.88~8.51 逐次运行不
  定），使能后的锚点非确定。真机（F91 折叠后多圈计数归零，使能锚在零位附
  近）不会出现 wrap；lag err≈TC*v 只取决于 Δ 与时长、与锚点绝对位置无关，
  故以相对 Δ 运行让本验收在任意锚点下都确定复现实测滞后量级，同时免疫该
  sim artifact。JTC 容差引擎只关心 err 本身的量级，不关心 (TC, v) 的组合拆
  分，故不强求 TC=真机 0.11 s：只要峰值 err 落在 0.050~0.085 窗口即证明复
  现了真实故障形态。FJT 轨迹必须以 error 0（SUCCESSFUL）完成；容差收紧回
  0.05 时本测试应失败（即真实 abort 复现）。

  python3 scripts/a3_test/p1p2_tracking_lag_acceptance.py \
      [DOMAIN_ID] [--alpha X] [--duration D]

Acceptance（docs/edge/REQUIREMENTS.md F112）:
  A. 带实测量级跟踪滞后，从活锚点按 home->ready 的相对位移 Δ 运动 ->
     SUCCESSFUL（error 0）
  B. 反向按 -Δ 回到锚点 -> SUCCESSFUL
  C. 整个摆程 |ref-fbk| 的最大值（= JTC trajectory-tolerance 引擎实际检查的
     量）落入 0.050~0.085 rad 窗口（0.05 旧容差之外、0.15 新容差的近半内层，
     证明测试确实在复现实测滞后量级，而非走过场）

Exit code 0 = all checks passed.

用法（先证伪旧配置，再验证修复）：
  python3 scripts/a3_test/p1p2_tracking_lag_acceptance.py   # 当前配置
  当 el_a3_controllers.yaml trajectory=0.05 时预计 A/B FAIL(error -4)、
  C PASS；放宽到 0.15 后 A/B PASS、C PASS。
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
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

WS = "/home/cat/a3_arm_ws"
VCAN = "vcan981"
SIM_LOG = "/tmp/p1p2_sim.log"
STACK_LOG = "/tmp/p1p2_stack.log"

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
RESULTS = []

# 真机 ~/.a3/poses.yaml ready（F109）；home 实测 L2≈0/L3≈0，与 URDF 零位一致
# （L4 下垂 0.335 等静态偏置不影响跟踪滞后复现）。sim 起点即 URDF 零位。
READY = [0.0, 1.05, -1.575, 0.0, 0.0, 0.0]


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
        super().__init__("p1p2_acceptance")
        self.js = None
        self.js_stamp = None
        # JTC controller_state carries reference/feedback/error at
        # state_publish_rate; |error.position| is the exact quantity the
        # trajectory-tolerance engine checks against `trajectory:`.
        self.max_track_err = 0.0
        # Only collect tracking error while a trajectory is actually executing:
        # the enable/settle transient (wrapped-anchor re-anchoring in the sim)
        # produces huge |ref-fbk| that is NOT the lag the tolerance engine sees
        # during motion, so it must not pollute C.
        self.sampling = False
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
        # JTC published `error` is a misleading/trajectory-relative quantity;
        # the tolerance engine compares state.position to the sampled reference,
        # so compute the same quantity directly from reference/feedback.
        ref = msg.reference.positions
        fbk = msg.feedback.positions
        n = min(len(ref), len(fbk))
        if self.sampling:
            for i in range(n):
                self.max_track_err = max(self.max_track_err, abs(float(fbk[i]) - float(ref[i])))
        if os.environ.get("A3_P1P2_DUMP"):
            with open("/tmp/p1p2_ts.csv", "a") as f:
                verr = " ".join(f"{float(fbk[i]) - float(ref[i]):+.4f}"
                                for i in range(n))
                vref = " ".join(f"{float(msg.reference.velocities[i]):+.4f}"
                                for i in range(n))
                pref = " ".join(f"{float(ref[i]):+.4f}" for i in range(n))
                f.write(f"{time.time():.3f} P[{pref}] E[{verr}] V[{vref}]\n")

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
        # max_track_err is NOT reset per-run: it accumulates across the A/B
        # sweep so C measures the max over both windows (including, in the COLD
        # 0.05 regression, the pre-abort buildup of A — a mid-run abort leaves
        # B with only a short displacement). Sampling is windowed by
        # self.sampling to keep enable/settle transients out of C.
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
    subprocess.run(["sudo", "-S", "modprobe", "vcan"],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run([f"sudo", "-S", "ip", "link", "del", "dev", VCAN],
                   input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "add", "dev", VCAN, "type", "vcan"],
        input=b"temppwd\n", capture_output=True)
    subprocess.run(
        ["sudo", "-S", "ip", "link", "set", VCAN, "up"],
        input=b"temppwd\n", capture_output=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("domain", nargs="?", default="93")
    ap.add_argument("--alpha", type=float, default=0.023,
                    help="vcan motor first-order follow gain producing the "
                         "measured lag MAGNITUDE band: err scales linearly with "
                         "TC=1/(alpha*200). 0.023 -> TC=0.22 s; on the 11.5 s "
                         "two-point spline (JTC max v_ref ~0.35 rad/s) the peak "
                         "|ref-fbk| measures ~0.07-0.08 rad, inside the real "
                         "0.050-0.055 measured band plus margin. The (TC,v) "
                         "split need not equal the real arm: the JTC tolerance "
                         "engine only sees the resulting peak |err|.")
    ap.add_argument("--duration", type=float, default=11.5,
                    help="home<->ready two-point trajectory duration: must be "
                         "long enough that the JTC spline peak reference "
                         "velocity is NOT a strawman. A short 3.5 s run peaks "
                         "~2.2 rad/s = 4.8x the real goto cruise and "
                         "over-stresses the sim, tripping even the fixed 0.15 "
                         "tolerance; at 11.5 s the measured peak is ~0.35 "
                         "rad/s, keeping peak |err| in the measured band.")
    args = ap.parse_args()

    os.environ["ROS_DOMAIN_ID"] = args.domain
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["PYTHONNOUSERSITE"] = "1"

    ensure_vcan()
    env = make_env(args.domain)
    rclpy.init()
    node = HarnessNode()

    sim = ProcessGroup(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", VCAN, "--alpha", str(args.alpha),
         "--alpha-hz", "200"],
        SIM_LOG, env=env)
    time.sleep(1.0)
    stack = ProcessGroup(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={VCAN}",
         "use_mqtt:=false", "use_teleop:=false", "use_rviz:=false"],
        STACK_LOG, env=env)

    try:
        if not node.wait_fresh_js():
            raise RuntimeError("no /joint_states in 40 s")
        node.enable()
        assert node.fjt.wait_for_server(timeout_sec=10)

        # ---- A/B move by the home<->ready DISPLACEMENT Δ from the live
        # anchor, not absolute targets. The sim's cool-start wind-up can land
        # the enable anchor at wrapped ±2π-ish positions (L2 +0.785→-4.831,
        # L4 ±7.88..8.51 across runs) — a sim-only artifact (the real arm's
        # multi-turn counters are zeroed at fold, F91, so it anchors near
        # home). lag err≈TC*v depends only on Δ and duration, so relative
        # motion reproduces the measured 0.050-0.055 rad band regardless.
        anchor = node.arm_positions()
        rw = args.duration + 6.0

        # ---- A. lagged q0 -> q0+Δ (home->ready displacement) completes ----
        # alpha=0.023 @ 200Hz: TC = 0.22 s; the 11.5 s two-point spline's
        # measured max v_ref is ~0.35 rad/s -> peak |err| ≈ TC*v ~0.07 rad:
        # inside the new 0.15 tolerance, but above the old 0.05 (would abort).
        acc, elapsed, code = node.run_trajectory(
            [anchor[i] + READY[i] for i in range(6)],
            args.duration, result_wait_s=rw)
        check("A lagged Δ home->ready accepted/terminated",
              acc, f"code={code} elapsed={elapsed}")
        check("A lagged Δ home->ready SUCCESSFUL",
              acc and code == 0, f"code={code}")

        # ---- B. lagged q0+Δ -> q0 (ready->home) completes ----
        acc, elapsed, code = node.run_trajectory(anchor,
                                                 args.duration, result_wait_s=rw)
        check("B lagged Δ ready->home accepted/terminated",
              acc, f"code={code} elapsed={elapsed}")
        check("B lagged Δ ready->home SUCCESSFUL",
              acc and code == 0, f"code={code}")

        # ---- C. the sim really reproduced the measured lag magnitude ----
        # (live from JTC controller_state, the same quantity tolerance reads)
        e = node.max_track_err
        check("C max |ref-fbk| in 0.050..0.085 rad window",
              0.050 <= e <= 0.085,
              f"observed={e:.3f} 实测真机 0.050..0.055（L2）")
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        stack.terminate()
        sim.terminate()
        node.destroy_node()
        rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== P1P2 tracking-lag acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())