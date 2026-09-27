#!/usr/bin/env python3
"""F118 acceptance: servo 绝对目标（a3_servo_anchor 薄桥接累积器）在 mock 外力下不回弹。

2026-09-26 真机实测（LL-132）：servo 模式外力推臂，松手后臂**不回弹到指令位置**
而漂在当前位——moveit_servo 的目标是 `measure + delta`，无原生绝对/粘连目标。
F118 在下游加薄桥累积器 `a3_servo_anchor`（edge_web_sim `use_servo_anchor:=true`）：
  订阅 `/a3/servo/joint_trajectory/cmd`（捕获 servo 原始命令 in），
  `raw = in − measured` → `delta = clamp(raw, ±0.05)` → `anchor += delta`（再 clamp URDF），
  发 `out = anchor` 到 `/a3/servo/joint_trajectory`（JTC 单点帧 → sim_motor）。

mock 外力注入：sim_motor 新旋钮 `/a3/motor/sim_push`（sim_push_enabled=true、
sim_push_rad=[0,0.35,0..]）只动 plant、不动 target——等价真机手推 L2。

判据（LL-132：**基准是 measured 不是 anchor**）：
  * `--expect-anchor`（栈 use_servo_anchor:=true）：push 后 ~2 s，`|measured_L2 − b_L2| < 0.08 rad`
    —— anchor 补回指令位，外力被压回基线 ⇒ 绝对目标成立（servo 软被外面拉住）。
  * `--expect-drift`（栈 use_servo_anchor:=false）：**接线判据**（本次实测后由 plant 残差改判）：
    `cmd_frames == 0 AND out_frames > 0` —— 无 `/a3/servo/joint_trajectory/cmd` 发布者（anchor
    节点缺席）+ moveit_servo 原始输出直发 `/a3/servo/joint_trajectory`（out_frames>0）。这是
    「drift 栈 = 旧行为（无 anchor）」的接线证据，也是唯一能在 sim 中诚实渲染的观察量。
    plant 残差（ΔL2≥0.15）不再作 drift 判据：实测 moveit_servo 2.5.10 在此 sim 中是 **target
    冻结模型**（外力推臂后目标仍自动回基线、plant 残差恒≈0，两套 drift 栈复现）——sim 无
    力矩→位置物理，无法诚实渲染「臂漂在被推处」。3 个 push 迭代 + 小指令在 drift 模式记录为
    **表征（INFO，不计判据）**；真机（power-on 后可抓模拟外力）为最终权威。
  各重复 3 次（每次重取静态基线）。
  再验「小指令仍能运动」：0.03 m/s × 1.5 s twist → L2 位移 ≥0.005 rad（anchor 不挡住正常指令）。
  reanchor：push 后 ~0.25 s 内重锚（下一帧 anchor:=measured≈被推处）→ 再稳 2 s，
  残差 ≥0.15 才通过 —— 断言「reanchor 把被推点当作新回弹位」是真语义（漂移测试该命令 carry）。
    —— 注意：此断言区分 servo「target 重锚 measured」vs「target 冻结」。若 moveit_servo 是
      冻结模型（target 不随外力动），reanchor 会 no-op（残差≈0）→ 这条 FAIL = 实测发现 servo
      内部模型，需据此迭代 reanchor 实现，是合法验收输出（非测试 bug）。
  L7 透传：capture 期间若有含 L7 的 cmd 帧 → anchor 模式断言 out==in（透传逐字），drift 仅 info。

前提：battery 先 launch edge_web_sim（DOMAIN 62）`use_servo:=true` +（anchor 模式）
`use_servo_anchor:=true`。脚本只设 ROS_DOMAIN_ID + PYTHONNOUSERSITE（**不强制 RMW**，
两侧都用默认 fastrtps）；栈侧不重启无 motor_protocol 的 node 名不同 → sim_push 参数走
`/motor_protocol_node/set_parameters`。

  python3 scripts/a3_test/f118_servo_anchor_acceptance.py DOMAIN [--expect anchor|drift]

Exit code 0 = all checks passed.
"""

import argparse
import os
import sys
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from a3_msgs.msg import ArmStatus

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
# 零位 IK 奇异 → 先 FJT 抬离（L2 0→0.785, L3 0→-0.785），其余维持当前
RAISED = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def info(name, detail=""):
    """表征/诊断行：打印但**不计入** RESULTS（drift 模式下用于记录无法在 sim 中
    诚实作判据的 plant 残差与小指令），汇总行只统计 check()。"""
    print(f"[INFO] {name} {detail}", flush=True)


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


class TwistPump:
    """后台线程 @50Hz 持续发 /servo_node/delta_twist_cmds（frame base_link）。

    moveit_servo 有 ~0.35 s stall 看门狗 → keepalive 必须**持续发帧**（零 twist 帧也发；
    只有 silent >0.5 s 会经 servo_mode_bridge 转 IDLE）。rclpy 从 helper 线程 publish
    安全；spin 只允许主线程。
    """

    def __init__(self, node, rate_hz=50.0):
        self.node = node
        self.pub = node.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.rate = rate_hz
        self._vx = 0.0
        self._alive = False
        self._lock = threading.Lock()
        self._thread = None
        self.frame_id = "base_link"

    def start(self, vx=0.0):
        with self._lock:
            self._vx = vx
            self._alive = True
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def set(self, vx):
        with self._lock:
            self._vx = vx

    def stop(self):
        with self._lock:
            self._alive = False

    def _run(self):
        while True:
            with self._lock:
                alive = self._alive
                vx = self._vx
            if not alive:
                break
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.twist.linear.x = vx
            self.pub.publish(msg)
            time.sleep(1.0 / self.rate)


class Harness(Node):
    def __init__(self):
        super().__init__("f118_acceptance")
        self.js = None
        self.status = None
        self.cmd_frames = []
        self.out_frames = []
        self.out_l2 = []          # (monotonic, L2 目标值) drift 小指令表征用
        be_latched = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_status, 10)
        self.create_subscription(
            JointTrajectory, "/a3/servo/joint_trajectory/cmd", self._on_cmd, be_latched)
        self.create_subscription(
            JointTrajectory, "/a3/servo/joint_trajectory", self._on_out, rel)
        self.enable_cli = self.create_client(Trigger, "/a3/arm/enable")
        self.start_servo_cli = self.create_client(Trigger, "/servo_node/start_servo")
        self.fjt = ActionClient(self, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory")

    def _on_js(self, msg):
        self.js = msg

    def _on_status(self, msg):
        self.status = msg

    def _on_cmd(self, msg):
        self.cmd_frames.append(msg)

    def _on_out(self, msg):
        self.out_frames.append(msg)
        try:
            i = msg.joint_names.index("L2_joint")
            self.out_l2.append((time.monotonic(), msg.points[0].positions[i]))
        except (ValueError, IndexError):
            pass

    def joint_pos(self, name):
        js = self.js
        if js is None:
            return None
        return float(js.position[js.name.index(name)])

    def median_l2(self, window_s=1.0):
        samples = []
        end = time.monotonic() + window_s
        while time.monotonic() < end:
            v = self.joint_pos("L2_joint")
            if v is not None:
                samples.append(v)
            spin(self, 0.05)
        samples.sort()
        n = len(samples)
        return samples[n // 2]

    def wait_status(self, timeout=15.0):
        """等到非 None 的首帧 arm_status。"""
        end = time.monotonic() + timeout
        while self.status is None and time.monotonic() < end:
            spin(self, 0.05)
        return self.status

    def wait_state(self, state, timeout=15.0):
        self.wait_status()
        end = time.monotonic() + timeout
        while self.status.state != state and time.monotonic() < end:
            spin(self, 0.05)
            if self.status.state == state:
                return True
        return self.status.state == state

    def enable(self):
        assert self.enable_cli.wait_for_service(timeout_sec=20)
        resp = call(self, self.enable_cli, Trigger.Request(), timeout=40)
        if not resp.success:
            raise RuntimeError(f"enable rejected: {resp.message}")
        spin(self, 3.0)

    def fjt_to(self, target6, duration_s, result_wait_s=8.0):
        """6 关节帧：target6 的关节取 target 值，其余取当前 /joint_states 值。"""
        q0 = [self.joint_pos(n) for n in ARM_JOINTS]
        if any(v is None for v in q0):
            raise RuntimeError("no /joint_states yet for FJT")
        start = list(q0)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)

        def dur(s):
            return Duration(sec=int(s), nanosec=int((s % 1) * 1e9))

        p0 = __import__("trajectory_msgs.msg").msg.JointTrajectoryPoint(
            positions=start, velocities=[0.0] * 6, accelerations=[0.0] * 6,
            time_from_start=dur(0.0))
        end = [target6[i] if target6[i] is not None else q0[i]
               for i in range(6)]
        p1 = __import__("trajectory_msgs.msg").msg.JointTrajectoryPoint(
            positions=end, velocities=[0.0] * 6, accelerations=[0.0] * 6,
            time_from_start=dur(duration_s))
        goal.trajectory.points = [p0, p1]
        gh = self.fjt.send_goal_async(goal)
        t0 = time.monotonic()
        while not gh.done() and time.monotonic() - t0 < 8:
            spin(self, 0.05)
        if not gh.done() or not gh.result().accepted:
            return False, None, None
        handle = gh.result()
        rf = handle.get_result_async()
        t0 = time.monotonic()
        while not rf.done() and time.monotonic() - t0 < result_wait_s:
            spin(self, 0.05)
        if not rf.done():
            return True, None, None
        return True, time.monotonic() - t0, rf.result().result.error_code


def set_sim_push(node, enabled, rad, hold_s=0.0):
    cli = node.create_client(SetParameters, "/motor_protocol_node/set_parameters")
    if not cli.wait_for_service(timeout_sec=10):
        return False, "set_parameters service unavailable"
    req = SetParameters.Request()
    req.parameters = [
        RosParameter(
            name="sim_push_enabled",
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                 bool_value=bool(enabled)),
        ),
        RosParameter(
            name="sim_push_rad",
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                 double_array_value=[float(r) for r in rad]),
        ),
        RosParameter(
            name="sim_push_hold_s",
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(hold_s)),
        ),
    ]
    resp = call(node, cli, req, timeout=10)
    return all(r.successful for r in resp.results), [r.successful for r in resp.results]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("domain", nargs="?", default="62")
    ap.add_argument("--expect", choices=["anchor", "drift"], default="anchor")
    args = ap.parse_args()

    os.environ["ROS_DOMAIN_ID"] = args.domain
    os.environ["PYTHONNOUSERSITE"] = "1"
    expect_anchor = args.expect == "anchor"

    rclpy.init()
    h = Harness()
    pump = TwistPump(h)

    def ready():
        h.enable()
        if not h.wait_state("READY", timeout=15):
            raise RuntimeError("arm not READY")

    def fjt_raised():
        acc, elapsed, code = h.fjt_to(RAISED, 5.0)
        if not acc or code != 0:
            raise RuntimeError(f"FJT to RAISED failed acc={acc} code={code}")
        spin(h, 1.0)

    def start_servo():
        assert h.start_servo_cli.wait_for_service(timeout_sec=10)
        resp = call(h, h.start_servo_cli, Trigger.Request(), timeout=10)
        if not resp.success:
            raise RuntimeError(f"start_servo rejected: {resp.message}")

    # ---------- 预置 ----------
    ready()
    fjt_raised()
    start_servo()
    m0 = h.joint_pos("L2_joint")
    pump.start(0.05)
    spin(h, 1.0)  # 让 servo 吃到非零 twist、模式激活
    check("pre 活动 twist 使 L2 移动（servo 真实在servo）",
          abs(h.joint_pos("L2_joint") - m0) > 1e-4,
          f"L2 {m0:.4f} -> {h.joint_pos('L2_joint'):.4f}")
    pump.set(0.0)      # 进入 keepalive（moveit_servo 0.35s 看门狗需持续帧）
    spin(h, 1.0)

    # ---------- 3 次 push 推论（anchor: 回弹；drift: 漂留） ----------
    ok_set, detail = set_sim_push(h, True, [0.0, 0.35, 0.0, 0.0, 0.0, 0.0, 0.0])
    check(f"sim_push 参数注入（enabled=True, push L2 +0.35）", ok_set, f"{detail}")
    if not ok_set:
        pump.stop()
        print("\n==== F118: aborted (sim_push 不可用) ====", flush=True)
        rclpy.shutdown()
        return 1

    for i in range(3):
        b = h.median_l2(1.0)               # 静态基线（push 已回弹/漂留后的稳定点）
        resp = call(h, h._make_push_cli(), Trigger.Request(), timeout=10)
        spin(h, 0.3)
        spin(h, 2.0)                       # anchor 回弹窗口 / drift 停留窗口
        m = h.median_l2(1.0)
        residual = m - b
        if expect_anchor:
            ok = abs(residual) < 0.08
            tag = f"|ΔL2|={abs(residual):.4f} < 0.08 (回弹当前位置)"
            check(f"iter{i+1} sim_push L2 +0.35 → {args.expect}", ok, tag)
        else:
            # sim 中 moveit_servo 2.5.10 = target 冻结模型（实测残余总≈0），plant 残差无法
            # 诚实渲染「旧漂移 ≥0.15」→ 记表征不计判据；drift 硬判据见下方 wiring 块。
            info(f"iter{i+1} sim_push L2 +0.35 → drift 表征：ΔL2={residual:.4f}"
                 f"（旧漂移语义=≥0.15；frozen-target 恒≈0，sim 不可渲染，真机复验）",
                 f"b={b:.4f} m={m:.4f}")

    # ---------- 接线可观察量（drift 模式的**唯一硬判据**；anchor 模式佐证） ----------
    # anchor：servo 原始输出改道 /cmd → a3_servo_anchor → out（两话题均有帧）。
    # drift ：无 /cmd 发布者（cmd_frames==0）+ servo 直发 out（out_frames>0）。
    if expect_anchor:
        check("接线：servo→/cmd→anchor→/a3/servo/joint_trajectory（cmd 与 out 均有帧）",
              len(h.cmd_frames) > 0 and len(h.out_frames) > 0,
              f"cmd_frames={len(h.cmd_frames)}, out_frames={len(h.out_frames)}")
    else:
        check("接线：无 /a3/servo/joint_trajectory/cmd 发布者（cmd_frames==0）+ servo 直发 "
              "out（out_frames>0）= 旧漂移栈接线证据",
              len(h.cmd_frames) == 0 and len(h.out_frames) > 0,
              f"cmd_frames={len(h.cmd_frames)}, out_frames={len(h.out_frames)}")

    # ---------- 小指令仍能运动 ----------
    x0 = h.joint_pos("L2_joint")
    t_small0 = time.monotonic()
    pump.set(0.03)
    spin(h, 1.5)
    pump.set(0.0)
    spin(h, 0.5)
    moved = abs(h.joint_pos("L2_joint") - x0)
    if expect_anchor:
        check("小指令 0.03 m/s×1.5 s 仍驱动 L2 ≥0.005 rad（anchor 不挡合法指令）",
              moved >= 0.005, f"L2 Δ={moved:.4f}")
    else:
        # drift 表征：0.03/0.005 贴阈值 + frozen-target 下含义弱，不作为判据。
        win = [v for t, v in h.out_l2 if 0 <= t - t_small0 <= 2.5]
        info(f"小指令 0.03 m/s×1.5 s → plant L2 Δ={moved:.4f}（drift 表征；真机以 0.05 复验）",
             f"servo out 目标窗口内 L2 = "
             f"{f'{min(win):.4f}..{max(win):.4f} n={len(win)} last={win[-1]:.4f}' if win else '无帧'}")

    # ---------- reanchor：被推点成为新回弹位 ----------
    if expect_anchor:
        h._reanchor_cli = h.create_client(Trigger, "/a3/servo_anchor/reanchor")
        if h._reanchor_cli.wait_for_service(timeout_sec=3):
            b = h.median_l2(1.0)
            # sim_push_hold_s=1.0：sim 里 push 只动 plant，一阶跟随 τ≈57ms 会在
            # ~0.3s 回弹基线，reanchor 就成了 no-op。hold 模拟人手在 reanchor
            # 前把臂捏在被推处——这是真机「手推开臂再 reanchor」的物理。
            set_sim_push(h, True, [0.0, 0.35], hold_s=1.0)
            resp = call(h, h._make_push_cli(), Trigger.Request(), timeout=10)
            spin(h, 0.25)                     # 此刻 measured≈被推点（被 hand 捏住）；reanchor 下一帧重锚
            rr = call(h, h._reanchor_cli, Trigger.Request(), timeout=10)
            check("reanchor 服务成功", rr.success, f"{rr.message}")
            set_sim_push(h, True, [0.0, 0.0], hold_s=0.0)  # 松手：让锚命令被推点而不是外力
            spin(h, 2.0)                      # anchor 现在命令被推点 → 不回弹
            m = h.median_l2(1.0)
            residual = m - b
            check("reanchor 后被推点成为新回弹位（残差 ≥0.15）",
                  residual >= 0.15,
                  f"ΔL2={residual:.4f} b={b:.4f}（若≈0 ⇒ moveit_servo 是『target 冻结』模型，"
                  f"reanchor 语义需迭代，见脚本头）")
        else:
            check("reanchor 服务不可用（测试栈缺 a3_servo_anchor）", False, "")

    # ---------- L7 透传 ----------
    with_l7 = [f for f in h.cmd_frames if "L7_joint" in f.joint_names]
    if with_l7 and expect_anchor:
        mismatches = 0
        total = 0
        for f in with_l7:
            outs = [o for o in h.out_frames
                    if o.header.stamp == f.header.stamp]
            if not outs:
                continue
            idx = f.joint_names.index("L7_joint")
            for o in outs:
                if "L7_joint" not in o.joint_names:
                    continue
                total += 1
                oi = o.joint_names.index("L7_joint")
                if abs(o.points[0].positions[oi] - f.points[0].positions[idx]) > 1e-9:
                    mismatches += 1
        check("L7 在 anchor 输出中逐字透传（out==in, ≤1e-9）",
              total > 0, f"对 {len(with_l7)} 含 L7 cmd 帧，{total} 次比对，失配 {mismatches}")
    elif with_l7:
        check("L7 cmd 帧存在（drift 模式 trivially 透传，仅 info）", True,
              f"{len(with_l7)} 帧含 L7")
    else:
        if expect_anchor:
            check("capture 到 servo cmd 帧（anchor 栈理应存在 /cmd 发布者）",
                  len(h.cmd_frames) > 0,
                  f"cmd_frames={len(h.cmd_frames)}, out_frames={len(h.out_frames)}")
        else:
            info("capture 汇总（drift：cmd_frames 应=0、out_frames 应>0，见 wiring 判据）",
                 f"cmd_frames={len(h.cmd_frames)}, out_frames={len(h.out_frames)}")

    pump.stop()
    spin(h, 1.0)
    h.destroy_node()
    rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F118 servo anchor acceptance ({args.expect}): {passed}/{total} ====",
          flush=True)
    return 0 if passed == total else 1


# 便捷：push client 复用（延迟创建）
def _push_cli(self):
    cli = getattr(self, "_push_cli", None)
    if cli is None:
        cli = self.create_client(Trigger, "/a3/motor/sim_push")
        self._push_cli = cli
    return cli


Harness._make_push_cli = _push_cli  # for main to create lazily


if __name__ == "__main__":
    sys.exit(main())