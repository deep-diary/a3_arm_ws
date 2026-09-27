#!/usr/bin/env python3
"""F119 acceptance: R3 任意模式软失能（先回 idle 再失能，不硬断）。

2026-09-26 真机基线缺陷（LL-133 同源）：`_disable_cb` 的「防阻塞前序」对
`mode ∈ BLOCKED_MODES{SERVO, ZERO_TORQUE, GRAVITY_COMP}` 或 busy state 直接拒绝；
servo 测完（mode 还在 SERVO）按 R3 无反应 = 用户以为失能了其实没有。F119 把前序改为
「任何模式都软失能」：先退当前模式（SERVO→stop_servo、TEACH→stop_teach、
ZERO_TORQUE→stop 零力矩、GRAVITY_COMP→stop 重力补偿），spin 等 `_mode` 退出
`disable_mode_exit_timeout_s=2.0`，再走原 SAFE_PARK 回 idle → 断电失能。

验收分 5 个场景（edge_web_sim DOMAIN 63, use_servo:=true；每 S 结束 arm 都到 DISABLED，
下 S 重新使能）：

  S1 从 READY（零位新使能）disable → 状态序含 SAFE_PARK→DISABLED、success。
  S2 从 SERVO（start_servo + 非零 twist）disable → prelude 内 pump 后撤（静默），
     **0.5s 桥转 IDLE 后 prelude 见退出** → SAFE_PARK→DISABLED；重使能后 start_servo
     仍可用；再 disable（在 idle 快路径）→ DISABLED 且无 SAFE_PARK。
  S3 从 TEACH（start_teach 后 ZERO_TORQUE）disable → TEACH 退出 + mode→IDLE +
     SAFE_PARK→DISABLED；F54 自动存档 teach_*.yaml 计数增加（++）。
  S4 从 GRAVITY_COMP（/a3/gravity_compensation/start）disable → success、
     SAFE_PARK→DISABLED。（--skip-gravity 跳过）
  S5 负例：持续非零 twist 使 SERVO 不消 → disable 在 2.0s 超时**拒绝**，success=False、
     message 含 verbatim 「松开输入后再试」+「紧急失能 /a3/motor/reset」、state 保持
     READY（不误伤）；停 pump 后 mode 出 SERVO ≤3s → 再 disable 成功 SAFE_PARK→DISABLED。
  `~/.a3/trajectories/teach_*.yaml`（排除 latest.yaml）计数增加 = F54 自动存档副作用被验证。

前提：外部栈先 launch（DOMAIN 63）edge_web_sim use_servo:=true。本脚本只设
ROS_DOMAIN_ID + PYTHONNOUSERSITE（不强制 RMW，两侧默认 fastrtps 一致）。

  python3 scripts/a3_test/f119_r3_soft_disable_acceptance.py DOMAIN [--skip-gravity]

Exit code 0 = all checks passed.
"""

import argparse
import glob
import os
import sys
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from a3_msgs.msg import ArmStatus

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
RESULTS = []
STATE_READY = "READY"
STATE_TEACH = "TEACH"
STATE_SAFE_PARK = "SAFE_PARK"
STATE_DISABLED = "DISABLED"
MODE_SERVO = "SERVO"
MODE_IDLE = "IDLE"
MODE_ZERO_TORQUE = "ZERO_TORQUE"
MODE_GRAVITY_COMP = "GRAVITY_COMP"


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


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
    """50Hz 持续发 /servo_node/delta_twist_cmds 的 helper（零 twist 帧也发，保证 stall
    看门狗 + 0.5s 静默转 IDLE 的语义都由 pump 显式控制：stop()=完全静默，set(0)=保持
    帧但零值）。rclpy 从 helper 线程 publish 安全；只有主线程 spin。"""

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
        super().__init__("f119_acceptance")
        self.js = None
        self.status = None
        self.status_samples = []
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_status, 10)
        self.en = self.create_client(Trigger, "/a3/arm/enable")
        self.dis = self.create_client(Trigger, "/a3/arm/disable")
        self.start_servo = self.create_client(Trigger, "/servo_node/start_servo")
        self.stop_servo = self.create_client(Trigger, "/servo_node/stop_servo")
        self.start_teach = self.create_client(Trigger, "/a3/arm/start_teach")
        self.grav_start = self.create_client(Trigger, "/a3/gravity_compensation/start")
        self.fjt = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

    def _on_js(self, msg):
        self.js = msg

    def _on_status(self, msg):
        self.status = msg
        self.status_samples.append((time.monotonic(), msg.state, msg.mode))

    def joint_pos(self, name):
        js = self.js
        if js is None:
            return None
        return float(js.position[js.name.index(name)])

    def wait_status(self, timeout=15.0):
        end = time.monotonic() + timeout
        while self.status is None and time.monotonic() < end:
            spin(self, 0.05)
        return self.status

    def wait_state(self, state, timeout=15.0):
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
        """等 mode 连续稳定 hold_s（20Hz 采样）。S4/S5 关键：off-idle FJT 的
        TRAJ_RUNNING→IDLE 清位有 gravity_torque_node 的名义 end_time(+0.2s) 尾巴，
        a3_fjt_action 的 early-result IDLE 会先到、再被 TRAJ_RUNNING 重新追上（30ms
        竞态）。单次 wait_mode(IDLE) 看到的是第一个 IDLE（此时 grav_start 仍会被
        TRAJ_RUNNING 拒绝）。稳定窗 = 越过 gravity 名义 end_time 后才放行。"""
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
        # 使能后停 0.5s：让 arm_monitor 的「none->all 使能沿重基准」先完成，再发轨迹。
        # 否则 enable→off_idle 的 FJT 落在重基准之前（20Hz tick 滞后 ~50-100ms），
        # _maybe_rebaseline 会抢占 _traj/_last_goal → FJT 全程运动被判 HOLD_DRIFT
        #（ladder stop→reset 会把 sim 电机 reset → arm 自发 DISABLED）。
        spin(self, 0.5)

    def fjt_l2_040(self, duration_s=3.0, result_wait_s=6.0):
        """off-idle：L2→+0.4，其余 5 关节取当前 /joint_states 值（a3_fjt_action 只对
        给的 joint_names 算 err，缺席名字按 0.0 → 超时 abort；故必须全 6 关节构图）。"""
        q0 = [self.joint_pos(n) for n in ARM_JOINTS]
        if any(v is None for v in q0):
            raise RuntimeError("no /joint_states for FJT")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)

        def dur(s):
            return Duration(sec=int(s), nanosec=int((s % 1) * 1e9))

        end = list(q0)
        end[1] = 0.4  # L2_joint
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


def wait_future(h, fut, status_timeout=40.0):
    """disable 是 blocking 服务 → call_async + 主线程 spin 等 done，同时持续采集状态序。
    调用方须在 call_async 前 `h.status_samples = []` 以只含本窗口采样。"""
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < status_timeout:
        spin(h, 0.05)
    done = fut.done()
    resp = fut.result() if done else None
    return done, resp, list(h.status_samples)


def states(samples):
    return [s[1] for s in samples]


def modes(samples):
    return [s[2] for s in samples]


def teach_count():
    pat = os.path.expanduser("~/.a3/trajectories/teach_*.yaml")
    files = [p for p in glob.glob(pat) if "latest" not in os.path.basename(p)]
    return len(files)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("domain", nargs="?", default="63")
    ap.add_argument("--skip-gravity", action="store_true",
                    help="S4 需要 gravity_torque_node 发布 GRAVITY_COMP；无此组件可跳过")
    args = ap.parse_args()

    os.environ["ROS_DOMAIN_ID"] = args.domain
    os.environ["PYTHONNOUSERSITE"] = "1"

    rclpy.init()
    h = Harness()
    pump = TwistPump(h)

    def ready():
        h.enable()
        if not h.wait_state(STATE_READY, timeout=15):
            raise RuntimeError("arm not READY")

    def off_idle():
        acc, _, code = h.fjt_l2_040()
        if not acc or code != 0:
            raise RuntimeError(f"off-idle FJT failed acc={acc} code={code}")
        # 轨迹 end_time 处 gravity 的 TRAJ_RUNNING→IDLE 清位滞后：
        # a3_fjt_action early-result 先写 IDLE，gravity 名义 end_time(+0.2s) 再追上
        # 一个 TRAJ_RUNNING（30ms 竞态）。单次 wait_mode(IDLE) 会看到首个 IDLE 后立即
        # 放行 → grav_start 的 gate 仍被 TRAJ_RUNNING 拒绝（=S4 原始失败）。故要稳定窗。
        if not h.wait_mode_stable(MODE_IDLE, hold_s=1.0, timeout=8.0):
            raise RuntimeError(
                f"mode not IDLE-stable after off-idle FJT (mode={h.status.mode})")

    def settle_after_disable():
        """场景间停 3.0s：让 arm_monitor 的 UNEXPECTED_DISABLE 闩锁（由 disable 的
        /a3/motor/reset 触发：report 需已 DISABLED 0.5s，恢复约+2.1s）在「已 DISABLED」
        期间排空，随后使能才不会被 watchdog 消费在 READY 上打断。真机 R3→重使能间隔秒级，
        sim 压缩到 ~50ms，此开关正是这条 sim 时序墙（LL-132/133 记录，非产品缺陷）。"""
        spin(h, 3.0)

    def start_servo():
        assert h.start_servo.wait_for_service(timeout_sec=10)
        resp = call(h, h.start_servo, Trigger.Request(), timeout=10)
        if not resp.success:
            raise RuntimeError(f"start_servo rejected: {resp.message}")

    def start_servo_mode():
        """pump 非零 twist → bridge 转 SERVO（须 pump 先 start + 非零值）。"""
        pump.start(0.05)
        if not h.wait_mode(MODE_SERVO, timeout=3.0):
            raise RuntimeError(
                f"servo mode not engaged (mode={h.status.mode}); "
                "servo 桥不消费 /servo_node/delta_twist_cmds？")

    # ============================ S1: 从 READY disable ===========================
    ready()
    h.status_samples = []
    done, resp, samples = wait_future(h, h.dis.call_async(Trigger.Request()))
    check("S1 disable service returned", done, "")
    if done:
        check("S1 disable success=True", resp.success, f"msg={resp.message}")
        st = states(samples)
        check("S1 状态序含 SAFE_PARK→DISABLED",
              STATE_SAFE_PARK in st and st and st[-1] == STATE_DISABLED,
              f"states={st}")
    settle_after_disable()

    # ============================ S2: 从 SERVO disable ===========================
    ready()
    off_idle()
    start_servo()
    start_servo_mode()          # mode 确认在 SERVO（pump 非零）
    h.status_samples = []
    fut = h.dis.call_async(Trigger.Request())
    spin(h, 0.3)                # prelude 已进 stop_servo + mode 退出 spin-wait
    pump.stop()                 # 静默 → servo_mode_bridge ~0.5s 后 IDLE → prelude 见退出
    done, resp, samples = wait_future(h, fut)
    check("S2 disable(从 SERVO) service returned", done, "")
    if done:
        check("S2 success=True", resp.success, f"msg={resp.message}")
        st = states(samples)
        check("S2 mode 出 SERVO 且 SAFE_PARK→DISABLED",
              MODE_IDLE in modes(samples)
              and STATE_SAFE_PARK in st and st[-1] == STATE_DISABLED,
              f"states={st} modes={modes(samples)}")
    # 重使能前先让 monitor 的 UNEXPECTED_DISABLE 闩锁排空（见 settle_after_disable）
    settle_after_disable()
    # 重使能后 start_servo 仍可用
    ready()
    start_servo()
    spin(h, 0.5)
    check("S2 重使能后 start_servo 仍可用", True, "start_servo second call succeeded")
    call(h, h.stop_servo, Trigger.Request(), timeout=10)
    pump.stop()
    if not h.wait_mode(MODE_IDLE, timeout=3.0):
        check("S2 stop_servo→mode IDLE", False, f"mode={h.status.mode}")
    else:
        check("S2 stop_servo→mode IDLE", True, "")
    h.status_samples = []
    done, resp, samples = wait_future(h, h.dis.call_async(Trigger.Request()))
    check("S2 disable(在 idle 快路径) service returned", done, "")
    if done:
        check("S2 idle 快路径 success=True", resp.success, f"msg={resp.message}")
        st = states(samples)
        check("S2 idle 快路径无 SAFE_PARK（直接 DISABLED）",
              STATE_SAFE_PARK not in st and st[-1] == STATE_DISABLED,
              f"states={st}")
    settle_after_disable()

    # ============================ S3: 从 TEACH disable ===========================
    ready()
    off_idle()
    n0 = teach_count()
    assert h.start_teach.wait_for_service(timeout_sec=10)
    resp = call(h, h.start_teach, Trigger.Request(), timeout=10)
    check("S3 start_teach success", resp.success, f"msg={resp.message}")
    teach_ok = h.wait_state(STATE_TEACH, timeout=5.0) and h.wait_mode(
        MODE_ZERO_TORQUE, timeout=5.0)
    check("S3 进入 TEACH 且 mode=ZERO_TORQUE", teach_ok,
          f"state={h.status.state} mode={h.status.mode}")
    spin(h, 2.0)                # hold 在 TEACH（记录多个 ZERO_TORQUE 采样）
    h.status_samples = []
    done, resp, samples = wait_future(h, h.dis.call_async(Trigger.Request()))
    check("S3 disable(从 TEACH) service returned", done, "")
    if done:
        check("S3 success=True", resp.success, f"msg={resp.message}")
        st = states(samples)
        ms = modes(samples)
        check("S3 TEACH 退出 + mode→IDLE + SAFE_PARK→DISABLED",
              STATE_TEACH in st and MODE_IDLE in ms
              and STATE_SAFE_PARK in st and st[-1] == STATE_DISABLED,
              f"states={st} modes={ms}")
    n1 = teach_count()
    check("S3 F54 自动存档 teach_*.yaml 计数增加", n1 > n0, f"{n0} -> {n1}")
    settle_after_disable()

    # ============================ S4: 从 GRAVITY_COMP disable ====================
    if args.skip_gravity:
        check("S4 gravity （--skip-gravity 跳过）", True, "跳过")
    else:
        ready()
        off_idle()
        assert h.grav_start.wait_for_service(timeout_sec=10)
        resp = call(h, h.grav_start, Trigger.Request(), timeout=10)
        check("S4 gravity start success", resp.success, f"msg={resp.message}")
        gok = h.wait_mode(MODE_GRAVITY_COMP, timeout=5.0)
        check("S4 mode=GRAVITY_COMP", gok, f"mode={h.status.mode}")
        h.status_samples = []
        done, resp, samples = wait_future(h, h.dis.call_async(Trigger.Request()))
        check("S4 disable(从 GRAVITY_COMP) service returned", done, "")
        if done:
            check("S4 success=True", resp.success, f"msg={resp.message}")
            st = states(samples)
            check("S4 SAFE_PARK→DISABLED",
                  STATE_SAFE_PARK in st and st[-1] == STATE_DISABLED,
                  f"states={st}")
        settle_after_disable()

    # ============================ S5: SERVO 不消 → 拒绝 ==========================
    ready()
    off_idle()
    start_servo()
    start_servo_mode()          # pump 非零
    h.status_samples = []
    fut = h.dis.call_async(Trigger.Request())
    # 保持 pump 持续非零（不断帧）→ bridge 永远 SERVO → prelude 2.0s 超时拒绝
    done, resp, samples = wait_future(h, fut, status_timeout=12.0)
    check("S5 disable(持续 twist) service returned", done, "")
    if done:
        st = states(samples)
        check("S5 拒绝：success=False", not resp.success, f"msg={resp.message}")
        check("S5 message 含「松开输入后再试」", "松开输入后再试" in resp.message,
              f"msg={resp.message}")
        check("S5 message 含「紧急失能 /a3/motor/reset」", "紧急失能 /a3/motor/reset" in resp.message,
              f"msg={resp.message}")
        check("S5 state 未误伤（无 SAFE_PARK/DISABLED，保持 READY）",
              STATE_SAFE_PARK not in st and STATE_DISABLED not in st,
              f"states={st}")
    # 恢复：停 pump → servo 桥 0.5s 后 IDLE → 再 disable 成功
    pump.stop()
    m_idle = h.wait_mode(MODE_IDLE, timeout=3.0)
    check("S5 停 pump 后 mode 出 SERVO ≤3s", m_idle, f"mode={h.status.mode}")
    h.status_samples = []
    done, resp, samples = wait_future(h, h.dis.call_async(Trigger.Request()))
    check("S5 恢复后 disable service returned", done, "")
    if done:
        st = states(samples)
        check("S5 恢复后 success=True 且 SAFE_PARK→DISABLED",
              resp.success and STATE_SAFE_PARK in st
              and st[-1] == STATE_DISABLED,
              f"states={st} msg={resp.message}")

    pump.stop()
    h.destroy_node()
    rclpy.shutdown()

    total = len(RESULTS)
    passed = sum(1 for _, ok in RESULTS if ok)
    print(f"\n==== F119 R3 soft-disable acceptance: {passed}/{total} ====", flush=True)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())