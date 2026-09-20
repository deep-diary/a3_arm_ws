#!/usr/bin/env python3
"""F62 合成 /joy 全功能验证（纯仿真，禁真机）。

50Hz 持续发布 sensor_msgs/Joy（严格按 config/ds4_linux.yaml：8 axes / 14 buttons，
扳机 rest=1.0/pressed=-1.0），逐场景模拟 F60 键位，断言：
  编排层状态(/a3/arm_status)、电源门禁(/power_sequence/*)、灯态/震动
  (/a3/ds4/feedback，变化时发布，故用滚动历史)、关节(/joint_states BEST_EFFORT)、
  夹爪(/a3/gripper_status)、末端 TF(base_link→end_effector)、示教文件落盘。

前置：
  source scripts/a3_shell_env.sh
  export ROS_DOMAIN_ID=45
  ros2 launch a3_bringup edge_teleop_full_sim.launch.py   # 另一终端
  python3 scripts/a3_test/ps4_sim_test.py

真机勿跑（会主动下发 power/enable/goto/playback）。
"""

import json
import math
import os
import sys
import time
from collections import deque

import rclpy
from a3_msgs.msg import ArmStatus, GripperStatus
from geometry_msgs.msg import TransformStamped  # noqa: F401  (tf2 反序列化需要)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Reporter  # noqa: E402

# ---- ds4_linux.yaml 布局 ----
B_CROSS, B_CIRCLE, B_TRIANGLE, B_SQUARE = 0, 1, 2, 3
B_L1, B_R1, B_L2, B_R2 = 4, 5, 6, 7
B_SHARE, B_OPTIONS, B_PS, B_L3, B_R3, B_TOUCH = 8, 9, 10, 11, 12, 13
N_BTN = 14

AX_LX, AX_LY, AX_L2, AX_RX, AX_RY, AX_R2, AX_DX, AX_DY = range(8)
TRIG_REST, TRIG_PRESSED = 1.0, -1.0

_BUILTIN_READY = [0.0, 0.785, -1.57, 0.0, 0.785, 0.0, 0.0]
_BUILTIN_HOME = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0, 0.0]
POSE_TOL = 0.08


def _load_named_pose(name, builtin):
    """~/.a3/poses.yaml 覆盖包内置位姿（真机标定值；arm_controller 同规则）。"""
    try:
        import yaml
        with open(os.path.expanduser("~/.a3/poses.yaml"), "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        p = (data.get("poses") or {}).get(name)
        if isinstance(p, dict):
            p = p.get("positions")
        if p and len(p) == 7:
            return [float(x) for x in p]
    except Exception:
        pass
    return builtin


READY_POSE = _load_named_pose("ready", _BUILTIN_READY)
HOME_POSE = _load_named_pose("home", _BUILTIN_HOME)

LATEST_YAML = os.path.expanduser("~/.a3/trajectories/latest.yaml")


class Ps4SimTest(Node):
    def __init__(self):
        super().__init__("ps4_sim_test")
        self._axes = [0.0] * 8
        self._axes[AX_L2] = TRIG_REST
        self._axes[AX_R2] = TRIG_REST
        self._btns = [0] * N_BTN

        self._js = None
        self._arm = None
        self._power_state = None
        self._gate = None
        self._grip = None
        self._fb_hist = deque(maxlen=4000)  # (monotonic, dict)

        sensor_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latch_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._joy_pub = self.create_publisher(Joy, "/joy", 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, sensor_qos)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_arm, 10)
        self.create_subscription(String, "/power_sequence/state", self._on_power, latch_qos)
        self.create_subscription(Bool, "/power_sequence/gate_open", self._on_gate, latch_qos)
        self.create_subscription(GripperStatus, "/a3/gripper_status", self._on_grip, 10)
        self.create_subscription(String, "/a3/ds4/feedback", self._on_fb, 10)
        self.create_timer(0.02, self._pub_joy)  # 50Hz

    # ---- joy 发布 ----
    def _pub_joy(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [float(v) for v in self._axes]
        msg.buttons = [int(v) for v in self._btns]
        self._joy_pub.publish(msg)

    def press(self, b):
        self._btns[b] = 1

    def release(self, b):
        self._btns[b] = 0

    def tap(self, b, hold=0.30):
        self.press(b)
        self.sleep(hold)
        self.release(b)
        self.sleep(0.25)

    # ---- 订阅回调 ----
    def _on_js(self, msg):
        self._js = msg

    def _on_arm(self, msg):
        self._arm = msg

    def _on_power(self, msg):
        self._power_state = msg.data

    def _on_gate(self, msg):
        self._gate = bool(msg.data)

    def _on_grip(self, msg):
        self._grip = msg

    def _on_fb(self, msg):
        try:
            self._fb_hist.append((time.monotonic(), json.loads(msg.data)))
        except (ValueError, TypeError):
            pass

    # ---- 工具 ----
    def spin_for(self, dt):
        end = time.monotonic() + dt
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def sleep(self, dt):
        self.spin_for(dt)

    def marker(self):
        return time.monotonic()

    def fb_since(self, t0):
        return [d for t, d in self._fb_hist if t >= t0]

    def joints(self):
        if self._js is None:
            return None
        order = {n: i for i, n in enumerate(self._js.name)}
        try:
            return [self._js.position[order[f"L{i}_joint"]] for i in range(1, 8)]
        except KeyError:
            return None

    def ee_xyz(self):
        # tf2 在 spin 中维护；直接查 buffer
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link", "end_effector", rclpy.time.Time()
            )
            p = tf.transform.translation
            return [p.x, p.y, p.z]
        except Exception:
            return None

    def attach_tf(self):
        import tf2_ros
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def arm_state(self):
        return self._arm.state if self._arm else None

    def arm_msg(self):
        return self._arm.message if self._arm else ""

    def wait_arm(self, states, timeout, msg_prefix=None):
        if isinstance(states, str):
            states = (states,)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self._arm is not None and self._arm.state in states:
                if msg_prefix is None or self._arm.message.startswith(msg_prefix):
                    return True
            self.spin_for(0.05)
        return False

    def wait_pose(self, target, tol, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            q = self.joints()
            if q is not None and max(abs(a - b) for a, b in zip(q, target)) < tol:
                return True
            self.spin_for(0.05)
        return False

    def wait_power(self, state, gate, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self._power_state == state and self._gate == gate:
                return True
            self.spin_for(0.05)
        return False


def pose_err(q, target):
    if q is None:
        return float("inf")
    return max(abs(a - b) for a, b in zip(q, target))


def main():
    rclpy.init()
    node = Ps4SimTest()
    node.attach_tf()
    rep = Reporter("F62 PS4 合成 /joy 全功能（仿真）")

    def fb_any(t0, pred):
        return any(pred(d) for d in node.fb_since(t0))

    # ---- 等待栈就绪 ----
    print("[INFO] 等待仿真栈：/joint_states / arm_status / power latch ...", flush=True)
    boot_deadline = time.monotonic() + 40.0
    while time.monotonic() < boot_deadline:
        if node.joints() is not None and node._arm is not None and node._gate is not None:
            break
        node.spin_for(0.1)
    if node.joints() is None or node._arm is None:
        print("[FAIL] 仿真栈 40s 内未就绪，终止", flush=True)
        sys.exit(2)

    # 全零基线 ≥25 拍（mapper joy_alive_ticks=25 @50Hz），实际给 1.5s
    print("[INFO] 全零基线 1.5s（mapper 攒满 alive ticks，auto-start servo）", flush=True)
    node.sleep(1.5)

    # ---- 场景 0：基线 ----
    rep.info(f"场景0 基线: power={node._power_state} gate={int(node._gate)} "
             f"arm={node.arm_state()}")
    rep.check("S0 电源 Running/gate 开", node.wait_power("Running", True, 3.0),
              f"state={node._power_state} gate={node._gate}")
    rep.check("S0 编排层 IDLE", node.arm_state() == "IDLE", node.arm_state() or "None")
    t0 = node.marker()
    node.sleep(0.8)
    # 反馈仅在 payload 变化时 15Hz 发布；栈先于脚本启动并已稳定时，窗口内可能
    # 一条都收不到——此时按 power/gate/arm 三态确定性推断橙色 idle。
    if node.fb_since(t0):
        orange_idle = fb_any(t0, lambda d: d.get("color") == "orange"
                             and d.get("class") in ("idle", "powering"))
    else:
        orange_idle = (node._power_state == "Running" and node._gate is True
                       and node.arm_state() == "IDLE")
        rep.info("S0 窗口内无 feedback（启动前已稳态），按 Running/gate/IDLE 推断橙灯")
    rep.check("S0 灯态橙色（已上电未使能）", orange_idle)

    # ---- 场景 1：PS init → READY + 白闪 ----
    t0 = node.marker()
    print("[INFO] 场景1: PS 短按（arm_init）", flush=True)
    node.tap(B_PS, 0.30)
    ok_init = node.wait_arm("READY", 12.0)
    q = node.joints()
    rep.check("S1 PS init 后 READY（自动 enable）", ok_init,
              f"state={node.arm_state()} msg={node.arm_msg()!r}")
    white = fb_any(t0, lambda d: d.get("color") == "white")
    rep.check("S1 白闪一次（init 完成）", white)
    # 白闪窗口（white_flash_s=0.6s）内 green 被覆盖，必须等窗口过后才重发绿
    node.sleep(0.9)
    green = fb_any(t0, lambda d: d.get("color") == "green" and d.get("state") == "READY")
    rep.check("S1 随后绿灯 READY", green)

    # ---- 场景 2：Triangle → ready，TRAJ/紫 → READY/绿 ----
    node.sleep(0.5)
    t0 = node.marker()
    print("[INFO] 场景2: Triangle 短按（goto ready）", flush=True)
    node.tap(B_TRIANGLE)
    purple_goto = False
    ok_pose = node.wait_pose(READY_POSE, POSE_TOL, 9.0)
    purple_goto = fb_any(t0, lambda d: d.get("color") == "purple"
                         and "goto ready" in d.get("reason", ""))
    ok_ready = node.wait_arm("READY", 3.0)
    q = node.joints()
    rep.check("S2 收敛 ready 位姿（tol 0.08）", ok_pose,
              f"maxerr={pose_err(q, READY_POSE):.3f} q={['%.2f' % x for x in (q or [])]}")
    rep.check("S2 过程紫灯 TRAJ(goto ready)", purple_goto)
    rep.check("S2 结束回 READY/绿灯", ok_ready and green_steady(node),
              f"state={node.arm_state()}")
    node.sleep(4.0)  # 等 mapper pose_block（响应后锁 3.5s）释放

    # ---- 场景 3：L3 幂等，仍 READY 绿 ----
    t0 = node.marker()
    print("[INFO] 场景3: L3 短按（已 READY，start/enable 幂等）", flush=True)
    node.tap(B_L3)
    node.sleep(2.5)
    no_disturb = not fb_any(t0, lambda d: d.get("class") in
                            ("offline", "fault", "traj"))
    rep.check("S3 L3 幂等：仍 READY 绿灯、无紫/红扰动",
              node.arm_state() == "READY" and no_disturb,
              f"state={node.arm_state()}")

    # ---- 场景 4：R2 不按 L1，夹爪开合 ----
    print("[INFO] 场景4: R2 不按 L1，0→1（1.0Nm 力闭合）保持 7s", flush=True)
    q0 = node.joints()[6]
    t0 = node.marker()
    node._axes[AX_R2] = TRIG_PRESSED
    max_q7 = q0
    grip_force_evidence = False
    end = time.monotonic() + 7.0
    while time.monotonic() < end:
        q = node.joints()
        if q:
            max_q7 = max(max_q7, q[6])
        if node._grip is not None and node._grip.mode == "force":
            grip_force_evidence = True
        node.spin_for(0.1)
    moved_close = (max_q7 - q0) >= 0.30
    rep.check("S4 R2 无 L1 力闭合：L7 位移 ≥0.30rad", moved_close,
              f"q7 {q0:.3f}→{max_q7:.3f} (Δ={max_q7 - q0:.3f})")
    rep.check("S4 /a3/gripper_status mode=force", grip_force_evidence,
              f"grip={None if node._grip is None else node._grip.state + '/' + node._grip.mode}")
    print("[INFO] 场景4: R2 松开（release，1.5s 张开轨迹）", flush=True)
    node._axes[AX_R2] = TRIG_REST
    node.sleep(3.0)
    q_end = node.joints()[6]
    rep.check("S4 松手释放：L7 回到 ≤0.15rad", q_end <= 0.15, f"q7={q_end:.3f}")

    # ---- 场景 5：摇杆死人开关 + 逐轴 ----
    # 必须从 ready 标定位开始：平移扫轴会漂向腕部奇异区，折叠 home 位 servo 直接
    # IK -31 / emergency stop（LL-007 族，实测见 /tmp 栈日志）。
    print("[INFO] 场景5 预备: Triangle 回 ready（逐轴前离开奇异区）", flush=True)
    node.tap(B_TRIANGLE)
    node.wait_pose(READY_POSE, POSE_TOL, 9.0)
    node.wait_arm("READY", 3.0)
    node.sleep(4.0)  # 等 mapper pose_block（goto 后锁 3.5s）释放

    print("[INFO] 场景5a: 不按 L1 推左摇杆 1.2s，必须不动", flush=True)
    q_lock = node.joints()
    node._axes[AX_LX] = 1.0
    node.sleep(1.2)
    node._axes[AX_LX] = 0.0
    d_lock = max(abs(a - b) for a, b in zip(node.joints(), q_lock))
    rep.check("S5a 无 L1 摇杆死锁（关节 Δ<0.03rad）", d_lock < 0.03, f"maxΔ={d_lock:.4f}")
    node.sleep(0.6)

    axis_cases = [
        ("left_x → lin_y", AX_LX, "ee_y"),
        ("left_y → lin_z", AX_LY, "ee_z"),
        ("right_y → lin_x", AX_RY, "ee_x"),
    ]
    signs = {}
    for label, ax, ee_key in axis_cases:
        node.press(B_L1)
        node.sleep(0.2)
        ee0 = node.ee_xyz()
        node._axes[ax] = 1.0
        node.sleep(1.4)
        node._axes[ax] = 0.0
        ee1 = node.ee_xyz()
        node.release(B_L1)
        node.sleep(0.8)
        idx = {"ee_x": 0, "ee_y": 1, "ee_z": 2}[ee_key]
        d = ee1[idx] - ee0[idx] if (ee0 and ee1) else 0.0
        signs[label] = d
        rep.check(f"S5b {label}：{ee_key} 位移 ≥0.01m", abs(d) >= 0.01,
                  f"Δ{ee_key}={d:+.3f}m  ee0={['%.3f' % v for v in (ee0 or [])]}"
                  f"→{['%.3f' % v for v in (ee1 or [])]}")

    # right_x → 偏航：实测 yaw 同时驱动 L1+L5（反向）；三轴平移后构型会漂向奇异，
    # 先回 ready 再测，断言两关节合转动。
    print("[INFO] 场景5b: 偏航前 Triangle 回 ready（平移漂移会导致 yaw IK 奇异）",
          flush=True)
    node.tap(B_TRIANGLE)
    node.wait_pose(READY_POSE, POSE_TOL, 9.0)
    node.wait_arm("READY", 3.0)
    node.sleep(4.0)
    node.press(B_L1)
    node.sleep(0.2)
    q0_yaw = node.joints()
    node._axes[AX_RX] = 1.0
    node.sleep(1.4)
    node._axes[AX_RX] = 0.0
    q1_yaw = node.joints()
    node.release(B_L1)
    node.sleep(0.8)
    dl1 = q1_yaw[0] - q0_yaw[0]
    dl5 = q1_yaw[4] - q0_yaw[4]
    signs["right_x → ang_z(yaw)"] = dl1
    rep.check("S5b right_x → 偏航：L1+L5 合转动 ≥0.05rad",
              abs(dl1) + abs(dl5) >= 0.05,
              f"ΔL1={dl1:+.3f} ΔL5={dl5:+.3f}rad")
    rep.info("方向符号表（轴+1 → 位移，供 invert 标定）: "
             + "; ".join(f"{k}={v:+.3f}" for k, v in signs.items()))

    # 松 L1 停住：0.5s bridge 超时后，0.8s 窗口内不再动
    node.sleep(0.7)
    qh0 = node.joints()
    node.sleep(0.8)
    d_halt = max(abs(a - b) for a, b in zip(node.joints(), qh0))
    rep.check("S5c 松 L1 后 ~1s 停住（Δ<0.03rad）", d_halt < 0.03, f"maxΔ={d_halt:.4f}")

    # R1 加速档：同方向 1.0s，位移应明显大于 0.35 档
    node.press(B_L1)
    node.press(B_R1)
    node.sleep(0.2)
    ee0 = node.ee_xyz()
    node._axes[AX_LX] = 1.0
    node.sleep(1.0)
    node._axes[AX_LX] = 0.0
    ee1 = node.ee_xyz()
    node.release(B_R1)
    node.release(B_L1)
    node.sleep(0.8)
    d_boost = abs(ee1[1] - ee0[1]) if (ee0 and ee1) else 0.0
    d_base = abs(signs["left_x → lin_y"])
    ratio = (d_boost / 1.0) / (d_base / 1.4) if d_base > 1e-6 else 0.0
    rep.check("S5d R1 速度档 1.0（归一速度比 >1.2）", ratio > 1.2,
              f"boost={d_boost:.3f}m/1.0s base={d_base:.3f}m/1.4s ratio={ratio:.2f}")

    # ---- 场景 6：Circle → home ----
    node.sleep(1.0)
    t0 = node.marker()
    print("[INFO] 场景6: Circle 短按（goto home）", flush=True)
    node.tap(B_CIRCLE)
    ok_home = node.wait_pose(HOME_POSE, POSE_TOL, 9.0)
    purple_home = fb_any(t0, lambda d: d.get("color") == "purple"
                         and "goto home" in d.get("reason", ""))
    ok_ready = node.wait_arm("READY", 3.0)
    q = node.joints()
    rep.check("S6 收敛 home 位姿", ok_home,
              f"maxerr={pose_err(q, HOME_POSE):.3f}")
    rep.check("S6 紫灯(goto home)→绿", purple_home and ok_ready and green_steady(node))
    node.sleep(4.0)

    # ---- 场景 7：示教 Share/Options + Square 回放 ----
    t0 = node.marker()
    print("[INFO] 场景7a: Share 短按（start_teach）", flush=True)
    node.tap(B_SHARE)
    ok_teach = node.wait_arm("TEACH", 3.0)
    blue = fb_any(t0, lambda d: d.get("color") == "blue")
    rep.check("S7a Share → TEACH", ok_teach, f"state={node.arm_state()}")
    rep.check("S7a 蓝灯（呼吸）", blue)
    print("[INFO] 场景7a: 静态录制 3.2s（≈160 样本 > 10 门限）", flush=True)
    node.sleep(3.2)
    mtime_before = mtime(LATEST_YAML)
    t0 = node.marker()
    print("[INFO] 场景7b: Options 短按（stop_teach + 自动存盘）", flush=True)
    node.tap(B_OPTIONS, 0.30)
    ok_back = node.wait_arm("READY", 5.0)
    node.sleep(0.5)
    mtime_after = mtime(LATEST_YAML)
    saved = mtime_after is not None and (
        mtime_before is None or mtime_after >= mtime_before - 1e-3) and (
        t0 - 60 <= (mtime_after or 0))
    rep.check("S7b Options → READY", ok_back, f"state={node.arm_state()}")
    rep.check("S7b latest.yaml 已更新", saved,
              f"mtime {mtime_before} → {mtime_after} msg={node.arm_msg()!r}")

    t0 = node.marker()
    print("[INFO] 场景7c: Square 短按（playback latest）", flush=True)
    node.tap(B_SQUARE)
    # 先确认确实进入回放 TRAJ（点下时臂已是 READY，旧循环立即退出是假失败根因），
    # 回放总长 ≈ ramp 5s + warp 8.7s ≈ 14s，给 20s 窗口。
    saw_pb_traj = False
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        if (node.arm_state() == "TRAJ"
                and node.arm_msg().startswith("playback")):
            saw_pb_traj = True
            break
        if any(d.get("color") == "purple" and d.get("reason", "").startswith("playback")
               for d in node.fb_since(t0)):
            saw_pb_traj = True
            break
        node.spin_for(0.1)
    pb_ready = saw_pb_traj and node.wait_arm("READY", 20.0)
    purple_pb = any(d.get("color") == "purple"
                    and d.get("reason", "").startswith("playback")
                    for d in node.fb_since(t0))
    rep.check("S7c Square 回放紫灯(playback*)→READY 绿",
              purple_pb and pb_ready,
              f"state={node.arm_state()} msg={node.arm_msg()!r}")
    node.sleep(4.0)

    # ---- 场景 8 前：确保不在 home（在 home 时 R3 直接失能、无 SAFE_PARK）----
    # 折叠 home 位 servo 处于奇异硬停，jog 推不动（run1 假失败根因）；直接
    # Triangle 去 ready，ready 确定离 home。
    print("[INFO] 场景8 预备: Triangle 回 ready（离开 home 以触发 safe-park）",
          flush=True)
    node.tap(B_TRIANGLE)
    node.wait_pose(READY_POSE, POSE_TOL, 9.0)
    node.wait_arm("READY", 3.0)
    node.sleep(4.0)

    t0 = node.marker()
    print("[INFO] 场景8: R3 短按（safe-park → disable）", flush=True)
    node.tap(B_R3)
    saw_park = False
    end = time.monotonic() + 14.0
    while time.monotonic() < end:
        if node.arm_state() == "SAFE_PARK":
            saw_park = True
        if node.arm_state() == "DISABLED":
            break
        node.spin_for(0.1)
    # 状态边沿即时发布（LL-064）后，这里可能在 DISABLED 边沿帧到达的几 ms 内
    # 跳出，而反馈节点 15Hz 还没派生橙灯/弱震；补 drain 等其发布。
    node.spin_for(0.6)
    purple_park = any(d.get("color") == "purple" for d in node.fb_since(t0))
    weak = any(d.get("rumble_weak", 0) >= 0.3 for d in node.fb_since(t0))
    orange_dis = node.arm_state() == "DISABLED" and fb_any(
        t0, lambda d: d.get("color") == "orange" and d.get("state") == "DISABLED")
    rep.check("S8 经过 SAFE_PARK（紫灯）", saw_park and purple_park,
              f"终态={node.arm_state()}（若已在 home 会直接失能，无 park）")
    rep.check("S8 终态 DISABLED + 橙灯", node.arm_state() == "DISABLED"
              and any(d.get("color") == "orange" for d in node.fb_since(t0)),
              f"state={node.arm_state()}")
    rep.check("S8 失能弱震 120ms", weak)

    # ---- 场景 9：L3 恢复 ----
    t0 = node.marker()
    print("[INFO] 场景9: L3 短按（gate 仍开，直接 enable 恢复）", flush=True)
    node.tap(B_L3)
    ok9 = node.wait_arm("READY", 8.0)
    rep.check("S9 L3 恢复 READY/绿灯", ok9
              and fb_any(t0, lambda d: d.get("color") == "green"),
              f"state={node.arm_state()} power={node._power_state} gate={node._gate}")
    node.sleep(1.0)

    # ---- 场景 10：X 长按 1s 硬急停，L3 恢复 ----
    t0 = node.marker()
    print("[INFO] 场景10: Cross 长按 1.1s（硬急停 shutdown）", flush=True)
    node.press(B_CROSS)
    node.sleep(1.15)
    node.release(B_CROSS)
    ok_down = node.wait_power("Idle", False, 4.0)
    red = fb_any(t0, lambda d: d.get("class") == "offline"
                 or (d.get("color") == "red"))
    strong = fb_any(t0, lambda d: d.get("rumble_strong", 0) >= 0.5)
    rep.check("S10 power Idle / gate 关", ok_down,
              f"state={node._power_state} gate={node._gate}")
    rep.check("S10 红闪（硬急停/失电）", red)
    rep.check("S10 强震 600ms", strong)
    rep.warn("仿真差异记录：无 SoftProne 动画、无 F51 联动、门禁不阻断轨迹"
             "（真机不同）；arm 状态=" + str(node.arm_state()))

    t0 = node.marker()
    print("[INFO] 场景10 恢复: L3 短按（start → enable）", flush=True)
    node.tap(B_L3)
    ok_up = node.wait_power("Running", True, 6.0)
    ok_ready10 = node.wait_arm("READY", 8.0)
    # 经 INIT 回 READY 有 0.6s 白闪覆盖绿，等窗口过后再断言
    node.sleep(0.7)
    rep.check("S10 恢复 Running/gate 开 + READY 绿灯",
              ok_up and ok_ready10
              and fb_any(t0, lambda d: d.get("color") == "green"),
              f"power={node._power_state} gate={node._gate} arm={node.arm_state()}")
    node.sleep(1.0)

    # ---- 场景 11：Options 长按 3s set_zero（仿真差异）----
    t0 = node.marker()
    print("[INFO] 场景11: Options 长按 3.3s（power set_zero）", flush=True)
    node.press(B_OPTIONS)
    node.sleep(3.4)
    node.release(B_OPTIONS)
    node.sleep(1.5)
    no_fault = node.arm_state() not in ("FAULT",) and node._power_state == "Running"
    rep.check("S11 set_zero 后无故障、power=Running", no_fault,
              f"power={node._power_state} arm={node.arm_state()}")
    rep.warn("仿真差异记录：sim_power 对 set_zero 不校验状态前提（真机仅 "
             "Idle+gate 关 接受），且不会真的清多圈零点")

    rep.summary()
    n_fail = sum(1 for _, ok, _ in rep.results if not ok)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail else 0)


def green_steady(node):
    """反馈最近是否曾稳定绿（class ready）。"""
    for _, d in list(node._fb_hist)[-200:]:
        if d.get("color") == "green":
            return True
    return False


def mtime(path):
    try:
        return os.path.getmtime(path)
    except OSError:
        return None


if __name__ == "__main__":
    main()
