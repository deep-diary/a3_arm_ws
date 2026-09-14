#!/usr/bin/env python3
"""F51 使能安全 + F52 缺电机降级档的**真机**验收（domain 0，L1–L5 档）。

前提：真机栈已按 5J 档起好（见 docs/edge/QUICKSTART.md「使能安全与事故回归」）：
  ros2 launch a3_bringup a3_bringup.launch.py use_power_sequence:=false use_teleop:=false \
      gains_file:=<...>/control_gains_5j.yaml motor_map_file:=<...>/motor_map_5j.yaml
  ros2 launch a3_arm_controller arm_controller.launch.py config_file:=<...>/arm_controller_5j.yaml

本脚本是**唯一驱动真机的验收脚本**，安全口径：
  · 只驱动档位内电机（L1–L5）；L6/L7 断线缺失，任何使能/轨迹都不会碰到它们；
  · 唯一运动项是 P6 的小幅 move_to：增量经 safety_limits.clamp_delta（≤0.30 rad）、
    时长 ≥3 s（≥2.5 s 下限）、先走再原路返回；
  · 使能/失效能都由执行层 F51 语义保护（重锚到反馈位 + kp 软起步）；
  · 全程结束时电机处于失能态（最后一步是带外 reset），**不做 F40 回 home 的 park**
    （本机臂悬在 L4≈-1.1 rad，park 是 ~1.4 rad 的单次运动，超出今晚无人在场的授权范围）。

用法（先 source scripts/a3_shell_env.sh）：
  A3_REAL_ARM_ACCEPT=1 python3 scripts/a3_test/f51_real_arm_acceptance.py \
      --bridge-log /tmp/a3_hw_5j.log
"""

import argparse
import os
import re
import sys
import threading
import time

import rclpy
from a3_can_bridge.msg import MotorStates
from a3_can_bridge.srv import MotorCommand, MotorStop
from a3_msgs.msg import ArmStatus, MonitorStatus
from a3_msgs.srv import MoveToJointPositions
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import safety_limits as SL  # noqa: E402
from incident_regression_test import (  # noqa: E402
    BUS_CAN1, CMD_CONTROL, KD_MAX, KD_MIN, KP_MAX, KP_MIN, P_MAX, P_MIN,
    TORQUE_MAX, log, u2f,
)
from trajectory_msgs.msg import JointTrajectory  # noqa: E402

PROFILE = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint"]
MOTORS = [1, 2, 3, 4, 5]

MAX_SETTLE_RAD = 0.25      # 使能后允许的重力沉降（越大越危险）
KP_STEADY_MIN = 40.0       # 额定 kp 下限（ramp 是否真的升到位）
RAMP_WINDOW_S = 1.5        # kp 软起步窗口（enable_ramp_duration_s 0.8 s + 余量）
MOVE_DELTA_RAD = 0.10      # P6 运动增量（≤ safety_limits 的 0.30 上限）
MOVE_DURATION_S = 3.0      # P6 时长（≥ 2.5 s 下限）


class RealArm(Node):
    """真机观测节点：只订阅 + 调用服务，不发布任何控制量（P6 的轨迹单独经 safety_limits）。"""

    def __init__(self):
        super().__init__("f51_real_arm_accept")
        be = QoSProfile(depth=4000, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        # 执行层 /can_tx_frames 是 RELIABLE depth 4000（LL-030），双订阅覆盖两种 QoS
        self.create_subscription(UInt8MultiArray, "/can_tx_frames", self._on_tx, be)
        self.create_subscription(UInt8MultiArray, "/can_tx_frames", self._on_tx, 4000)
        self.frames = []          # (t, motor, kind, p, kp, kd, tau) —— 与 8a 同一套解码
        self.js = None
        self.states = None
        self.monitor = None
        self.arm = None
        self._t0 = time.monotonic()
        for topic, attr, typ in (
            ("/joint_states", "js", JointState),
            ("/a3/motor/states", "states", MotorStates),
            ("/a3/monitor/status", "monitor", MonitorStatus),
            ("/a3/arm_status", "arm", ArmStatus),
        ):
            self.create_subscription(typ, topic, lambda m, a=attr: setattr(self, a, m), be)
        self._lock = threading.Lock()

    def t(self):
        return time.monotonic() - self._t0

    def _on_tx(self, msg):
        d = bytes(msg.data)
        if len(d) != 15:
            return
        if d[0] != BUS_CAN1:
            return
        can_id = int.from_bytes(d[3:7], "big")
        data = d[7:15]
        cmd = (can_id >> 24) & 0x1F
        mid = can_id & 0xFF
        if mid not in MOTORS:
            return
        with self._lock:
            now = self.t()
            if cmd == 0x03:
                self.frames.append((now, mid, "enable", 0.0, 0.0, 0.0, 0.0))
            elif cmd == 0x04:
                self.frames.append((now, mid, "reset", 0.0, 0.0, 0.0, 0.0))
            elif cmd == 0x06:
                self.frames.append((now, mid, "set_zero", 0.0, 0.0, 0.0, 0.0))
            elif cmd == CMD_CONTROL:
                tau = u2f((can_id >> 8) & 0xFFFF, -TORQUE_MAX[mid], TORQUE_MAX[mid])
                p = u2f((data[0] << 8) | data[1], P_MIN, P_MAX)
                kp = u2f((data[4] << 8) | data[5], KP_MIN, KP_MAX)
                kd = u2f((data[6] << 8) | data[7], KD_MIN, KD_MAX)
                self.frames.append((now, mid, "mit", p, kp, kd, tau))

    def mit(self, motor, t_from=0.0):
        with self._lock:
            return [f for f in self.frames if f[1] == motor and f[2] == "mit" and f[0] >= t_from]

    def enables(self, t_from=0.0):
        with self._lock:
            return sorted({f[1] for f in self.frames if f[2] == "enable" and f[0] >= t_from})

    def js_pos(self):
        """档位顺序的当前位置（champ 域，rad）。"""
        if self.js is None:
            return []
        return list(self.js.position)

    def modes(self):
        if self.states is None:
            return {}
        return {int(s.motor_id): int(s.mode_status) for s in self.states.states}

    def temps(self):
        if self.states is None:
            return {}
        return {int(s.motor_id): float(s.temperature_c) for s in self.states.states}


def wait_for(node, attr, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if getattr(node, attr) is not None:
            return True
        time.sleep(0.1)
    return False


def call(node, client, req, timeout=5.0):
    fut = client.call_async(req)
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < timeout:
        time.sleep(0.005)
    return fut.result() if fut.done() else None


def parse_reanchor_delta(msg):
    m = re.search(r"最大丢弃目标距离\s*([-+0-9.eE]+)\s*rad", msg or "")
    return float(m.group(1)) if m else None


def parse_array(path, key):
    """从 gains yaml 里读逐关节数组（与 f52_profile_test 同一套解析）。"""
    with open(path) as f:
        m = re.search(rf"^\s*{key}:\s*\[([^\]]*)\]", f.read(), re.M)
    if not m:
        raise SystemExit(f"✗ {path} 里找不到 {key}")
    return [float(x) for x in m.group(1).split(",")]


def champ_to_mit(champ, signs, offsets):
    """/joint_states（champ 域）→ MIT 域，与 /can_tx_frames 里的 p 同域可比值。"""
    return [s * c + o for s, c, o in zip(signs, champ, offsets)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bridge-log", default="", help="can_bridge launch 日志（用于核对 F52 档位行）")
    ap.add_argument("--gains", default=os.path.join(
        os.environ.get("A3_INSTALL", "/home/cat/a3_arm_ws/install"),
        "a3_can_bridge/share/a3_can_bridge/config/control_gains_5j.yaml"),
        help="档位对应 gains（取 joint_signs/joint_offsets_rad 做域换算）")
    ap.add_argument("--move", action="store_true",
                    help="打开 P6 小幅运动（默认跳过；--move 才动）")
    ap.add_argument("--nudge-joint", default="L4_joint",
                    help="P4b 修复动作的目标关节（默认 L4_joint）")
    ap.add_argument("--nudge-delta", type=float, default=0.0,
                    help="P4b 把该关节缓慢挪出限位外的增量 rad（0=关闭；"
                         "事故后 L4 静置在 URDF 限位外 0.05 rad，导致编排层 F48 门禁拒绝使能）")
    args = ap.parse_args()
    signs = parse_array(args.gains, "joint_signs")
    offsets = parse_array(args.gains, "joint_offsets_rad")
    if len(signs) != len(PROFILE) or len(offsets) != len(PROFILE):
        log(f"✗ {args.gains} 的 joint_signs/offsets 长度 {len(signs)}/{len(offsets)} != 档位 {len(PROFILE)}")
        return 2

    if os.environ.get("ROS_DOMAIN_ID", "") not in ("", "0"):
        log(f"✗ 本脚本必须跑在真机栈域（ROS_DOMAIN_ID=0），当前 {os.environ['ROS_DOMAIN_ID']}")
        return 2
    if os.environ.get("A3_REAL_ARM_ACCEPT") != "1":
        log("✗ 真机验收需显式确认：A3_REAL_ARM_ACCEPT=1 python3 ...（防误跑）")
        return 2

    failures = []
    rclpy.init()
    node = RealArm()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    # rclpy 签名是 (srv_type, srv_name)——类型在前
    cli = {n: node.create_client(t, s) for n, (s, t) in {
        "motor_enable": ("/a3/motor/enable", MotorCommand),
        "motor_reset": ("/a3/motor/reset", MotorCommand),
        "motor_stop": ("/a3/motor/stop", MotorStop),
        "arm_enable": ("/a3/arm/enable", Trigger),
        "move_to": ("/a3/arm/move_to", MoveToJointPositions),
    }.items()}
    # 直接下发轨迹（执行层路径，不经编排层）——只用于 P4b 的限位内修复动作
    traj_pub = node.create_publisher(
        JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10)

    def reset_end():
        """任何路径的收尾：带外失能（不发 park）。"""
        call(node, cli["motor_reset"], MotorCommand.Request(motor_id=0, command=2))

    try:
        # ---------------- P0 预检（只读） ----------------
        log("=== P0 预检（只读） ===")
        for attr in ("js", "states", "monitor", "arm"):
            if not wait_for(node, attr, 10.0):
                log(f"✗ 未收到 {attr}（真机栈没起？）")
                return 2
        js_names = list(node.js.name)
        log(f"P0 /joint_states {len(js_names)} 关节: {js_names}")
        log(f"P0 编排层 state={node.arm.state} ；看门狗 status={node.monitor.status}/{node.monitor.fault or '-'}")
        if js_names != PROFILE:
            failures.append(f"P0 F52 档位不符：/joint_states={js_names}（应为 {PROFILE}，L6/L7 不得出现）")
        if [int(s.motor_id) for s in node.states.states] != MOTORS:
            failures.append(f"P0 MotorStates 电机清单不符：{[int(s.motor_id) for s in node.states.states]}")
        bad = [(int(s.motor_id), s.fresh, s.enabled, int(s.fault_mask), float(s.temperature_c))
               for s in node.states.states if not s.fresh or s.enabled or s.fault_mask or s.temperature_c > 60]
        if bad:
            failures.append(f"P0 电机状态异常 (id, fresh, enabled, fault, temp): {bad}")
        else:
            log(f"P0 5 电机 fresh/失能/无故障，温度 {node.temps()} ✓")
        if node.monitor.fault:
            failures.append(f"P0 看门狗已报故障：{node.monitor.status}/{node.monitor.fault}")
        if node.arm.state not in ("IDLE", "DISABLED"):
            failures.append(f"P0 编排层初始 state={node.arm.state}（应为 IDLE/DISABLED）")
        q0 = node.js_pos()
        log(f"P0 当前位姿 (champ) {[round(v, 4) for v in q0]}")

        if args.bridge_log:
            try:
                txt = open(args.bridge_log).read()
                if "F52 档位：5 关节" not in txt:
                    failures.append(f"P0 执行层日志未见「F52 档位：5 关节」（{args.bridge_log}）")
                else:
                    log("P0 执行层日志确认 F52 档位：5 关节 ✓")
            except OSError as e:
                log(f"P0 跳过档位日志核对（{e}）")

        if failures:
            log("✗ P0 未通过，直接收尾（不动电机）")
            reset_end()
            raise SystemExit(1)

        # ---------------- P1 使能重锚（执行层直驱） ----------------
        log("=== P1 使能重锚（/a3/motor/enable 广播，不改变编排层状态） ===")
        t_en = node.t()
        r = call(node, cli["motor_enable"], MotorCommand.Request(motor_id=0))
        if r is None or not r.success:
            failures.append(f"P1 使能失败：{getattr(r, 'message', None)}")
        else:
            log(f"P1 使能响应：{r.message}")
            d_reanchor = parse_reanchor_delta(r.message)
            log(f"P1 重锚丢弃的陈旧目标最大距离 {d_reanchor} rad"
                if d_reanchor is not None else "P1 响应未见重锚距离（摘要格式待核对）")
            if "F51" not in r.message or "重锚" not in r.message:
                failures.append("P1 使能响应缺少 F51 重锚摘要")
            if f"重锚 {len(MOTORS)} 电机" not in r.message:
                failures.append(f"P1 重锚电机数不是档位内 {len(MOTORS)} 台：{r.message}")
        time.sleep(0.35)   # 等 enable 帧与反馈模式切换
        en = node.enables(t_from=t_en)
        if en != MOTORS:
            failures.append(f"P1 使能帧电机 {en}（应为 {MOTORS}）")
        else:
            log(f"P1 使能帧仅覆盖档位内 {en} ✓")

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and set(node.modes().get(m) for m in MOTORS) != {2}:
            time.sleep(0.1)
        modes = node.modes()
        if set(modes.get(m) for m in MOTORS) != {2}:
            failures.append(f"P1 使能后 mode 不为 2：{ {m: modes.get(m) for m in MOTORS} }")
        else:
            log("P1 5 电机 mode=2（使能生效）✓")

        time.sleep(2.0)   # 观察重力沉降
        worst_cmd, worst_pos, detail = 0.0, 0.0, []
        for i, m in enumerate(MOTORS):
            fr = node.mit(m, t_from=t_en)
            if not fr:
                failures.append(f"P1 电机 {m} 使能后无 MIT 帧")
                continue
            # 帧里是 MIT 域目标；/joint_states 是 champ 域——换算后再比
            js_mit = champ_to_mit(node.js_pos(), signs, offsets)
            cmd_dev = max(abs(f[3] - js_mit[i]) for f in fr[-50:])
            pos_dev = abs(node.js_pos()[i] - q0[i])
            worst_cmd = max(worst_cmd, cmd_dev)
            worst_pos = max(worst_pos, pos_dev)
            detail.append((m, round(fr[-1][3], 3), round(node.js_pos()[i] - q0[i], 3)))
        log(f"P1 使能后命令位 vs 反馈位最大偏差 {worst_cmd:.4f} rad；"
            f"实际沉降 {worst_pos:.4f} rad（(电机, MIT 命令位, Δ实际 champ 位) {detail}）")
        if worst_cmd > MAX_SETTLE_RAD:
            failures.append(f"P1 F51 失败：使能后命令位偏离反馈位 {worst_cmd:.3f} rad（陈旧目标未丢）")
        if worst_pos > MAX_SETTLE_RAD:
            failures.append(f"P1 使能后重力沉降 {worst_pos:.3f} rad 超限（重力补偿关闭，属预期但需人工确认）")

        # ---------------- P2 kp 软起步 ----------------
        log("=== P2 kp 软起步（0.8 s 斜坡） ===")
        ramp_ok = True
        for m in MOTORS:
            fr = [f for f in node.mit(m, t_from=t_en) if f[4] > 0.01]
            if not fr:
                continue
            kp_max = max(f[4] for f in fr)
            first = fr[0][4]
            t_reach = next((f[0] - t_en for f in fr if f[4] >= 0.9 * kp_max), None)
            if kp_max < KP_STEADY_MIN or first > 0.5 * kp_max or t_reach is None or t_reach > RAMP_WINDOW_S:
                ramp_ok = False
                log(f"P2 电机 {m}: 首帧 kp={first:.2f} 额定={kp_max:.1f} "
                    f"到 90% 用时 {t_reach if t_reach is None else round(t_reach, 2)} s ✗")
            else:
                log(f"P2 电机 {m}: kp {first:.2f}→{kp_max:.1f}，{t_reach:.2f} s 达 90% ✓")
        if not ramp_ok:
            failures.append("P2 kp 软起步不达标（首帧过高或无斜坡）")

        # ---------------- P3 stop latch ----------------
        log("=== P3 /a3/motor/stop 保持抑制（零增益保活，不续发旧目标） ===")
        t_stop = node.t()
        r = call(node, cli["motor_stop"], MotorStop.Request(motor_id=0))
        if r is None or not r.success:
            failures.append(f"P3 stop 调用失败：{getattr(r, 'message', None)}")
        time.sleep(1.5)
        after = [f for m in MOTORS for f in node.mit(m, t_from=t_stop + 0.3)]
        hot = [f for f in after if f[4] > 0.01]
        if not after:
            failures.append("P3 stop 后完全停帧（保活流断，LL-020 会致 js 冻结）")
        elif hot:
            failures.append(f"P3 F51 失败：stop 后仍续发 kp>0 帧 {[(f[1], round(f[4], 1)) for f in hot[:3]]}")
        else:
            log(f"P3 stop 后 {len(after)} 帧（5 电机合计）全为零增益保活 ✓；"
                f"当前位姿 {[round(v, 3) for v in node.js_pos()]}")

        # ---------------- P4 reset（带外失能，紧急路径） ----------------
        log("=== P4 /a3/motor/reset 带外失能 ===")
        t_reset = node.t()
        r = call(node, cli["motor_reset"], MotorCommand.Request(motor_id=0, command=2))
        time.sleep(1.2)
        modes = node.modes()
        if set(modes.get(m) for m in MOTORS) != {0}:
            failures.append(f"P4 reset 后 mode 不为 0：{ {m: modes.get(m) for m in MOTORS} }")
        else:
            log("P4 5 电机 mode=0（带外失能生效）✓")
        hot = [f for m in MOTORS for f in node.mit(m, t_from=t_reset + 0.3) if f[4] > 0.01]
        if hot:
            failures.append(f"P4 reset 后仍有 kp>0 帧 {[(f[1], round(f[4], 1)) for f in hot[:3]]}")

        # ---------------- P4b 限位修复动作（可选，唯一「先使能再小动」的判据） ----------------
        # 事故后 L4 静置在 URDF 限位外 0.05 rad（-1.0981 vs -1.0472），编排层 F48
        # 门禁会拒绝使能（正确行为）。要验证编排层就得先把该关节缓慢挪回限位内：
        #   执行层广播使能（F51 重锚）→ 经轨迹话题下发 1 段 ≤0.30 rad / ≥2.5 s 的
        #   小幅插值轨迹（safety_limits.build_safe_delta_trajectory）→ 保持使能态交给 P5。
        if args.nudge_delta:
            j = PROFILE.index(args.nudge_joint) if args.nudge_joint in PROFILE else -1
            if j < 0:
                failures.append(f"P4b 未知关节 {args.nudge_joint}（档位内 {PROFILE}）")
            else:
                log(f"=== P4b 修复动作：{args.nudge_joint} "
                    f"{args.nudge_delta:+.3f} rad（{SL.MIN_SEG_DURATION_S} s 插值） ===")
                r = call(node, cli["motor_enable"], MotorCommand.Request(motor_id=0, command=1))
                time.sleep(1.5)
                if set(node.modes().get(m) for m in MOTORS) != {2}:
                    failures.append(f"P4b 修复前使能失败：{getattr(r, 'message', None)}")
                else:
                    q_before = node.js_pos()
                    delta = SL.clamp_delta(args.nudge_delta)
                    if abs(delta - args.nudge_delta) > 1e-9:
                        log(f"P4b 增量被 safety_limits 限幅为 {delta:+.3f} rad（上限 "
                            f"{SL.MAX_TARGET_RAD}）")
                    deltas = [0.0] * len(PROFILE)
                    deltas[j] = delta
                    goal = [a + b for a, b in zip(q_before, deltas)]
                    traj = SL.build_safe_delta_trajectory(PROFILE, q_before, [deltas])
                    traj_pub.publish(traj)
                    t_end = time.monotonic() + SL.MIN_SEG_DURATION_S + 5.0
                    q_now = node.js_pos()
                    while time.monotonic() < t_end:
                        q_now = node.js_pos()
                        if max(abs(a - b) for a, b in zip(q_now, goal)) <= 0.06:
                            break
                        time.sleep(0.2)
                    dev = max(abs(a - b) for a, b in zip(q_now, goal))
                    log(f"P4b {args.nudge_joint}: {q_before[j]:.4f} → {q_now[j]:.4f}"
                        f"（目标 {goal[j]:.4f}，最大误差 {dev:.4f} rad，其余关节最大偏移 "
                        f"{max(abs(a - b) for k, (a, b) in enumerate(zip(q_now, q_before)) if k != j):.4f}）")
                    if dev > 0.10:
                        failures.append(f"P4b 修复动作未到位：最大误差 {dev:.3f} rad")

        # ---------------- P5 编排层 enable → READY ----------------
        log("=== P5 /a3/arm/enable → READY（含 F48 限位门禁） ===")
        r = call(node, cli["arm_enable"], Trigger.Request())
        if r is None or not r.success:
            failures.append(f"P5 编排层使能失败：{getattr(r, 'message', None)}")
            if r is not None and "position check failed" in (r.message or ""):
                log("P5 提示：F48 限位门禁拒绝（姿态在 URDF 限位外）。这是设计行为；"
                    "要验证编排层请加 --nudge-delta 先按安全限幅挪回限位内。")
        else:
            log(f"P5 响应：{r.message}")
            if "F51" not in r.message:
                failures.append("P5 响应缺少 F51 重锚摘要（执行层摘要未上抛）")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and node.arm.state != "READY":
            time.sleep(0.1)
        if node.arm.state != "READY":
            failures.append(f"P5 编排层 state={node.arm.state}（应为 READY）")
        else:
            log(f"P5 编排层 READY ✓（mode={node.arm.mode}）")

        # ---------------- P6 小幅运动（唯一运动项） ----------------
        # 安全门禁：前面任何零运动判据失败（F51 语义没站住）就不许再动真机。
        if args.move and failures:
            log(f"=== P6 小幅运动：跳过 —— 前置零运动判据已有 {len(failures)} 项失败，不允许运动 ===")
            for f in failures:
                log("  ! " + f)
        elif args.move:
            log(f"=== P6 小幅 move_to（L1 +{MOVE_DELTA_RAD} rad，{MOVE_DURATION_S} s，再原路返回） ===")
            q_start = node.js_pos()
            deltas = [0.0] * len(PROFILE)
            deltas[0] = SL.clamp_delta(MOVE_DELTA_RAD)
            goal = [a + b for a, b in zip(q_start, deltas)]
            r = call(node, cli["move_to"], MoveToJointPositions.Request(
                positions=goal, duration_s=MOVE_DURATION_S), timeout=15.0)
            if r is None or not r.success:
                failures.append(f"P6 move_to 失败：{getattr(r, 'message', None)}")
            time.sleep(MOVE_DURATION_S + 1.5)
            q_now = node.js_pos()
            dev = [abs(a - b) for a, b in zip(q_now, goal)]
            log(f"P6 到位：{[round(v, 4) for v in q_now]}（目标 {[round(v, 4) for v in goal]}，"
                f"最大误差 {max(dev):.4f} rad）")
            if max(dev) > 0.12:
                failures.append(f"P6 运动未到位：最大误差 {max(dev):.3f} rad")
            if node.monitor.fault:
                failures.append(f"P6 运动期间看门狗报故障：{node.monitor.status}/{node.monitor.fault}")
            # 原路返回
            r = call(node, cli["move_to"], MoveToJointPositions.Request(
                positions=q_start, duration_s=MOVE_DURATION_S), timeout=15.0)
            time.sleep(MOVE_DURATION_S + 1.5)
            back = node.js_pos()
            dev_back = max(abs(a - b) for a, b in zip(back, q_start))
            log(f"P6 返回起始位姿：最大误差 {dev_back:.4f} rad")
            if dev_back > 0.12:
                failures.append(f"P6 未回到起始位姿：最大误差 {dev_back:.3f} rad")
        else:
            log("=== P6 小幅运动：默认跳过（加 --move 才动） ===")

        # ---------------- P7 保持期观察（HOLD_DRIFT 假触发回归） ----------------
        log("=== P7 保持期 3 s 观察（LL-039 假触发修复后不得误报） ===")
        faults = []
        for _ in range(15):
            if node.monitor.fault:
                faults.append((node.monitor.status, node.monitor.fault, node.monitor.pending_faults))
            time.sleep(0.2)
        if faults:
            failures.append(f"P7 保持期看门狗误报：{faults[:3]}")
        else:
            log(f"P7 保持期看门狗状态 {node.monitor.status}（无故障）✓")

        # ---------------- P8 带外失能双通道（看门狗 + 编排层本地兜底） ----------------
        # 前置：必须是「整臂使能中」被带外失能——否则没有 all→none 沿，故障类判不出来。
        # 另一个前置：看门狗只在 ENABLED_EXPECTED_STATES（READY/TRAJ/SAFE_PARK）检查
        # 意外失能，编排层停在 IDLE（如 P5 被 F48 拒）时本判据不适用——单独报不适用，
        # 不记为失败（失败原因在上游 P5）。
        log("=== P8 带外 reset → 看门狗 UNEXPECTED_DISABLE + 编排层转 DISABLED ===")
        p8_applicable = node.arm.state in ("READY", "TRAJ", "SAFE_PARK")
        if not p8_applicable:
            log(f"P8 不适用：编排层 state={node.arm.state}（看门狗只在 "
                f"READY/TRAJ/SAFE_PARK 检查意外失能；根因见 P5）")
        if p8_applicable and set(node.modes().get(m) for m in MOTORS) != {2}:
            log("P8 前置：电机当前非全使能，先补一次执行层广播使能（F51 重锚）")
            call(node, cli["motor_enable"], MotorCommand.Request(motor_id=0, command=1))
            time.sleep(1.5)
        pre_modes = {m: node.modes().get(m) for m in MOTORS}
        if p8_applicable and set(pre_modes.values()) != {2}:
            failures.append(f"P8 前置失败：电机未全部使能 {pre_modes}，无法测带外失能")
        if p8_applicable:
            t_trip = node.t()
            call(node, cli["motor_reset"], MotorCommand.Request(motor_id=0, command=2))
            seen_monitor, seen_ctrl = None, None
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline and not (seen_monitor and seen_ctrl):
                if not seen_monitor and node.monitor.fault:
                    seen_monitor = (node.monitor.status, node.monitor.fault,
                                    node.monitor.action, round(node.t() - t_trip, 2))
                if not seen_ctrl and node.arm.state in ("DISABLED", "FAULT"):
                    seen_ctrl = (node.arm.state, round(node.t() - t_trip, 2))
                time.sleep(0.1)
            if not seen_monitor:
                failures.append("P8 看门狗未报 UNEXPECTED_DISABLE（持续窗/阈值问题）")
            elif seen_monitor[1] != "UNEXPECTED_DISABLE":
                failures.append(f"P8 看门狗报的是 {seen_monitor[1]}（应为 UNEXPECTED_DISABLE）")
            else:
                log(f"P8 看门狗 {seen_monitor[0]}/{seen_monitor[1]} action={seen_monitor[2]}"
                    f"（带外失能后 {seen_monitor[3]} s）✓")
            if not seen_ctrl:
                failures.append(f"P8 编排层未转 DISABLED/FAULT（当前 {node.arm.state}）")
            else:
                log(f"P8 编排层 → {seen_ctrl[0]}（{seen_ctrl[1]} s）✓")

    finally:
        reset_end()
        time.sleep(0.8)
        modes = node.modes()
        temps = node.temps()
        log(f"收尾：5 电机 mode={ {m: modes.get(m) for m in MOTORS} }；温度 {temps}")
        if set(modes.get(m) for m in MOTORS) != {0}:
            failures.append(f"收尾后仍有电机使能：{ {m: modes.get(m) for m in MOTORS} }")
        ex.shutdown()
        time.sleep(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print()
    if failures:
        log("✗ F51/F52 真机验收失败：")
        for f in failures:
            log("  - " + f)
        return 1
    log("✓ F51/F52 真机验收通过（档位 5 / 使能重锚 / kp 软起步 / stop 抑制 / 带外失能双通道）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
