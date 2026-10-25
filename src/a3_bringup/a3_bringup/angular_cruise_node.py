#!/usr/bin/env python3
"""F193 角度巡航节点：绕末端点旋转（末端位置保持，只改姿态）。

动作形状取自已验证的 F185（`scripts/a3_test/f185_angular_cruise_demo.py`）：
每段 `0→+A→−B→0`，速度用正弦包络 `v(t)=v_peak·sin(πt/T)`（两端慢、中间快），
`T = π|Δ|/(2·v_peak)`。幅度默认取 F185 的 URDF 限位扫描结果（X ±80°、
Y +25/−45°、Z ±45°），不贴限位。

三种模式：
  xyz    —— 固定三轴各摆一轮：X → Y → Z（演示姿态三自由度）
  spin   —— 偏航(Z) 往复连续（"旋转巡游"），按同一键或 stop 结束
  random —— 每轮随机挑一根基座轴 + 随机幅度（0.5~1.0×安全幅度），循环到 stop

安全门禁（任一不满足 → 拒绝启动 / 立即停发零速）：
  - `/a3/arm_status` 必须是 READY / SERVO
  - `/a3/control_mode` 不能是 ZERO_TORQUE
  - `/a3/vision/mode` 不能是 face_track（避免两路 twist 抢同一话题）
"""

from __future__ import annotations

import math
import random
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from a3_msgs.msg import ArmStatus
from a3_msgs.srv import AngularCruise

AXES = {
    "x": (1.0, 0.0, 0.0),
    "y": (0.0, 1.0, 0.0),
    "z": (0.0, 0.0, 1.0),
}
# F185 `--analyze` 结果（URDF 限位留 3–10° 余量）：(正摆, 反摆) 度
DEFAULT_AMP = {
    "x": (80.0, 80.0),
    "y": (25.0, 45.0),
    "z": (45.0, 45.0),
}
Leg = Tuple[str, float, float]


class AngularCruiseNode(Node):
    def __init__(self) -> None:
        super().__init__("angular_cruise")

        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("arm_status_topic", "/a3/arm_status")
        self.declare_parameter("control_mode_topic", "/a3/control_mode")
        self.declare_parameter("vision_mode_topic", "/a3/vision/mode")
        self.declare_parameter("ee_pose_topic", "/a3/ee_pose")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("peak_dps", 30.0)          # 峰值角速度
        self.declare_parameter("spin_amp_deg", 45.0)      # spin 模式偏航幅度
        self.declare_parameter("random_min_frac", 0.5)    # random 最小幅度比例
        self.declare_parameter("random_peak_min_frac", 0.7)
        # translate：连续随机平移（OU 过程，参考 scripts/a3_test/f181_servo_track.py）
        self.declare_parameter("translate_duration_s", 30.0)
        self.declare_parameter("translate_vmax_mps", 0.04)
        self.declare_parameter("translate_bound_m", 0.06)
        self.declare_parameter("translate_rot_bound_deg", 15.0)
        self.declare_parameter("translate_tau_s", 0.8)
        self.declare_parameter("translate_accel", 0.8)

        p = self.get_parameter
        self.peak = abs(math.radians(float(p("peak_dps").value)))
        self.spin_amp = abs(float(p("spin_amp_deg").value))
        self.rand_min = min(1.0, max(0.1, float(p("random_min_frac").value)))
        self.rand_peak_min = min(1.0, max(0.1, float(p("random_peak_min_frac").value)))
        self.tr_dur = max(1.0, float(p("translate_duration_s").value))
        self.tr_vmax = abs(float(p("translate_vmax_mps").value))
        self.tr_bound = abs(float(p("translate_bound_m").value))
        self.tr_rot = math.radians(abs(float(p("translate_rot_bound_deg").value)))
        self.tr_tau = max(0.05, float(p("translate_tau_s").value))
        self.tr_accel = max(0.05, float(p("translate_accel").value))
        self.dt = 1.0 / max(1.0, float(p("rate_hz").value))
        self.arm_state = ""
        self.control_mode = ""
        self.vision_mode = ""
        self._running = False
        self._mode = ""
        self._legs: List[Leg] = []
        self._leg_i = 0
        self._leg_t0 = 0.0
        self._leg_T = 0.0
        self._leg_sgn = 1.0
        self._leg_speed = 0.0
        self._cycles_left: Optional[int] = None   # None=无限
        self._repeat = ""
        self._zero_until = 0.0
        self._last_reason = ""
        # translate 状态
        self.ee_p = None
        self.ee_q = None
        self.ee_t = 0.0
        self._v = [0.0, 0.0, 0.0]
        self._last_v = [0.0, 0.0, 0.0]
        self._tr = {}

        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            TwistStamped, str(p("twist_topic").value), rel)
        self.create_subscription(ArmStatus, str(p("arm_status_topic").value),
                                 self._on_status, rel)
        self.create_subscription(String, str(p("control_mode_topic").value),
                                 self._on_control_mode, rel)
        self.create_subscription(String, str(p("vision_mode_topic").value),
                                 self._on_vision_mode, rel)
        self.create_subscription(PoseStamped, str(p("ee_pose_topic").value),
                                 self._on_ee, rel)
        # 尽力启动 servo（a3_servo_target 链路下该服务可能不存在，忽略即可）
        self._servo_start_cli = self.create_client(Trigger, "/servo_node/start_servo")

        self.create_service(AngularCruise, "/a3/demo/angular_cruise",
                            self._on_cruise)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"angular_cruise 就绪：{p('twist_topic').value}，峰值 {math.degrees(self.peak):.0f}°/s，"
            f"默认幅度 {DEFAULT_AMP}")

    # ------------------------------------------------------------------ 订阅
    def _on_status(self, msg: ArmStatus) -> None:
        self.arm_state = str(msg.state)

    def _on_control_mode(self, msg: String) -> None:
        self.control_mode = str(msg.data)

    def _on_vision_mode(self, msg: String) -> None:
        self.vision_mode = str(msg.data)

    def _on_ee(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.ee_p = (float(p.x), float(p.y), float(p.z))
        self.ee_q = (float(q.x), float(q.y), float(q.z), float(q.w))
        self.ee_t = time.monotonic()

    # ------------------------------------------------------------------ 服务
    def _on_cruise(self, req: AngularCruise.Request,
                   resp: AngularCruise.Response) -> AngularCruise.Response:
        action = int(req.action)
        if action == 2:                       # toggle
            action = 0 if self._running else 1
        if action == 0:
            self._stop("operator stop")
            resp.success = True
            resp.message = "angular cruise stopped"
            resp.mode = self._mode
            return resp

        mode = (req.mode or "xyz").strip().lower()
        if mode not in ("xyz", "spin", "random", "translate", "translate_xyz"):
            resp.success = False
            resp.message = f"unknown mode '{mode}' (xyz/spin/random/translate/translate_xyz)"
            resp.mode = self._mode
            return resp
        why = self._gate_reason()
        if why:
            resp.success = False
            resp.message = f"cannot start: {why}"
            resp.mode = self._mode
            return resp

        if mode in ("translate", "translate_xyz"):
            if self.ee_p is None or (time.monotonic() - self.ee_t) > 0.5:
                resp.success = False
                resp.message = "收不到 /a3/ee_pose（a3_ee_pose 在跑吗？）"
                resp.mode = self._mode
                return resp
            vmax = float(req.vmax_mps) if req.vmax_mps > 0.0 else self.tr_vmax
            bound = float(req.bounds_m) if req.bounds_m > 0.0 else self.tr_bound
            rotb = (float(req.rot_bound_deg) if req.rot_bound_deg > 0.0
                    else math.degrees(self.tr_rot))
            if mode == "translate_xyz":
                self._start_translate_xyz(bound, vmax, rotb)
                resp.message = (f"translate sweep start: ±{bound * 100:.0f}cm "
                                f"(z ±{bound * 75:.0f}cm), {vmax:.3f} m/s, "
                                f"姿态 ≤{rotb:.0f}°")
            else:
                dur = float(req.duration_s) if req.duration_s > 0.0 else self.tr_dur
                self._start_translate(dur, vmax, bound, rotb)
                resp.message = (f"translate tour start: {dur:.0f}s, vmax={vmax:.3f} m/s, "
                                f"bound=±{bound * 100:.0f}cm, rot≤{rotb:.0f}°")
            resp.success = True
            resp.mode = self._mode
            self.get_logger().warn(resp.message)
            return resp

        peak = abs(math.radians(float(req.peak_dps))) if req.peak_dps > 0.0 else self.peak
        amp = abs(float(req.amp_deg)) if req.amp_deg > 0.0 else 0.0
        cycles = int(req.cycles)
        self._start(mode, amp, peak, cycles)
        resp.success = True
        resp.message = (
            f"angular cruise start: mode={mode}, peak={math.degrees(peak):.0f}°/s, "
            f"cycles={'inf' if self._cycles_left is None else self._cycles_left}")
        resp.mode = self._mode
        self.get_logger().warn(resp.message)
        return resp

    # ------------------------------------------------------------------ 动作
    def _gate_reason(self) -> str:
        if self.arm_state not in ("READY", "SERVO"):
            return f"arm_state={self.arm_state or '?'}（需 READY）"
        if self.control_mode == "ZERO_TORQUE":
            return "control_mode=ZERO_TORQUE"
        if self.vision_mode == "face_track":
            return "人脸跟踪开着（两路 twist 会打架，先退出 TRACK）"
        return ""

    def _start(self, mode: str, amp_override: float, peak: float,
               cycles: int) -> None:
        self._mode = mode
        self.peak = peak
        if mode == "xyz":
            self._cycles_left = 1
            self._repeat = ""
            self._legs = self._legs_xyz(amp_override)
        elif mode == "spin":
            self._cycles_left = None if cycles == 0 else cycles
            self._repeat = "spin"
            self._legs = self._legs_spin(amp_override)
        else:
            self._cycles_left = None if cycles == 0 else cycles
            self._repeat = "random"
            self._legs = self._legs_random(amp_override)
        self._leg_i = 0
        self._begin_leg()
        self._running = True
        self._zero_until = 0.0
        self._last_reason = ""
        if self._servo_start_cli.service_is_ready():
            self._servo_start_cli.call_async(Trigger.Request())

    def _start_translate(self, dur: float, vmax: float, bound: float,
                         rot_deg: float) -> None:
        """连续随机平移巡游（OU 速度过程，参考 f181_servo_track.py）。"""
        self._mode = "translate"
        self._running = True
        self._zero_until = 0.0
        self._cycles_left = None
        self._repeat = ""
        self._legs = []
        self._v = [0.0, 0.0, 0.0]
        self._last_v = [0.0, 0.0, 0.0]
        self._tr = {
            "t0": time.monotonic(),
            "dur": dur,
            "vmax": abs(vmax),
            "hard": abs(bound),
            "soft": abs(bound) * 0.7,
            "rot": math.radians(abs(rot_deg)),
            "p0": self.ee_p,
            "q0": self.ee_q,
        }
        if self._servo_start_cli.service_is_ready():
            self._servo_start_cli.call_async(Trigger.Request())
        self.get_logger().warn(
            f"translate tour 开始：起点 {tuple(round(v, 3) for v in self.ee_p)}，"
            f"{dur:.1f}s / vmax {vmax:.3f} m/s / 半径 {bound * 100:.0f}cm / "
            f"姿态上限 {rot_deg:.0f}°")

    def _start_translate_xyz(self, amp: float, vmax: float, rot_deg: float) -> None:
        """固定三轴平移巡航：末端点只平移不旋转（X→Y→Z，各 0→+A→−A→0）。"""
        self._mode = "translate_xyz"
        self._running = True
        self._zero_until = 0.0
        self._cycles_left = 1
        self._repeat = ""
        self._leg_speed = abs(vmax)
        self._legs = self._legs_translate(abs(amp))
        self._leg_i = 0
        self._tr = {
            "p0": self.ee_p,
            "q0": self.ee_q,
            "rot": math.radians(abs(rot_deg)),
            "hard": abs(amp),
        }
        self._begin_leg()
        if self._servo_start_cli.service_is_ready():
            self._servo_start_cli.call_async(Trigger.Request())
        self.get_logger().warn(
            f"translate sweep 开始：起点 {tuple(round(v, 3) for v in self.ee_p)}，"
            f"水平 ±{amp * 100:.0f}cm / z ±{amp * 75:.0f}cm，"
            f"{vmax:.3f} m/s，姿态上限 {rot_deg:.0f}°")

    def _legs_translate(self, amp: float) -> List[Leg]:
        horiz = abs(amp) if amp > 0.0 else self.tr_bound
        vert = horiz * 0.75
        legs: List[Leg] = []
        for axis, a in (("x", horiz), ("y", horiz), ("z", vert)):
            legs += [(axis, 0.0, a), (axis, a, -a), (axis, -a, 0.0)]
        return legs

    def _legs_xyz(self, amp_override: float) -> List[Leg]:
        legs: List[Leg] = []
        for axis in ("x", "y", "z"):
            ap, an = self._axis_amp(axis, amp_override)
            legs += [(axis, 0.0, ap), (axis, ap, -an), (axis, -an, 0.0)]
        return legs

    def _legs_spin(self, amp_override: float) -> List[Leg]:
        a = abs(amp_override) if amp_override > 0.0 else self.spin_amp
        a = min(a, DEFAULT_AMP["z"][0], DEFAULT_AMP["z"][1])
        return [("z", 0.0, a), ("z", a, -a), ("z", -a, 0.0)]

    def _legs_random(self, amp_override: float) -> List[Leg]:
        axis = random.choice(("x", "y", "z"))
        cap = abs(amp_override) if amp_override > 0.0 else 0.0
        ap, an = self._axis_amp(axis, cap, randomize=True)
        old = self.peak
        self.peak = old * random.uniform(self.rand_peak_min, 1.0)
        self.get_logger().info(
            f"random 轮次：axis={axis} +{ap:.0f}°/-{an:.0f}°，"
            f"峰值 {math.degrees(self.peak):.0f}°/s")
        return [(axis, 0.0, ap), (axis, ap, -an), (axis, -an, 0.0)]

    def _axis_amp(self, axis: str, override: float,
                  randomize: bool = False) -> Tuple[float, float]:
        dp, dn = DEFAULT_AMP[axis]
        if override > 0.0:
            dp = dn = min(override, max(dp, dn))
        if randomize:
            dp *= random.uniform(self.rand_min, 1.0)
            dn *= random.uniform(self.rand_min, 1.0)
        return dp, dn

    def _begin_leg(self) -> None:
        axis, a0, a1 = self._legs[self._leg_i]
        d = a1 - a0
        self._leg_sgn = 1.0 if d >= 0.0 else -1.0
        if self._mode == "translate_xyz":
            delta = abs(d)                       # 米
            speed = self._leg_speed
        else:
            delta = abs(math.radians(d))         # 度 → 弧度
            speed = self.peak
        self._leg_T = math.pi * delta / (2.0 * max(1e-3, speed))
        self._leg_t0 = time.monotonic()

    def _stop(self, reason: str) -> None:
        if self._running:
            self.get_logger().warn(f"angular cruise stop：{reason}")
        self._running = False
        self._zero_until = time.monotonic() + 0.3
        self._publish((0.0, 0.0, 0.0))

    # ------------------------------------------------------------------ 主循环
    def _tick(self) -> None:
        now = time.monotonic()
        if not self._running:
            if now < self._zero_until:
                self._publish((0.0, 0.0, 0.0))
            return
        why = self._gate_reason()
        if why:
            self._stop(f"gate: {why}")
            return
        if self._mode == "translate":
            self._tick_translate(now)
            return
        if self._mode == "translate_xyz":
            self._tick_translate_xyz(now)
            return
        axis, a0, a1 = self._legs[self._leg_i]
        el = now - self._leg_t0
        if el >= self._leg_T:
            self._leg_i += 1
            if self._leg_i >= len(self._legs):
                self._leg_i = 0
                if self._cycles_left is not None:
                    self._cycles_left -= 1
                    if self._cycles_left <= 0:
                        self._stop("完成")
                        return
                if self._repeat == "random":
                    self._legs = self._legs_random(0.0)
                elif self._repeat == "spin":
                    self._legs = self._legs_spin(0.0)
            self._begin_leg()
            return
        w = self.peak * math.sin(math.pi * el / self._leg_T) * self._leg_sgn
        ax = AXES[axis]
        self._publish((w * ax[0], w * ax[1], w * ax[2]))

    def _tick_translate_xyz(self, now: float) -> None:
        """固定三轴平移：只用 linear，姿态保持不变（漂移超限即停）。"""
        tr = self._tr
        if self.ee_p is None or (now - self.ee_t) > 0.5:
            self._stop("ee_pose 超时")
            return
        p0, p = tr["p0"], self.ee_p
        dev = [p[i] - p0[i] for i in range(3)]
        hard = tr["hard"]
        if (abs(dev[0]) > hard * 1.2 or abs(dev[1]) > hard * 1.2
                or abs(dev[2]) > hard * 0.95):
            self._stop(f"触平移边界 {tuple(round(v * 1000) for v in dev)}mm")
            return
        q0, q = tr["q0"], self.ee_q
        if q0 is not None and q is not None:
            dot = abs(sum(a * b for a, b in zip(q0, q)))
            ang = 2.0 * math.acos(min(1.0, dot))
            if ang > tr["rot"]:
                self._stop(
                    f"姿态漂移 {math.degrees(ang):.1f}° > {math.degrees(tr['rot']):.0f}°")
                return
        axis, a0, a1 = self._legs[self._leg_i]
        el = now - self._leg_t0
        if el >= self._leg_T:
            self._leg_i += 1
            if self._leg_i >= len(self._legs):
                self._stop("完成")
                return
            self._begin_leg()
            return
        s = self._leg_speed * math.sin(math.pi * el / self._leg_T) * self._leg_sgn
        v = [0.0, 0.0, 0.0]
        v["xyz".index(axis)] = s
        self._publish((0.0, 0.0, 0.0), v)

    def _tick_translate(self, now: float) -> None:
        tr = self._tr
        if now - tr["t0"] >= tr["dur"]:
            self._stop(f"达到时长 {tr['dur']:.1f}s")
            return
        if self.ee_p is None or (now - self.ee_t) > 0.5:
            self._stop("ee_pose 超时")
            return
        p0, p = tr["p0"], self.ee_p
        dx, dy, dz = p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]
        rh = math.hypot(dx, dy)
        hard, soft, vmax = tr["hard"], tr["soft"], tr["vmax"]
        if rh >= hard or abs(dz) >= hard * 0.8:
            self._stop(f"触平移边界 h={rh * 1000:.0f}mm z={dz * 1000:.0f}mm")
            return
        q0, q = tr["q0"], self.ee_q
        if q0 is not None and q is not None:
            dot = abs(sum(a * b for a, b in zip(q0, q)))
            ang = 2.0 * math.acos(min(1.0, dot))
            if ang > tr["rot"]:
                self._stop(
                    f"姿态漂移 {math.degrees(ang):.1f}° > {math.degrees(tr['rot']):.0f}°")
                return

        dt = self.dt
        v = self._v
        sigma = 0.8 * vmax
        for i in range(3):
            v[i] += (-(v[i] / self.tr_tau) * dt
                     + sigma * math.sqrt(dt) * random.gauss(0.0, 1.0))
        # 软边界回中（参考 f181：越靠边越往回推）
        if rh > soft:
            k = (rh - soft) / max(1e-3, hard - soft) * 2.0 * vmax
            v[0] -= math.copysign(1.0, dx) * k
            v[1] -= math.copysign(1.0, dy) * k
        z_soft, z_hard = soft * 0.8, hard * 0.8
        if abs(dz) > z_soft:
            v[2] -= (math.copysign(1.0, dz)
                     * (abs(dz) - z_soft) / max(1e-3, z_hard - z_soft) * 2.0 * vmax)
        # 加速度限幅 + 模长限幅
        prev = self._last_v
        lim = self.tr_accel * dt
        for i in range(3):
            dv = v[i] - prev[i]
            v[i] = prev[i] + max(-lim, min(lim, dv))
        n = math.sqrt(sum(c * c for c in v))
        if n > vmax and n > 1e-9:
            v = [c * vmax / n for c in v]
        self._v = v
        self._last_v = list(v)
        self._publish((0.0, 0.0, 0.0), v)

    def _publish(self, w, v=(0.0, 0.0, 0.0)) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (
            float(v[0]), float(v[1]), float(v[2]))
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = (
            float(w[0]), float(w[1]), float(w[2]))
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AngularCruiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop("shutdown")
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
