#!/usr/bin/env python3
"""F185：三轴角度巡航 demo（绕末端点连续摆幅，速度可调，基于 home 基准）。

与 `f183_servo_rotation_check.py` 的区别
  f183 是**逐项验收**：每轴正负各转一小段、每段之间停 0.5 s，判方向/幅值/回位。
  本脚本是**连续演示**：每轴按 `+A → −A → 0` 一趟走完（三段无停顿），三轴接力，
  全程末端点不动、最后回到起始姿态；用来看"接触参考点后末端会不会漂"，也就是精度。

摆幅来源
  `--analyze` 用与控栈同一份 URDF 做纯运动学仿真：从 home 出发，每 1° 求一次
  "末端点不动、只转姿态"的 IK，遇关节限位或 IK 无解即停 → 得到各轴正负的**最大可行摆幅**。
  脚本内置默认值就是该扫描结果留余量后的整数（改机构/改限位后重跑 --analyze 核对）。

速度上限（2026-10-24 真机实测）：X ≤60°/s、Y ≤40°/s、Z ≤35°/s（三轴连跑建议 ≤35）。
超限表现为"角度只走到 7~9 成、末端 Δp 涨到 2cm"——Y/Z 要整臂重构，关节速度约 2× 工具角速度，
先撞 1.5 rad/s 关节限速；X 只需 L6 1:1，所以能快。

用法
  python3 scripts/a3_test/f185_angular_cruise_demo.py --analyze        # 只仿真，不动臂
  python3 scripts/a3_test/f185_angular_cruise_demo.py --speed 30       # 巡航，峰值 30°/s
  python3 scripts/a3_test/f185_angular_cruise_demo.py --speed 60 --axis x,z
  python3 scripts/a3_test/f185_angular_cruise_demo.py --auto-enable     # 不是 READY 时自己使能+回 home
  python3 scripts/a3_test/f185_angular_cruise_demo.py --amp x=85/85 --amp z=50/50

前置
  - 真机栈已起；`--auto-enable` 会代替手柄做「门禁 start → /a3/arm/enable → goto home」，
    不开该开关则要求已经是 READY；
  - 人脸跟踪必须关闭（否则两个发布者抢 /servo_node/delta_twist_cmds）；
  - 基准位形是 home（`named_poses.yaml` 的 legacy 半抬位）。

退出码
  0 通过 / 2 状态不是 READY / 3 有其它 twist 发布者 / 4 精度超限或触发保护 / 5 中途掉出 READY
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

try:
    from a3_msgs.msg import ArmStatus
    from a3_msgs.srv import GotoNamedPose
except ImportError:  # pragma: no cover
    print("找不到 a3_msgs：先 colcon build --packages-select a3_msgs && source install/setup.bash")
    sys.exit(1)

TOPIC_TWIST = "/servo_node/delta_twist_cmds"
ARM_JOINTS = ("L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint")
# named_poses.yaml 的 home（legacy 半抬位）
HOME_Q = np.array([0.0, 0.785, -0.785, 0.0, 0.0, 0.0])
AXES = {"x": np.array([1.0, 0.0, 0.0]),
        "y": np.array([0.0, 1.0, 0.0]),
        "z": np.array([0.0, 0.0, 1.0])}
# `--analyze` 扫描结果（2026-10-24，URDF el_a3 / 6 关节，home=[0,.785,-.785,0,0,0]）：
#   X: ±90°（L6 ±90 先到）  Y: +28°/−53°（L4 −60…+90，方向不对称）  Z: ±55°（L5 ±90 先到）
# 默认取 ~90%（留 3–10° 余量），正负分开给。想更大幅度用 --amp 显式指定，但别贴限位。
DEFAULT_AMP = {
    "x": (80.0, 80.0),
    "y": (25.0, 45.0),
    "z": (45.0, 45.0),
}


# --------------------------------------------------------------------- 运动学仿真
def _pin():
    try:
        import pinocchio as pin
        from a3_robot_model import robot_model
    except Exception as exc:  # noqa: BLE001
        print(f"[x] --analyze 需要 pinocchio + a3_robot_model：{exc}")
        sys.exit(1)
    model = pin.buildModelFromUrdf(robot_model.urdf_path())
    return pin, model, model.createData()


def _aa(axis: np.ndarray, ang: float) -> np.ndarray:
    a = axis / max(1e-12, float(np.linalg.norm(axis)))
    K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(ang) * K + (1.0 - math.cos(ang)) * (K @ K)


def _ik(pin, model, data, fid, q0, T_target, lo, hi, iters=400, damping=0.01, max_dq=0.20):
    """阻尼最小二乘 IK（种子 q0，限位内），收敛返回 (q, True)。"""
    q = np.clip(np.array(q0, dtype=float), lo, hi)
    for _ in range(iters):
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)
        T = data.oMf[fid]
        err = pin.log6(T.actInv(T_target)).vector          # LOCAL 系 6D 误差
        if np.linalg.norm(err[:3]) < 3e-4 and np.linalg.norm(err[3:]) < 2e-3:
            return q, True
        J = pin.getFrameJacobian(model, data, fid, pin.LOCAL)
        dq = J.T @ np.linalg.solve(J @ J.T + damping * damping * np.eye(6), err)
        n = np.linalg.norm(dq)
        if n > max_dq:
            dq *= max_dq / n
        q = np.clip(q + dq, lo, hi)
    return q, False


def analyze(step_deg: float = 1.0, max_deg: float = 180.0) -> None:
    """扫描各轴正负方向的最大可行摆幅（末端点固定、关节不越限）。"""
    pin, model, data = _pin()
    fid = model.getFrameId("end_effector")
    lo = np.array([model.lowerPositionLimit[model.idx_qs[model.getJointId(n)]] for n in ARM_JOINTS])
    hi = np.array([model.upperPositionLimit[model.idx_qs[model.getJointId(n)]] for n in ARM_JOINTS])
    q_home = np.clip(HOME_Q, lo, hi)
    pin.framesForwardKinematics(model, data, q_home)
    T_home = data.oMf[fid].copy()
    print(f"URDF: {model.name}  nq={model.nq}")
    print(f"home q = {np.round(q_home, 4)}")
    print(f"home tip p = {np.round(T_home.translation, 4)} m")
    print("关节限位 L1..L6 (deg): " +
          "  ".join(f"{n[0:2]}{n.split('_')[0]}[{math.degrees(a):.0f},{math.degrees(b):.0f}]"
                    for n, a, b in zip(ARM_JOINTS, lo, hi)))
    print(f"\n{'轴':>3} {'方向':>4} {'最大可行':>9} {'最先顶到的关节':>18}")
    best: dict[str, dict[float, float]] = {}
    for name, axis in AXES.items():
        best[name] = {}
        for sign in (+1.0, -1.0):
            q = q_home.copy()
            ok_deg = 0.0
            culprit = "-"
            deg = step_deg
            while deg <= max_deg:
                T = T_home.copy()
                T.rotation = _aa(axis, sign * math.radians(deg)) @ T_home.rotation
                q_new, ok = _ik(pin, model, data, fid, q, T, lo, hi)
                if not ok:
                    # 判断是不是关节顶到限位（贴着限位且 IK 不收敛）
                    at = [ARM_JOINTS[i] for i in range(6)
                          if q_new[i] <= lo[i] + 1e-3 or q_new[i] >= hi[i] - 1e-3]
                    culprit = at[0] if at else "IK 无解"
                    break
                q = q_new
                ok_deg = deg
                deg += step_deg
            best[name][sign] = ok_deg
            print(f"{name.upper():>3} {'+' if sign > 0 else '-':>4} {ok_deg:8.0f}° {culprit:>18}")
    print("\n建议默认（取 90% 并向下取整到 5°）：")
    for name in AXES:
        print(f"  {name.upper()}: +{5 * math.floor(0.9 * best[name][1.0] / 5):.0f}° / "
              f"-{5 * math.floor(0.9 * best[name][-1.0] / 5):.0f}°")


# --------------------------------------------------------------------- 巡航执行
class Cruise(Node):
    def __init__(self, speed_dps: float) -> None:
        super().__init__("f185_angular_cruise")
        self.speed = abs(speed_dps)
        self.p: np.ndarray | None = None
        self.r = np.eye(3)
        self.state = ""
        self.q: dict[str, float] = {}
        self.stale = False
        self.other_twist_t = 0.0
        self.own_pub_t = 0.0
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, "/a3/ee_pose", self._on_ee, rel)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_status, rel)
        self.create_subscription(TwistStamped, TOPIC_TWIST, self._on_twist, rel)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        # 硬件看门狗（latched）：L6 接头接触不良会周期性把反馈打成 stale → 整臂 freeze-hold，
        # 这时命令被丢弃、实测冻结，Δp 会莫名其妙涨（实测 191 mm）。直接判 stale 更清楚。
        self.create_subscription(
            Bool, "/a3/hardware/feedback_stale", self._on_stale,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(TwistStamped, TOPIC_TWIST, rel)
        self.cli = self.create_client(GotoNamedPose, "/a3/arm/goto_named_pose")
        self.cli_trig = self.create_client(Trigger, "/a3/arm/enable")
        self.pub_power = self.create_publisher(String, "/power_sequence/command", 10)

    # ---------------------------------------------------------------- 回调
    def _on_ee(self, m: PoseStamped) -> None:
        self.p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        q = m.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
        x, y, z, w = x / n, y / n, z / n, w / n
        self.r = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def _on_status(self, m: ArmStatus) -> None:
        self.state = m.state

    def _on_js(self, m: JointState) -> None:
        for n, v in zip(m.name, m.position):
            self.q[n] = float(v)

    def _on_stale(self, m: Bool) -> None:
        self.stale = bool(m.data)

    def _on_twist(self, _m: TwistStamped) -> None:
        if time.time() - self.own_pub_t > 0.02:
            self.other_twist_t = time.time()

    # ---------------------------------------------------------------- 基础
    def spin_for(self, sec: float) -> None:
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, w: np.ndarray) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.angular.x = float(w[0])
        msg.twist.angular.y = float(w[1])
        msg.twist.angular.z = float(w[2])
        self.own_pub_t = time.time()
        self.pub.publish(msg)

    def rot_deg(self, ref: np.ndarray) -> float:
        dr = ref.T @ self.r
        c = max(-1.0, min(1.0, (np.trace(dr) - 1.0) * 0.5))
        return math.degrees(math.acos(c))

    def goto_home(self, timeout: float = 25.0) -> bool:
        if not self.cli.wait_for_service(timeout_sec=2.0):
            print("[!] goto_named_pose 不可用")
            return False
        req = GotoNamedPose.Request()
        req.pose_name = "home"
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            print("[!] goto_named_pose 超时")
            return False
        print(f"[{'✓' if res.success else '!'}] goto home: {res.message}")
        return bool(res.success)

    def wait_ready(self, timeout: float = 8.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.state == "READY":
                return True
            self.spin_for(0.1)
        return self.state == "READY"

    def enable_and_home(self, timeout: float = 25.0) -> bool:
        """代替手柄：开门禁 + /a3/arm/enable + 回 home（仅 --auto-enable 时调用）。"""
        self.pub_power.publish(String(data="start"))
        self.spin_for(0.5)
        if not self.cli_trig.wait_for_service(timeout_sec=3.0):
            print("[!] /a3/arm/enable 不可用")
            return False
        fut = self.cli_trig.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        print(f"[{'✓' if res and res.success else '!'}] enable: "
              f"{res.message if res else '超时'}")
        if not (res and res.success):
            return False
        self.spin_for(0.5)
        return self.goto_home(timeout)


def run(args) -> int:
    axis_sel = [a.strip().lower() for a in args.axis.split(",") if a.strip()]
    bad = [a for a in axis_sel if a not in AXES]
    if bad:
        print(f"[x] 未知轴 {bad}，可选 x,y,z")
        return 2
    # 每轴一趟：0 → +A → −B → 0（净零，段间不停顿）
    legs: list[tuple[str, float, float]] = []
    for name in axis_sel:
        amp_pos, amp_neg = args.amp.get(name, DEFAULT_AMP[name])
        legs += [(name, 0.0, +amp_pos), (name, +amp_pos, -amp_neg), (name, -amp_neg, 0.0)]

    rclpy.init()
    node = Cruise(args.speed)
    try:
        node.spin_for(1.0)
        t_wait = time.time()
        while not node.state and time.time() - t_wait < 5.0:   # 等第一帧 arm_status
            node.spin_for(0.1)
        if node.state != "READY":
            if not args.auto_enable:
                print(f"[x] arm_status={node.state!r}，需要 READY（先使能，或加 --auto-enable）")
                return 2
            print(f"[*] arm_status={node.state!r} → 自动开门禁/使能/回 home")
            if not node.enable_and_home():
                print("[x] 自动使能失败")
                return 2
            node.wait_ready(8.0)
            if node.state != "READY":
                print(f"[x] 仍不是 READY（{node.state!r}）")
                return 2
        node.other_twist_t = 0.0
        node.spin_for(0.6)
        if node.other_twist_t > 0.0:
            print("[x] 还有别的节点在发 delta_twist_cmds（先关闭人脸跟踪）")
            return 3

        # 必须先拿到一帧 /a3/ee_pose：DDS 发现偶发慢于这里的 1.6s，拿不到就会以原点为基准算漂移
        # （实测出现过 起点 tip=(0,0,0) → Δp 411mm 的假数据）。
        t_wait = time.time()
        while node.p is None and time.time() - t_wait < 5.0:
            node.spin_for(0.1)
        if node.p is None:
            print("[x] 收不到 /a3/ee_pose（a3_ee_pose 是否在跑？/joint_states 是否 stale？）")
            return 2
        p0, r0 = node.p.copy(), node.r.copy()
        print(f"起点 tip = ({p0[0]:+.3f}, {p0[1]:+.3f}, {p0[2]:+.3f}) m，峰值角速度 {node.speed:.1f}°/s")
        print(f"{'段':>28} {'指令':>7} {'耗时':>6} {'实测角':>7} {'Δp(mm)':>8}")
        max_drift = 0.0
        for name, a0, a1 in legs:
            axis = AXES[name]
            d_deg = a1 - a0
            if abs(d_deg) < 1e-6:
                continue
            # 正弦速度包络：v(t)=v_peak·sin(πt/T)，位移 Δ=2·v_peak·T/π → T=π|Δ|/(2·v_peak)
            v_peak = math.radians(node.speed)
            T = math.pi * abs(math.radians(d_deg)) / (2.0 * v_peak)
            sgn = 1.0 if d_deg > 0 else -1.0
            r_ref = node.r.copy()
            t0 = time.time()
            aborted = ""
            while True:
                el = time.time() - t0
                if el >= T:
                    break
                amp = v_peak * math.sin(math.pi * el / T)
                node.send(sgn * amp * axis)
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.02)
                if node.p is not None:
                    max_drift = max(max_drift, float(np.linalg.norm(node.p - p0)) * 1e3)
                if node.state != "READY":
                    aborted = f"state={node.state}"
                    break
                if node.stale:
                    aborted = "硬件反馈 stale（电机掉线/CAN 异常）"
                    break
                if max_drift > args.max_drift_mm:
                    aborted = f"Δp {max_drift:.0f}mm > {args.max_drift_mm:.0f}mm"
                    break
            node.send(np.zeros(3))
            node.spin_for(0.08)
            ang_meas = node.rot_deg(r_ref)
            print(f"{name.upper() + f' {a0:+.0f}°→{a1:+.0f}°':>28} {abs(d_deg):6.0f}° {T:6.2f}s "
                  f"{ang_meas:6.1f}° {float(np.linalg.norm(node.p - p0)) * 1e3:8.1f}")
            if aborted:
                print(f"[x] 中止：{aborted}")
                break
        node.send(np.zeros(3))
        node.spin_for(0.6)
        p_end, r_end = node.p.copy(), node.r.copy()
        dp = float(np.linalg.norm(p_end - p0)) * 1e3
        da = node.rot_deg(r0)
        print(f"\n终点残差：位置 {dp:.2f} mm，姿态 {da:.2f}°；全程末端最大漂移 {max_drift:.2f} mm")
        ok = dp <= args.tol_res_mm and da <= args.tol_res_deg and max_drift <= args.max_drift_mm
        print(f"[{'PASS' if ok else 'FAIL'}] 判据：终点 Δp≤{args.tol_res_mm}mm、Δ角≤{args.tol_res_deg}°、"
              f"过程漂移≤{args.max_drift_mm}mm")
        if not args.no_home:
            node.goto_home()
            node.spin_for(0.5)
        return 0 if ok else 4
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    ap = argparse.ArgumentParser(description="F185 三轴角度巡航 demo（绕末端点，连续）")
    ap.add_argument("--analyze", action="store_true", help="只做运动学仿真扫描最大可行摆幅")
    ap.add_argument("--speed", type=float, default=30.0, help="峰值角速度 deg/s（默认 30）")
    ap.add_argument("--axis", default="x,y,z", help="参与巡航的轴，默认 x,y,z")
    ap.add_argument("--amp", action="append", default=[],
                    help="覆盖摆幅，形如 x=60/60、y=20/20（正/负，度）；可重复")
    ap.add_argument("--tol-res-mm", type=float, default=3.0, help="终点位置残差容差 mm")
    ap.add_argument("--tol-res-deg", type=float, default=1.0, help="终点姿态残差容差 deg")
    ap.add_argument("--max-drift-mm", type=float, default=8.0, help="过程末端漂移上限 mm（超则中止）")
    ap.add_argument("--no-home", action="store_true", help="结束后不回 home")
    ap.add_argument("--auto-enable", action="store_true",
                    help="不是 READY 时自动开门禁 + /a3/arm/enable + 回 home（代替手柄）")
    args = ap.parse_args()

    if args.analyze:
        analyze()
        return 0

    amps: dict[str, tuple[float, float]] = {}
    for item in args.amp:
        try:
            k, v = item.split("=")
            pos, neg = v.split("/")
            amps[k.strip().lower()] = (abs(float(pos)), abs(float(neg)))
        except ValueError:
            print(f"[x] --amp 格式应为 x=60/60，收到 {item!r}")
            return 2
    args.amp = amps
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
