#!/usr/bin/env python3
"""A 步骤验收：a3_servo_target 的角速度语义（绕末端点旋转）。

验证内容
  1. 定向探针：沿基座 X/Y/Z 各正负转一小角度（默认 0.10 rad），要求
       - 末端**位置基本不动**（|Δp| < 5 mm）→ 证明是「绕末端点」而不是绕基座原点画弧；
       - 实测角度与指令角度同号、同轴，幅值偏差 < 25%；
       - 反向转回后位置与姿态残差都很小。
  2. 随机相位：每 1.5 s 随机挑 {+X,-X,+Y,-Y,+Z,-Z} 转一小段，总时长 60 s 或累计 60° 即停；
     逐段核对"本段实测角 ≈ 指令角"；若某关节触机械限位则提前结束并注明
     （home 是全伸位形，绕末端点旋转要靠整臂重构保持末端不动，L4 约 ±60° 会先到限位）；
     结束时调用 /a3/arm/goto_named_pose 回 home。

前置
  - 真机栈已起、`/a3/arm_status` 为 READY（L3 使能）、门禁开；
  - **人脸跟踪必须关闭**（否则两个发布者抢 /servo_node/delta_twist_cmds）：
      ros2 service call /a3/face_track/enable std_srvs/srv/SetBool "{data: false}"
  - 机械臂周围留出空间：绕末端点的旋转会让工具姿态改变，腕部可能扫过桌面。

用法
  python3 scripts/a3_test/f183_servo_rotation_check.py                 # 定向 + 随机
  python3 scripts/a3_test/f183_servo_rotation_check.py --no-random     # 只做定向 6 次
  python3 scripts/a3_test/f183_servo_rotation_check.py --ang 0.05 --speed 0.1   # 更保守

退出码
  0 全通过 / 2 状态不是 READY / 3 有其它 twist 发布者 / 4 有探针项失败 / 5 中途掉出 READY
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

try:
    from a3_msgs.msg import ArmStatus
    from a3_msgs.srv import GotoNamedPose
except ImportError:  # pragma: no cover
    print("找不到 a3_msgs：先 colcon build --packages-select a3_msgs && source install/setup.bash")
    sys.exit(1)

ARM_JOINTS = ("L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint")
try:  # 读 URDF 限位，用于判断"随机相位卡住"是机械限位还是伺服异常
    import pinocchio as pin
    from a3_robot_model import robot_model

    _M = pin.buildModelFromUrdf(robot_model.urdf_path())
    LIMITS = {n: (_M.lowerPositionLimit[_M.idx_qs[_M.getJointId(n)]],
                  _M.upperPositionLimit[_M.idx_qs[_M.getJointId(n)]]) for n in ARM_JOINTS}
except Exception:  # noqa: BLE001
    LIMITS = {}

TOPIC_TWIST = "/servo_node/delta_twist_cmds"
AXES = {"X": np.array([1.0, 0.0, 0.0]),
        "Y": np.array([0.0, 1.0, 0.0]),
        "Z": np.array([0.0, 0.0, 1.0])}


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_angle_axis(ra: np.ndarray, rb: np.ndarray) -> tuple[float, np.ndarray]:
    """相对旋转 raᵀ·rb 的 (角度 rad, 单位轴)。

    ⚠ 返回的轴表达在 **ra 的坐标系**（工具系）里。要跟 base 系比较必须先 `ra @ axis`，
    否则 dot 恒接近 0（实测 -0.2，误判方向错）。
    """
    dr = ra.T @ rb
    c = max(-1.0, min(1.0, (np.trace(dr) - 1.0) * 0.5))
    ang = math.acos(c)
    if ang < 1e-6:
        return 0.0, np.zeros(3)
    axis = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]])
    n = np.linalg.norm(axis)
    return ang, (axis / n if n > 1e-9 else np.zeros(3))


def aa_to_matrix(axis: np.ndarray, ang: float) -> np.ndarray:
    """Rodrigues：轴角 → 旋转矩阵。"""
    a = axis / max(1e-12, float(np.linalg.norm(axis)))
    K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(ang) * K + (1.0 - math.cos(ang)) * (K @ K)


class RotationCheck(Node):
    def __init__(self, speed: float) -> None:
        super().__init__("f183_servo_rotation_check")
        self.speed = abs(speed)
        self.p: np.ndarray | None = None
        self.r = np.eye(3)
        self.state = ""
        self.other_twist_t = 0.0
        self.own_pub_t = 0.0
        self.q: dict[str, float] = {}
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, "/a3/ee_pose", self._on_ee, rel)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_status, rel)
        self.create_subscription(TwistStamped, TOPIC_TWIST, self._on_twist, rel)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.pub = self.create_publisher(TwistStamped, TOPIC_TWIST, rel)
        self.cli = self.create_client(GotoNamedPose, "/a3/arm/goto_named_pose")

    # ------------------------------------------------------------------ 回调
    def _on_ee(self, m: PoseStamped) -> None:
        self.p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        q = m.pose.orientation
        self.r = quat_to_matrix(q.x, q.y, q.z, q.w)

    def _on_status(self, m: ArmStatus) -> None:
        self.state = m.state

    def _on_js(self, m: JointState) -> None:
        for n, v in zip(m.name, m.position):
            self.q[n] = float(v)

    def joint_at_limit(self, tol: float = 0.03) -> str:
        """返回触限位的关节名（含余量 tol），没有则空串。"""
        for n, (lo, hi) in LIMITS.items():
            v = self.q.get(n)
            if v is not None and (v <= lo + tol or v >= hi - tol):
                return f"{n}({v:+.2f}, 限位 {lo:+.2f}..{hi:+.2f})"
        return ""

    def _on_twist(self, _m: TwistStamped) -> None:
        # 自己发的帧不计数：1 ms 内刚发过就当作自己的
        if time.time() - self.own_pub_t > 0.02:
            self.other_twist_t = time.time()

    # ------------------------------------------------------------------ 基础
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

    def hold(self, w: np.ndarray, sec: float) -> None:
        t0 = time.time()
        while time.time() - t0 < sec:
            self.send(w)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def pose(self) -> tuple[np.ndarray, np.ndarray]:
        self.spin_for(0.12)          # 等一帧新的 ee_pose
        if self.p is None:
            return np.zeros(3), np.eye(3)
        return self.p.copy(), self.r.copy()

    def stop(self, sec: float = 0.6) -> None:
        self.hold(np.zeros(3), 0.25)
        self.spin_for(sec)

    def goto_home(self, timeout: float = 25.0) -> bool:
        if not self.cli.wait_for_service(timeout_sec=2.0):
            print("[!] goto_named_pose 不可用，请手动回 home（Triangle）")
            return False
        req = GotoNamedPose.Request()
        req.pose_name = "home"
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            print("[!] goto_named_pose 超时")
            return False
        ok = bool(fut.result().success)
        print(f"[{'✓' if ok else '!'}] goto_named_pose home: {fut.result().message}")
        return ok


def main() -> int:
    ap = argparse.ArgumentParser(description="a3_servo_target 旋转语义验收（绕末端点）")
    ap.add_argument("--ang", type=float, default=0.10, help="单次探针角度 rad（默认 0.10）")
    ap.add_argument("--speed", type=float, default=0.15, help="角速度 rad/s（默认 0.15）")
    ap.add_argument("--tol-pos-mm", type=float, default=5.0, help="位置漂移容差 mm")
    ap.add_argument("--tol-ang-ratio", type=float, default=0.25, help="角度幅值相对误差容差")
    ap.add_argument("--random-s", type=float, default=60.0, help="随机相位总时长 s")
    ap.add_argument("--max-angle-deg", type=float, default=60.0, help="随机相位累计角上限 deg")
    ap.add_argument("--slice-s", type=float, default=1.5, help="随机相位每段时长 s")
    ap.add_argument("--no-random", action="store_true", help="跳过随机相位")
    ap.add_argument("--no-home", action="store_true", help="结束后不自动回 home")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()
    if args.seed is not None:
        random.seed(args.seed)

    rclpy.init()
    node = RotationCheck(args.speed)
    try:
        node.spin_for(1.0)
        if node.state != "READY":
            print(f"[x] arm_status={node.state!r}，需要 READY（先 L3 使能）")
            return 2
        # 0.6 s 内不允许有别的 twist 发布者（人脸跟踪必须关）
        node.other_twist_t = 0.0
        node.spin_for(0.6)
        if node.other_twist_t > 0.0:
            print("[x] 还有别的节点在发 /servo_node/delta_twist_cmds（先关闭人脸跟踪）")
            return 3

        probe_s = args.ang / node.speed
        rows: list[tuple[str, bool, str]] = []

        print(f"== 定向探针：{args.ang:.3f} rad @ {node.speed:.2f} rad/s（每轴约 {probe_s:.2f}s）==")
        print(f"{'轴':>3} {'指令':>5} {'Δp(mm)':>8} {'实测角':>8} {'轴向dot':>8} {'回位Δp':>7} {'回位角':>7}  判定")
        for name, axis in AXES.items():
            for sign in (+1.0, -1.0):
                p0, r0 = node.pose()
                w = sign * node.speed * axis
                node.hold(w, probe_s)
                node.stop(0.5)
                p1, r1 = node.pose()
                d_ang, d_axis_local = rot_angle_axis(r0, r1)
                d_axis = r0 @ d_axis_local          # 工具系 → base 系
                dp_mm = float(np.linalg.norm(p1 - p0)) * 1e3
                axis_dot = float(np.dot(d_axis, axis * sign)) if d_ang > 1e-4 else 0.0
                # 反向回位
                node.hold(-w, probe_s)
                node.stop(0.6)
                p2, r2 = node.pose()
                back_mm = float(np.linalg.norm(p2 - p0)) * 1e3
                back_ang = rot_angle_axis(r0, r2)[0]

                ok = (dp_mm < args.tol_pos_mm
                      and abs(d_ang - args.ang) <= args.tol_ang_ratio * args.ang
                      and axis_dot > 0.8
                      and back_mm < args.tol_pos_mm
                      and back_ang < 0.02)
                rows.append((f"{name}{'+' if sign > 0 else '-'}", ok,
                             f"dp={dp_mm:.1f}mm ang={d_ang:.3f} dot={axis_dot:.2f} "
                             f"back={back_mm:.1f}mm/{back_ang:.3f}rad"))
                print(f"{name:>3} {sign:+.0f}   {dp_mm:8.1f} {d_ang:8.3f} {axis_dot:8.2f} "
                      f"{back_mm:7.1f} {back_ang:7.3f}  {'✓' if ok else '✗'}")
                if node.state != "READY":
                    print(f"[x] 中途 arm_status={node.state!r}（硬件/编排层异常），停止")
                    return 5

        if not args.no_random:
            print(f"\n== 随机相位：{args.random_s:.0f}s / 累计 {args.max_angle_deg:.0f}° ==")
            p_ref, r_ref = node.pose()
            t0 = time.time()
            acc = 0.0
            R_cmd = np.eye(3)   # 指令姿态的矩阵复合（随机轴/正负不能按标量相加）
            n = 0
            r_prev = r_ref
            cmd_seg = math.degrees(node.speed * args.slice_s)
            seg_ok: bool | None = None
            sat = ""
            while time.time() - t0 < args.random_s and math.degrees(acc) < args.max_angle_deg:
                name = random.choice(list(AXES))
                sign = random.choice((+1.0, -1.0))
                cmd_seg = math.degrees(node.speed * args.slice_s)
                node.hold(sign * node.speed * AXES[name], args.slice_s)
                node.stop(0.25)
                acc += node.speed * args.slice_s
                R_cmd = aa_to_matrix(AXES[name] * sign, node.speed * args.slice_s) @ R_cmd
                n += 1
                net_cmd = math.degrees(rot_angle_axis(np.eye(3), R_cmd)[0])
                cur = node.p if node.p is not None else p_ref
                if node.state != "READY":
                    print(f"[x] 随机相位中途 arm_status={node.state!r}，停止")
                    return 5
                seg_deg = math.degrees(rot_angle_axis(r_prev, node.r)[0])
                r_prev = node.r.copy()
                sat = node.joint_at_limit()
                if sat:
                    # 机械限位：这一段必然走不满，不作为失败项，直接收尾
                    print(f"  {n:02d} 段 {name}{'+' if sign > 0 else '-'} 本段仅 {seg_deg:5.1f}°"
                          f"/{cmd_seg:.1f}° → {sat} 触限位，随机相位提前结束")
                    break
                seg_ok = abs(seg_deg - cmd_seg) <= args.tol_ang_ratio * cmd_seg
                print(f"  {n:02d} 段 {name}{'+' if sign > 0 else '-'} "
                      f"本段 {seg_deg:5.1f}°/{cmd_seg:.1f}° {'✓' if seg_ok else '✗'}"
                      f" | 累计 {math.degrees(acc):5.1f}° | 净值指令 {net_cmd:5.1f}° "
                      f"| 位置偏 {np.linalg.norm(cur - p_ref) * 1e3:5.1f} mm | {node.state}")
            node.stop(0.8)
            p_end, r_end = node.pose()
            net_cmd = math.degrees(rot_angle_axis(np.eye(3), R_cmd)[0])
            # 期望终态 = R_cmd·R_ref（指令在 base 系左乘）；残差角 = 期望终态ᵀ·实测终态
            err_ang = math.degrees(rot_angle_axis(R_cmd @ r_ref, r_end)[0])
            dpos = float(np.linalg.norm(p_end - p_ref)) * 1e3
            ok = dpos < 3.0 * args.tol_pos_mm and seg_ok is not False
            note = f"（{sat} 触限位，提前结束）" if sat else ""
            rows.append(("random", ok, f"pos={dpos:.1f}mm net_cmd={net_cmd:.1f}° "
                                       f"err={err_ang:.1f}°{note}"))
            print(f"  汇总：位置漂移 {dpos:.1f} mm，净值指令 {net_cmd:.1f}°，"
                  f"姿态残差 {err_ang:.1f}°（实测净转 "
                  f"{math.degrees(rot_angle_axis(r_ref, r_end)[0]):.1f}°）{note}  "
                  f"{'✓' if ok else '✗'}")

        if not args.no_home:
            print()
            node.goto_home()

        bad = [r for r in rows if not r[1]]
        print("\n== 结果 ==")
        for name, ok, info in rows:
            print(f"  {'✓' if ok else '✗'} {name:<5} {info}")
        if bad:
            print(f"\n[FAIL] {len(bad)}/{len(rows)} 项未通过")
            return 4
        print(f"\n[PASS] {len(rows)}/{len(rows)} 项通过（绕末端点旋转语义正确）")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
