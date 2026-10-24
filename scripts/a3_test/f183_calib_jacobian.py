#!/usr/bin/env python3
"""F183 视觉伺服自标定 v2（标定帧 + 深度方向）。

在标定时的末端姿态下，沿基座 X/Y/Z 各走一小段，实测：

    [du]   [du/dY  du/dZ] [dY]          # 横向 2×2（图像归一化误差 / 米）
    [dv] = [dv/dY  dv/dZ] [dZ]

    depth_dir = sign(Δz / ΔX)           # 基座 +X 是"远离"还是"靠近"人脸

控制器把 J 和 X 轴都当作**标定帧**（= 标定时的 R_base_ee）下的量：
运行时先算标定帧速度，再按 `R_now · R_calᵀ` 旋转到当前基座系。
这样 L1 转了 90° 后，"后退"会自动从 -X 变成 ±Y，不会把左右指令打到前后上。

相机装成 45°/60°/歪着装都无所谓——角度全部被 J 吸收；改支架只需重跑本脚本。

标定同时记录：
  - `distance_mm`（Z0）：J ∝ 1/Z，控制器按 Z/Z0 缩放；
  - `r_base_ee`：标定时的末端姿态；
  - `depth_dir`：沿标定帧 +X 走，人脸是变远(+1)还是变近(-1)。

前置条件
  1. 真机栈已起且 arm_status = READY（PS3 L3 使能）、门禁开；
  2. 人脸（或桌面上平板里的脸）在画面里、尽量居中，之后 20 s 内不要动；
  3. 跟踪**未使能**（否则会和本脚本抢 /servo_node/delta_twist_cmds）。

用法
  python3 scripts/a3_test/f183_calib_jacobian.py                # 默认 2cm 探针
  python3 scripts/a3_test/f183_calib_jacobian.py --probe 0.03   # 探针 3cm（更准）

输出
  ~/.a3/camera/face_jacobian.yaml  （标定后调 /a3/face_track/reload_jacobian 热加载）
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String  # noqa: F401

try:
    from a3_msgs.msg import ArmStatus, FaceTarget
except ImportError:  # pragma: no cover
    print("找不到 a3_msgs：先 colcon build --packages-select a3_msgs && source install/setup.bash")
    sys.exit(1)

TOPIC_TWIST = "/servo_node/delta_twist_cmds"


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


class Calibrator(Node):
    def __init__(self, probe_m: float, speed: float) -> None:
        super().__init__("f183_calib_jacobian")
        self.probe = abs(probe_m)
        self.speed = abs(speed)
        self.ft: FaceTarget | None = None
        self.ee: np.ndarray | None = None
        self.r_base_ee = np.eye(3)
        self.last_z_mm: float | None = None   # 最近一次 face_uv 采样的深度中位数
        self.state = ""
        self.ctrl_twist_t = 0.0
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(FaceTarget, "/a3/face_target",
                                 lambda m: setattr(self, "ft", m), rel)
        self.create_subscription(PoseStamped, "/a3/ee_pose", self._on_ee, rel)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_status, rel)
        self.create_subscription(TwistStamped, TOPIC_TWIST, self._on_twist, rel)
        self.pub = self.create_publisher(TwistStamped, TOPIC_TWIST, rel)

    def _on_ee(self, m: PoseStamped) -> None:
        self.ee = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        q = m.pose.orientation
        self.r_base_ee = quat_to_matrix(q.x, q.y, q.z, q.w)

    def _on_status(self, m: ArmStatus) -> None:
        self.state = m.state

    def _on_twist(self, _m: TwistStamped) -> None:
        self.ctrl_twist_t = time.time()

    # ------------------------------------------------------------------ 基础动作
    def spin_for(self, sec: float) -> None:
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, v: np.ndarray, w: np.ndarray | None = None) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v[0])
        msg.twist.linear.y = float(v[1])
        msg.twist.linear.z = float(v[2])
        if w is not None:
            msg.twist.angular.x = float(w[0])
            msg.twist.angular.y = float(w[1])
            msg.twist.angular.z = float(w[2])
        self.pub.publish(msg)

    def stop(self, sec: float = 1.0) -> None:
        t0 = time.time()
        while time.time() - t0 < sec:
            self.send(np.zeros(3), np.zeros(3))
            rclpy.spin_once(self, timeout_sec=0.02)

    def rot_about(self, axis_base: np.ndarray, r0: np.ndarray) -> float:
        """相对 r0 绕 base 轴 axis_base 的带符号转角（rad）。"""
        r_inc = self.r_base_ee @ r0.T
        tr = float(np.clip((np.trace(r_inc) - 1.0) / 2.0, -1.0, 1.0))
        ang = math.acos(tr)
        if ang < 1e-6:
            return 0.0
        ax = np.array([r_inc[2, 1] - r_inc[1, 2],
                       r_inc[0, 2] - r_inc[2, 0],
                       r_inc[1, 0] - r_inc[0, 1]]) / (2.0 * math.sin(ang))
        return ang * (1.0 if float(np.dot(ax, axis_base)) >= 0 else -1.0)

    def rotate_cal(self, axis_cal: np.ndarray, angle: float,
                   speed: float = 0.15) -> float:
        """绕标定帧 axis_cal 转 angle 弧度（带符号），返回实际转过的角度。

        标定帧就是"标定时刻的 base 帧"（控制器里恒有 R_now·R_calᵀ 映射），
        探针从标定姿态出发，所以该轴在 base 下就是 axis_cal 本身。
        """
        if self.ee is None:
            return 0.0
        axis_base = np.asarray(axis_cal, dtype=float)
        n = float(np.linalg.norm(axis_base))
        if n < 1e-9:
            return 0.0
        axis_base = axis_base / n
        r0 = self.r_base_ee.copy()
        t0 = time.time()
        timeout = abs(angle) / speed * 3.0 + 3.0
        while time.time() - t0 < timeout:
            if abs(self.rot_about(axis_base, r0)) >= abs(angle):
                break
            self.send(np.zeros(3), axis_base * (np.sign(angle) * speed))
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop(0.6)
        return self.rot_about(axis_base, r0)

    def face_uv(self, n: int = 8, need: float = 0.6) -> tuple[float, float] | None:
        """取 n 个有效样本的中位数；不足 need 比例则返回 None。"""
        us, vs, zs, t0 = [], [], [], time.time()
        while len(us) < n and time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            m = self.ft
            if m is not None and m.detected and m.valid:
                us.append(m.image_error.x)
                vs.append(m.image_error.y)
                if m.xyz_mm.z > 0.0:
                    zs.append(m.xyz_mm.z)
        if len(us) < max(2, int(n * need)):
            return None
        if zs:
            self.last_z_mm = float(np.median(zs))
        return float(np.median(us)), float(np.median(vs))

    def move_dir(self, direction: np.ndarray, dist: float) -> bool:
        """沿基座方向 direction（单位向量）走 dist 米（带符号），按 ee_pose 实测闭环。"""
        if self.ee is None:
            return False
        d = np.asarray(direction, dtype=float)
        nd = float(np.linalg.norm(d))
        if nd < 1e-9:
            return False
        d = d / nd
        start = self.ee.copy()
        v = d * (np.sign(dist) * self.speed)
        t0 = time.time()
        timeout = abs(dist) / self.speed * 3.0 + 3.0
        while time.time() - t0 < timeout:
            if abs(float(np.dot(self.ee - start, d))) >= abs(dist):
                break
            self.send(v)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop(0.8)
        return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", type=float, default=0.02, help="每个轴探针位移（米）")
    ap.add_argument("--speed", type=float, default=0.02, help="探针速度（米/秒）")
    ap.add_argument("--ang-probe", type=float, default=0.10,
                    help="转动探针角度（弧度，默认 0.1 ≈ 5.7°）")
    ap.add_argument("--out", default="~/.a3/camera/face_jacobian.yaml")
    args = ap.parse_args()

    rclpy.init()
    node = Calibrator(args.probe, args.speed)
    log = node.get_logger()
    try:
        # 1) 前置检查
        node.spin_for(1.0)
        if node.state not in ("READY", "SERVO"):
            log.error(f"手臂状态 = {node.state or '?'}，需先 L3 使能到 READY")
            return 2
        if time.time() - node.ctrl_twist_t < 0.5:
            log.error("检测到其它节点正在发 twist —— 先关跟踪："
                      "ros2 service call /a3/face_track/enable std_srvs/srv/SetBool \"{data: false}\"")
            return 2
        e0 = node.face_uv()
        if e0 is None:
            log.error("画面里没有稳定的人脸，把脸（或平板照片）放到画面中间后再试")
            return 2
        if max(abs(e0[0]), abs(e0[1])) > 0.6:
            log.error(f"人脸太靠边 err=({e0[0]:+.2f},{e0[1]:+.2f})，先让它居中到 |err|<0.4")
            return 2
        z0 = node.last_z_mm
        r_cal = node.r_base_ee.copy()
        log.warn(f"基线 err=({e0[0]:+.3f},{e0[1]:+.3f})  Z0={z0:.0f}mm  "
                 "开始探针（约 25 s，人别动）")

        # 2) 转动 2×2：绕标定帧 Y(pitch) / Z(yaw) 各转一小角度
        #    --ang-probe 0 可跳过（伺服角速度环尚不可用时）
        Ja = np.zeros((2, 2))
        ang_info = {}
        do_ang = args.ang_probe > 1e-6
        if not do_ang:
            log.warn("跳过转动探针（--ang-probe 0）：只标定平移 + 深度")
        for i, (axis, name) in (enumerate((((0.0, 1.0, 0.0), "pitch"),
                                           ((0.0, 0.0, 1.0), "yaw"))) if do_ang else []):
            e_before = e0
            d_ang = node.rotate_cal(np.array(axis), +args.ang_probe)
            e1 = node.face_uv()
            if e1 is None or abs(d_ang) < 0.3 * args.ang_probe:
                log.error(f"{name} 转动探针异常：dθ={d_ang:+.3f} rad 或丢脸")
                node.rotate_cal(np.array(axis), -max(abs(d_ang), 1e-3), 0.10)
                return 2
            de = np.array([e1[0] - e_before[0], e1[1] - e_before[1]])
            Ja[:, i] = de / d_ang
            ang_info[name] = dict(d_ang_rad=round(d_ang, 4),
                                  de_u=round(float(de[0]), 4),
                                  de_v=round(float(de[1]), 4))
            log.info(f"{name}：dθ={d_ang*57.2958:+.1f}° → Δerr=({de[0]:+.3f},{de[1]:+.3f})")
            # 转回原位并重建基线
            node.rotate_cal(np.array(axis), -d_ang, 0.10)
            e0 = node.face_uv() or e_before
            node.spin_for(0.3)
        ja_det = float(np.linalg.det(Ja))
        log.warn(f"Ja = [{Ja[0,0]:+.2f} {Ja[0,1]:+.2f}; {Ja[1,0]:+.2f} {Ja[1,1]:+.2f}] /rad  "
                 f"det={ja_det:+.2f}")
        if do_ang and abs(ja_det) < 1e-3:
            log.error("角雅可比退化 → 转动跟踪不可用（检查腕部是否已到限位）")
            return 3

        # 3) 先测深度方向：沿基座 +X 走，看 z 是变大还是变小
        z_before = z0
        node.move_dir(np.array([1.0, 0.0, 0.0]), +node.probe)
        node.face_uv()
        z_after = node.last_z_mm
        node.move_dir(np.array([1.0, 0.0, 0.0]), -node.probe)
        node.face_uv()
        node.spin_for(0.3)
        if z_before is None or z_after is None:
            log.error("深度读数缺失（FaceTarget.xyz_mm.z 为 0）→ 检查检测节点深度标定")
            return 2
        dz = float(z_after - z_before)
        if abs(dz) < 30.0:
            log.warn(f"深度探针 Δz={dz:+.0f}mm 太小（<30mm），方向不可靠：默认 +X=远离")
            depth_dir = 1
        else:
            depth_dir = 1 if dz > 0 else -1
        log.info(f"X 轴：Δz={dz:+.0f}mm → depth_dir={depth_dir:+d}"
                 f"（{'基座 +X = 远离人脸' if depth_dir > 0 else '基座 +X = 靠近人脸'}）")

        # 4) 横向 2×2：基座 Y / Z
        e0 = node.face_uv() or e0
        J = np.zeros((2, 2))
        info = {}
        for i, (axis, name) in enumerate(((1, "Y"), (2, "Z"))):
            d = np.zeros(3)
            d[axis] = 1.0
            ee0 = node.ee.copy()
            e_before = e0
            if not node.move_dir(d, +node.probe):
                log.error("探针运动失败（ee_pose 无更新）")
                return 2
            e1 = node.face_uv()
            ee1 = node.ee.copy()
            d_ee = float(ee1[axis] - ee0[axis])
            if e1 is None or abs(d_ee) < 0.5 * node.probe:
                log.error(f"{name} 轴探针异常：Δee={d_ee*1000:+.0f}mm 或丢脸")
                node.move_dir(d, -node.probe)
                return 2
            de = np.array([e1[0] - e_before[0], e1[1] - e_before[1]])
            J[:, i] = de / d_ee
            info[name] = dict(d_ee_m=round(d_ee, 4),
                              de_u=round(float(de[0]), 4), de_v=round(float(de[1]), 4))
            log.info(f"{name} 轴：Δee={d_ee*1000:+.1f}mm → Δerr=({de[0]:+.3f},{de[1]:+.3f})")
            # 回到起点并重建基线
            node.move_dir(d, -node.probe)
            e0 = node.face_uv() or e_before
            node.spin_for(0.3)

        det = float(np.linalg.det(J))
        cond = float(np.linalg.cond(J))
        log.warn(f"J = [{J[0,0]:+.1f} {J[0,1]:+.1f}; {J[1,0]:+.1f} {J[1,1]:+.1f}] /m  "
                 f"det={det:+.1f} cond={cond:.1f}  depth_dir={depth_dir:+d}")
        if abs(det) < 1e-3:
            log.error("雅可比退化（两个轴对画面的影响几乎平行）→ 换个人脸位置或姿态再试")
            return 3
        if cond > 25.0:
            log.warn(f"条件数偏大（{cond:.0f}）：某些方向增益会很敏感，"
                     "建议探针加大到 --probe 0.03 重标")

        out = os.path.expanduser(args.out)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        payload = {
            "stamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "probe_m": node.probe,
            "speed_mps": node.speed,
            "frame": "cal",              # J / depth 都在标定帧（r_base_ee）下解释
            "ee_pose": [round(float(v), 4) for v in node.ee],
            "r_base_ee": [round(float(v), 5) for v in r_cal.reshape(-1)],
            "face_err": [round(float(v), 4) for v in e0],
            # 标定时的相机→人脸距离：控制器用 Z/Z0 比例补偿 J（J ∝ 1/Z）
            "distance_mm": round(z0, 1) if z0 else None,
            # 沿标定帧 +X 走时深度变化方向：+1 = +X 远离，-1 = +X 靠近
            "depth_dir": depth_dir,
            "depth_dz_mm": round(dz, 1),
            "columns": info,
            **({"angular_jacobian": [[round(float(v), 3) for v in row] for row in Ja],
                "angular_probe_rad": round(abs(args.ang_probe), 4),
                "angular_columns": ang_info} if do_ang else {}),
            "jacobian": [[round(float(v), 2) for v in row] for row in J],
            "note": "归一化图像误差/米；基座 Y、Z 两轴在标定帧下；X 轴用于固定距离（depth_dir）。"
                    "换支架/换工作距离后重跑本脚本。",
        }
        with open(out, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)
        z0_txt = f"Z0={node.last_z_mm:.0f} mm" if node.last_z_mm else "无深度，Z 缩放不生效"
        log.warn(f"已写入 {out}（{z0_txt}，depth_dir={depth_dir:+d}）")
        log.warn("热加载：ros2 service call /a3/face_track/reload_jacobian std_srvs/srv/Trigger")
        return 0
    finally:
        node.stop(0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
