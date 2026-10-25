#!/usr/bin/env python3
"""F183 人脸跟踪控制器：a3_msgs/FaceTarget → TwistStamped（base 系）。

  /a3/face_target  (检测节点)
  /a3/ee_pose      (F175，取 R_base_ee 用于把相机系速度转到 base 系)
  /a3/arm_status   (门控：只允许 READY / SERVO)
  /a3/control_mode (门控：ZERO_TORQUE 时闭嘴)
        ↓
  /servo_node/delta_twist_cmds (a3_servo_target 输入，50 Hz)

控制律（相机系 → base 系）：
  v_cam  = [kp·X/1000, kp·Y/1000, kz·(Z-z_ref)/1000]      # X/Y/Z 来自 FaceTarget.xyz_mm
  v_base = R_base_ee · R_ee_cam(45° 外参) · v_cam
  死区内输出 0；|v| ≤ v_max；逐拍斜坡 ≤ ramp_accel·dt

安全：
  - 默认**未使能**，必须调 /a3/face_track/enable (std_srvs/SetBool) 打开；
  - 未使能时**完全不发** twist（避免与 PS4 遥操作抢同一话题）；
  - 使能但门控不满足 → 发 0 并保持；
  - 任何异常/退出 → 发 0。
"""

from __future__ import annotations

import math
import os
import sys
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

try:
    from a3_msgs.msg import ArmStatus, FaceTarget
except ImportError:
    print("[x] 找不到 a3_msgs：先 colcon build --packages-select a3_msgs", file=sys.stderr)
    raise


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw)·Ry(pitch)·Rx(roll)，角度为弧度。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


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


class FaceTrackController(Node):
    def __init__(self) -> None:
        super().__init__("face_track_controller")

        self.declare_parameter("target_topic", "/a3/face_target")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("ee_pose_topic", "/a3/ee_pose")
        self.declare_parameter("arm_status_topic", "/a3/arm_status")
        self.declare_parameter("control_mode_topic", "/a3/control_mode")
        self.declare_parameter("vision_mode_topic", "/a3/vision/mode")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate_hz", 50.0)

        # 控制增益与限幅
        self.declare_parameter("kp_img", 0.9)        # [1/s] 横向/纵向
        self.declare_parameter("kz", 1.2)            # [1/s] 深度
        self.declare_parameter("v_max", 0.05)        # [m/s] F181 已验证档位
        self.declare_parameter("ramp_accel", 0.5)    # [m/s²]
        self.declare_parameter("deadband", 0.025)    # 归一化图像误差死区
        self.declare_parameter("deadband_exit", 0.055)  # 滞环：出死区阈值（防边界抖动）
        self.declare_parameter("edge_hold", 0.88)    # |err| 超过则视为不可信（人脸被裁切）
        self.declare_parameter("settle_time_s", 0.6)  # 重新捕获后增益从 0 渐变到 1 的时长
        self.declare_parameter("dz_deadband_mm", 20.0)
        self.declare_parameter("z_ref_m", 0.45)
        self.declare_parameter("z_min_m", 0.15)
        self.declare_parameter("z_max_m", 0.90)
        # 只做平面跟踪（不标定单目深度时用）：vz 恒 0，死区不看 dz
        self.declare_parameter("enable_depth", True)

        # 相机相对杆轴下偏约 45°（home 的 L4=0 时相机正对桌面）。由 FK + 实测反推（2026-10-23）：
        # 画面右 = 基座 +Y、画面上 = +Z、光轴 = -X（前后，暂不跟踪）。
        # 该解析外参只在 mapping=extrinsic 或未标定雅可比时使用。
        self.declare_parameter("cam_rpy_deg", [135.0, 0.0, 0.0])
        # 采样一致性：取最近 median_n 个目标的逐分量中位数（抗单帧跳变），
        # 且最近若干帧的离散度超过 agree_tol（归一化误差）时判为"不稳定"→ 停发
        self.declare_parameter("median_n", 3)
        self.declare_parameter("agree_tol", 0.10)
        # F188 误差整形：I 消稳态误差（否则"下沉→撞死区出口→抬一下"会形成极限环），
        # D 提供相位超前（响应更快/更稳，但会放大检测噪声，先给 0，试调 0.03~0.10）。
        self.declare_parameter("ki_img", 0.3)     # [1/s] 积分增益（作用在归一化误差上）
        self.declare_parameter("i_max", 0.05)     # 积分项限幅（归一化误差当量）
        self.declare_parameter("i_leak", 1.0)     # [1/s] 死区内泄漏，防长期偏置
        self.declare_parameter("kd_img", 0.0)     # [s] 微分增益
        self.declare_parameter("d_tau", 0.15)     # [s] 微分低通
        # D 项限幅（归一化误差当量）：微分对检测跳变极敏感，夹住它就不会因单帧抖动打尖峰
        self.declare_parameter("d_max", 0.03)
        # 视觉伺服映射：auto（有标定用雅可比，否则解析外参）| jacobian | extrinsic
        # jacobian 由 scripts/a3_test/f183_calib_jacobian.py 实测生成（与安装角无关）
        self.declare_parameter("mapping", "auto")
        self.declare_parameter("jacobian_file", "~/.a3/camera/face_jacobian.yaml")
        self.declare_parameter("enable_angular", False)
        # F191 标定（TRACK 模式长按触发）：
        #   target_err_x/y = 目标中心偏移（归一化图像误差当量，水平/竖向）
        #   z_ref_m        = 目标距离（前后）；0 = 跟随雅可比标定距离
        self.declare_parameter("target_err_x", 0.0)
        self.declare_parameter("target_err_y", 0.0)
        self.declare_parameter("calibration_file", "~/.a3/camera/face_target.yaml")
        self.declare_parameter("ka", 0.8)            # [1/s] 角速度增益（enable_angular 时）
        self.declare_parameter("w_max", 0.6)         # [rad/s] 角速度上限
        self.declare_parameter("w_ramp", 2.0)        # [rad/s²] 角速度斜坡
        # 开转动时把平移增益按此比例降低，避免"平移 + 转动"双重增益造成过冲
        self.declare_parameter("lin_frac", 0.5)
        # F187 关节限位软门：两次真机事故（L5 +1.74、L4 −1.115 堵转 FAULT_STALL）都是
        # "某个关节顶到机械限位后，控制器还在要求它继续走"。用 URDF 限位 + 当前关节角 +
        # J⁻¹[v;w] 判断"这一拍会让哪个关节朝限位走"，只在该方向上按余量线性收缩；
        # 朝外的运动不受限（能自己退回来）；已越限则整拍停。
        self.declare_parameter("joint_guard", True)
        self.declare_parameter("joint_margin_deg", 15.0)   # 余量小于此值开始收，半程归零

        self.declare_parameter("target_timeout", 0.5)
        self.declare_parameter("ee_timeout", 0.5)
        # 工作空间虚拟墙：接近边界时把朝外的速度分量置 0，避免目标顶到夹紧值后
        # 继续积分 → 目标跑到够不到的地方（真机实测会引发 IK 大量失败 + 手臂乱扭）。
        # 注意 z 下限必须高于"够不到基座"的高度，否则目标会一路钻到 servo 的 z 夹紧面。
        self.declare_parameter("workspace_min", [-0.55, -0.45, 0.15])
        self.declare_parameter("workspace_max", [0.45, 0.45, 0.70])
        self.declare_parameter("workspace_margin_m", 0.03)
        # 追不动保护：图像误差在窗口内没下降、且输出接近饱和 → 判定"够不到/被夹死"，
        # 停发一段（> servo twist_timeout 0.3s，强制其重锚实测）后重试，避免持续满速硬顶。
        self.declare_parameter("stall_window_s", 1.5)
        self.declare_parameter("stall_min_progress", 0.05)   # 窗口内 |e| 至少应下降这么多
        # 无进展判据改成"位移 + 转角"而不是"速度"：
        # 2026-10-25 实测：目标静止时平移环以 3 mm/s 慢速空转（执行结果主要是手腕滚转，
        # 图像中心几乎不动），速度远低于 0.5*v_max 所以旧判据从不触发，30 s 爬了 18°。
        self.declare_parameter("stall_travel_m", 0.04)       # 无进展时累计位移上限 [m]
        self.declare_parameter("stall_rot_rad", 0.05)        # 无进展时累计转角上限 [rad]
        self.declare_parameter("stall_hold_s", 0.6)          # 停发时长

        # 俯仰辅助（F183 Step C）：竖向平移被虚拟墙挡死 / 追不动时，改由绕标定帧 Y 的俯仰
        # 继续收敛竖向误差（典型场景：人要抬高、末端已顶到高度上限，平移无解但抬腕可以继续看）。
        self.declare_parameter("pitch_assist", False)
        self.declare_parameter("pitch_gain", 0.5)            # [1/s]
        self.declare_parameter("pitch_max", 0.35)            # [rad/s]
        self.declare_parameter("pitch_stall_window_s", 2.0)  # 追不动后多久内允许启用

        p = self.get_parameter
        self.twist_topic = str(p("twist_topic").value)
        self.base_frame = str(p("base_frame").value)
        self.kp = float(p("kp_img").value)
        self.kz = float(p("kz").value)
        self.v_max = abs(float(p("v_max").value))
        self.ramp = abs(float(p("ramp_accel").value))
        self.deadband = abs(float(p("deadband").value))
        self.deadband_exit = max(self.deadband, abs(float(p("deadband_exit").value)))
        self.edge_hold = min(1.5, abs(float(p("edge_hold").value)))
        self.settle_time = max(1e-3, float(p("settle_time_s").value))
        self._in_dead = True
        self.dz_dead = abs(float(p("dz_deadband_mm").value))
        self.z_ref = float(p("z_ref_m").value)
        self.z_min = float(p("z_min_m").value)
        self.z_max = float(p("z_max_m").value)
        self.enable_angular = bool(p("enable_angular").value)
        self.target_err = np.array([float(p("target_err_x").value),
                                    float(p("target_err_y").value)])
        self.calib_file = str(p("calibration_file").value)
        self.w_max = abs(float(p("w_max").value))
        self.w_ramp = abs(float(p("w_ramp").value))
        self.lin_frac = float(np.clip(float(p("lin_frac").value), 0.0, 1.0))
        self.median_n = max(1, int(p("median_n").value))
        self.agree_tol = abs(float(p("agree_tol").value))
        self.ki = float(p("ki_img").value)
        self.i_max = abs(float(p("i_max").value))
        self.i_leak = abs(float(p("i_leak").value))
        self.kd = float(p("kd_img").value)
        self.d_tau = max(1e-3, float(p("d_tau").value))
        self.d_max = abs(float(p("d_max").value))
        self._i_term = np.zeros(2)   # 积分项（归一化误差当量）
        self._de_lp = np.zeros(2)    # 微分（低通后）
        self._e_prev = None
        self.mapping = str(p("mapping").value)
        self.jac_file = str(p("jacobian_file").value)
        self.jac_z0 = None
        self.r_cal = None          # 标定时的 R_base_ee（标定帧）
        self.depth_dir = 1.0       # 标定帧 +X 是远离(+1)还是靠近(-1)人脸
        self.jac_a = None          # 实测角雅可比 d(u,v)/d(θ_y, θ_z)（1/rad）
        self.z_ref_auto = self.z_ref <= 0.0   # 0 = 沿用标定距离（固定距离跟踪）
        self.jac = self._load_jacobian(self.jac_file)
        if self.z_ref_auto and self.jac_z0 is not None:
            self.z_ref = self.jac_z0
        if self.z_ref <= 0.0:
            self.z_ref = 0.45
        self._hist = deque(maxlen=max(15, self.median_n))   # 窗口留够，median_n 可调
        self._hist_t = 0.0
        self.enable_depth = bool(p("enable_depth").value)
        self.ka = float(p("ka").value)
        self.target_timeout = float(p("target_timeout").value)
        self.ee_timeout = float(p("ee_timeout").value)
        self.ws_min = np.array([float(v) for v in p("workspace_min").value])
        self.ws_max = np.array([float(v) for v in p("workspace_max").value])
        self.ws_margin = abs(float(p("workspace_margin_m").value))
        self.stall_window = max(0.3, float(p("stall_window_s").value))
        self.stall_progress = abs(float(p("stall_min_progress").value))
        self.stall_travel = abs(float(p("stall_travel_m").value))
        self.stall_rot = abs(float(p("stall_rot_rad").value))
        self.stall_hold = max(0.35, float(p("stall_hold_s").value))
        self.pitch_assist = bool(p("pitch_assist").value)
        self.pitch_gain = float(p("pitch_gain").value)
        self.pitch_max = abs(float(p("pitch_max").value))
        self.pitch_stall_win = max(0.0, float(p("pitch_stall_window_s").value))
        # F187 关节限位软门
        self.joint_guard = bool(p("joint_guard").value)
        self.joint_margin = math.radians(abs(float(p("joint_margin_deg").value)))
        self.q6 = None            # /a3/arm_status 的 6 关节角（L1..L6 顺序）
        self._init_joint_limits()
        self._stall_last = 0.0
        self.ee_pos = None
        # 估计的「伺服内部积分目标」：EE 实测 + 累计发出的位移。
        # 虚拟墙必须作用在目标上——只看实测评测会在目标跑飞后永远不触发（真机已复现）。
        self._est = None
        self._stall_hist = deque(maxlen=128)
        self._stall_until = 0.0
        self._prog_e_best = None   # 无进展守卫：最近一次改善时的 |e|
        self._prog_travel = 0.0    # 自那次改善以来累计发出的位移 [m]
        self._prog_rot = 0.0       # 自那次改善以来累计转角 [rad]
        self._prog_t0 = 0.0
        rpy = [math.radians(float(v)) for v in p("cam_rpy_deg").value]
        self.r_ee_cam = rpy_to_matrix(rpy[0], rpy[1], rpy[2])

        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(FaceTarget, str(p("target_topic").value), self._on_target, rel)
        self.create_subscription(PoseStamped, str(p("ee_pose_topic").value), self._on_ee, rel)
        self.create_subscription(ArmStatus, str(p("arm_status_topic").value),
                                 self._on_status, rel)
        self.create_subscription(String, str(p("control_mode_topic").value),
                                 self._on_mode, rel)
        self.pub = self.create_publisher(TwistStamped, self.twist_topic, rel)
        # F194：跟踪开关（显示层据此决定"叠加标注"还是"透传原图"）
        self._vision_mode_pub = self.create_publisher(
            String, str(p("vision_mode_topic").value), rel)
        self.create_service(SetBool, "/a3/face_track/enable", self._on_enable)
        self.create_service(Trigger, "/a3/face_track/reload_jacobian",
                            self._on_reload_jacobian)
        # F191：标定三件套（TRACK 模式长按）
        self.create_service(Trigger, "/a3/face_track/status", self._on_track_status)
        self.create_service(Trigger, "/a3/face_track/set_target", self._on_set_target)

        self.enabled = False
        self.target: FaceTarget | None = None
        self.target_t = 0.0
        self.r_base_ee = np.eye(3)
        self.ee_t = 0.0
        self.arm_state = ""
        self.control_mode = ""
        self.v_cmd = np.zeros(3)
        self.w_cmd = np.zeros(3)
        self.hold_until = 0.0        # 失能后仍发 0 的截止时间
        self._last_reason = ""
        self._vm_t = 0.0             # 视觉模式话题上次发布时刻

        # 联调期允许运行时改增益/限速/深度轴（不用重启节点）
        self.add_on_set_parameters_callback(self._on_params)

        dt = 1.0 / max(1.0, float(p("rate_hz").value))
        self.dt = dt
        self.create_timer(dt, self._tick)
        self._load_target_calib()
        self.get_logger().info(
            f"face_track 就绪：{p('target_topic').value} → {self.twist_topic}；"
            f"v_max {self.v_max:.3f} m/s，kp {self.kp:.2f}，z_ref {self.z_ref:.2f} m，"
            f"深度轴 {'开' if self.enable_depth else '关（平面跟踪）'}，"
            f"cam_rpy {[round(math.degrees(a), 1) for a in rpy]}°；"
            "当前**未使能**（调 /a3/face_track/enable）")

    # ---------------------------------------------------------- F191 标定
    def _load_target_calib(self) -> None:
        """读 ~/.a3/camera/face_target.yaml（目标中心偏移 / 目标距离）。"""
        fp = os.path.expanduser(self.calib_file)
        try:
            with open(fp, encoding="utf-8") as f:
                d = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"标定文件不可用（{exc}）: {fp}")
            return
        try:
            self.target_err = np.array([float(d.get("target_err_x", 0.0)),
                                        float(d.get("target_err_y", 0.0))])
            zr = float(d.get("z_ref_m", 0.0))
            if zr > 0.0:
                self.z_ref = zr
                self.z_ref_auto = False
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f"标定文件格式错误（{exc}）: {fp}")
            return
        self.get_logger().warn(
            f"已载入目标标定：target=({self.target_err[0]:+.3f}, "
            f"{self.target_err[1]:+.3f}) z_ref={self.z_ref:.3f} m（{fp}）")

    def _save_target_calib(self) -> str:
        fp = os.path.expanduser(self.calib_file)
        tmp = fp + ".tmp"
        try:
            os.makedirs(os.path.dirname(fp), exist_ok=True)
            with open(tmp, "w", encoding="utf-8") as f:
                yaml.safe_dump({
                    "target_err_x": float(self.target_err[0]),
                    "target_err_y": float(self.target_err[1]),
                    "z_ref_m": float(self.z_ref),
                }, f, allow_unicode=True, sort_keys=False)
            os.replace(tmp, fp)
            return fp
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"写标定文件失败（{exc}）: {fp}")
            return ""

    def _recent_medians(self):
        """最近 median_n 帧的 (err_x, err_y, z_mm) 中位数；不足返回 None。"""
        if not self._hist:
            return None
        last = list(self._hist)[-self.median_n:]
        return (float(np.median([h[0] for h in last])),
                float(np.median([h[1] for h in last])),
                float(np.median([h[4] for h in last])))

    def _on_set_target(self, _req: Trigger.Request,
                       resp: Trigger.Response) -> Trigger.Response:
        """水平/竖向 + 前后目标标定：把当前画面位置/距离记为新目标。

        TRACK 模式「Circle 长按」触发（人站到你希望被保持的位置）。
        """
        med = self._recent_medians()
        if med is None:
            resp.success = False
            resp.message = "无有效目标：先把脸放进画面并等 1 s"
            return resp
        ex, ey, z_mm = med
        self.target_err = np.array([ex, ey])
        z_ok = z_mm > 50.0
        if z_ok:
            self.z_ref = z_mm / 1000.0
            self.z_ref_auto = False
        self._reset_err_state()
        self._stall_hist.clear()
        self._prog_e_best = None
        self._est = None
        path = self._save_target_calib()
        resp.success = True
        resp.message = (
            f"target=({ex:+.3f}, {ey:+.3f})"
            + (f" z_ref={self.z_ref:.3f} m" if z_ok else " z 无有效深度（未改距离）")
            + (f" → {path}" if path else "（未持久化）"))
        self.get_logger().warn(f"目标标定：{resp.message}")
        self._publish(np.zeros(3), np.zeros(3),
                      self.get_clock().now().nanoseconds * 1e-9)
        return resp

    def _on_track_status(self, _req: Trigger.Request,
                         resp: Trigger.Response) -> Trigger.Response:
        """查询当前跟踪状态与标定值（TRACK 模式「Square 短按」）。"""
        med = self._recent_medians()
        err = f"({med[0]:+.3f},{med[1]:+.3f})" if med else "n/a"
        z = f"{med[2]:.0f} mm" if med else "n/a"
        resp.success = True
        resp.message = (
            f"enabled={int(self.enabled)} err={err} z={z} "
            f"z_ref={self.z_ref:.3f} m target=({self.target_err[0]:+.3f},"
            f"{self.target_err[1]:+.3f}) jac={int(self.jac is not None)} "
            f"jac_a={int(self.jac_a is not None)} arm={self.arm_state or '?'} "
            f"mode={self.control_mode or '?'}")
        self.get_logger().info(f"track status: {resp.message}")
        return resp

    def _on_params(self, params):
        for prm in params:
            if prm.name == "enable_depth":
                self.enable_depth = bool(prm.value)
                self.get_logger().warn(f"深度轴 → {'开' if self.enable_depth else '关（平面跟踪）'}")
            elif prm.name == "v_max":
                self.v_max = abs(float(prm.value))
                self.get_logger().warn(f"v_max → {self.v_max:.3f} m/s")
            elif prm.name == "kp_img":
                self.kp = float(prm.value)
            elif prm.name == "kz":
                self.kz = float(prm.value)
            elif prm.name == "pitch_assist":
                self.pitch_assist = bool(prm.value)
                self.get_logger().warn(f"俯仰辅助 → {'开' if self.pitch_assist else '关'}")
            elif prm.name == "pitch_gain":
                self.pitch_gain = float(prm.value)
                self.get_logger().warn(f"pitch_gain → {self.pitch_gain:.2f}")
            elif prm.name == "pitch_max":
                self.pitch_max = abs(float(prm.value))
                self.get_logger().warn(f"pitch_max → {self.pitch_max:.3f} rad/s")
            elif prm.name == "deadband":
                self.deadband = abs(float(prm.value))
                self.deadband_exit = max(self.deadband, self.deadband_exit)
            elif prm.name == "deadband_exit":
                self.deadband_exit = max(self.deadband, abs(float(prm.value)))
            elif prm.name == "edge_hold":
                self.edge_hold = min(1.5, abs(float(prm.value)))
            elif prm.name == "settle_time_s":
                self.settle_time = max(1e-3, float(prm.value))
            elif prm.name == "dz_deadband_mm":
                self.dz_dead = abs(float(prm.value))
            elif prm.name == "z_ref_m":
                self.z_ref = float(prm.value)
                self.z_ref_auto = self.z_ref <= 0.0
                if self.z_ref_auto and self.jac_z0 is not None:
                    self.z_ref = self.jac_z0
                self.get_logger().warn(
                    f"z_ref → {self.z_ref:.2f} m"
                    f"{'（跟随标定距离）' if self.z_ref_auto else ''}")
            elif prm.name == "z_min_m":
                self.z_min = float(prm.value)
                self.get_logger().warn(f"z_min → {self.z_min:.2f} m")
            elif prm.name == "z_max_m":
                self.z_max = float(prm.value)
                self.get_logger().warn(f"z_max → {self.z_max:.2f} m")
            elif prm.name == "enable_angular":
                self.enable_angular = bool(prm.value)
                self.get_logger().warn(f"转动跟踪 → {'开' if self.enable_angular else '关'}")
            elif prm.name == "ka":
                self.ka = float(prm.value)
                self.get_logger().warn(f"ka → {self.ka:.2f} 1/s")
            elif prm.name == "w_max":
                self.w_max = abs(float(prm.value))
                self.get_logger().warn(f"w_max → {self.w_max:.2f} rad/s")
            elif prm.name == "ki_img":
                self.ki = float(prm.value)
                self.get_logger().warn(f"ki_img → {self.ki:.3f}")
            elif prm.name == "i_max":
                self.i_max = abs(float(prm.value))
                self.get_logger().warn(f"i_max → {self.i_max:.3f}")
            elif prm.name == "i_leak":
                self.i_leak = abs(float(prm.value))
                self.get_logger().warn(f"i_leak → {self.i_leak:.2f}")
            elif prm.name == "kd_img":
                self.kd = float(prm.value)
                self.get_logger().warn(f"kd_img → {self.kd:.3f}")
            elif prm.name == "d_max":
                self.d_max = abs(float(prm.value))
                self.get_logger().warn(f"d_max → {self.d_max:.3f}")
            elif prm.name == "d_tau":
                self.d_tau = max(1e-3, float(prm.value))
                self.get_logger().warn(f"d_tau → {self.d_tau:.3f}")
            elif prm.name == "median_n":
                self.median_n = max(1, int(prm.value))
                self.get_logger().warn(f"median_n → {self.median_n}")
            elif prm.name == "lin_frac":
                self.lin_frac = float(np.clip(float(prm.value), 0.0, 1.0))
                self.get_logger().warn(f"lin_frac → {self.lin_frac:.2f}")
            elif prm.name == "joint_guard":
                self.joint_guard = bool(prm.value)
                if self.joint_guard and not self._pin_ok:
                    self._init_joint_limits()
                self.get_logger().warn(f"关节限位软门 → {'开' if self.joint_guard else '关'}")
            elif prm.name == "joint_margin_deg":
                self.joint_margin = math.radians(abs(float(prm.value)))
                self.get_logger().warn(
                    f"关节限位余量 → {math.degrees(self.joint_margin):.0f}°")
            elif prm.name == "cam_rpy_deg":
                r = [math.radians(float(v)) for v in prm.value]
                self.r_ee_cam = rpy_to_matrix(r[0], r[1], r[2])
                self.get_logger().warn(
                    f"cam_rpy_deg → {[round(math.degrees(a), 1) for a in r]}°")
            elif prm.name == "target_err_x":
                self.target_err[0] = float(prm.value)
                self.get_logger().warn(f"target_err_x → {self.target_err[0]:+.3f}")
            elif prm.name == "target_err_y":
                self.target_err[1] = float(prm.value)
                self.get_logger().warn(f"target_err_y → {self.target_err[1]:+.3f}")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ 回调
    def _on_target(self, msg: FaceTarget) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self.target = msg
        self.target_t = now
        if msg.detected and msg.valid:
            if now - self._hist_t > 0.4:      # 断档超过 0.4s 就丢弃旧样本
                self._hist.clear()
            self._hist.append((msg.image_error.x, msg.image_error.y,
                               msg.xyz_mm.x, msg.xyz_mm.y, msg.xyz_mm.z))
            self._hist_t = now

    def _on_ee(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self.r_base_ee = quat_to_matrix(q.x, q.y, q.z, q.w)
        self.ee_pos = np.array([msg.pose.position.x, msg.pose.position.y,
                                msg.pose.position.z])
        self.ee_t = self.get_clock().now().nanoseconds * 1e-9

    def _reset_err_state(self) -> None:
        """停发/重新捕获时清积分与微分状态（避免旧偏置带进新目标）。"""
        self._i_term[:] = 0.0
        self._de_lp[:] = 0.0
        self._e_prev = None

    def _on_status(self, msg: ArmStatus) -> None:
        self.arm_state = msg.state
        if msg.joint_names and len(msg.positions) == len(msg.joint_names):
            q = dict(zip(msg.joint_names, msg.positions))
            if all(n in q for n in self._pnames):
                self.q6 = np.array([q[n] for n in self._pnames])

    def _init_joint_limits(self) -> None:
        """读 URDF 限位 + 建 pinocchio 模型；失败就自动关闭软门（不影响其它功能）。"""
        self._pnames = [f"L{i}_joint" for i in range(1, 7)]
        self._pin_ok = False
        if not self.joint_guard:
            self.get_logger().info("关节限位软门：关闭（joint_guard=false）")
            return
        try:
            import pinocchio as pin
            from a3_robot_model import robot_model

            m = pin.buildModelFromUrdf(robot_model.urdf_path())
            self._pin, self._pm, self._pd = pin, m, m.createData()
            self._pfid = m.getFrameId("end_effector")
            self._plo = np.array([m.lowerPositionLimit[m.idx_qs[m.getJointId(n)]]
                                  for n in self._pnames])
            self._phi = np.array([m.upperPositionLimit[m.idx_qs[m.getJointId(n)]]
                                  for n in self._pnames])
            self._pin_ok = True
            self.get_logger().info(
                f"关节限位软门：开（余量 {math.degrees(self.joint_margin):.0f}°，半程归零）"
                "L1..L6 限位 " + ", ".join(
                    f"[{math.degrees(a):.0f},{math.degrees(b):.0f}]"
                    for a, b in zip(self._plo, self._phi)))
        except Exception as exc:  # noqa: BLE001
            self._pin_ok = False
            self.get_logger().warn(f"关节限位软门不可用（{exc}）→ 无限位保护")

    def _limit_scale(self, v_base: np.ndarray, w_base: np.ndarray) -> float:
        """这一拍（平移+转动）会让哪个关节朝限位走 → [0,1] 收缩系数。

        用 J⁻¹[v;w] 推出关节速度方向，只对"朝限位走"的关节按余量线性收缩；
        朝外的运动给 1.0（能自己退回来）。任一关节已越 URDF 限位 → 直接返回 0
        （整拍停），避免继续硬顶把别的关节也拖过限位。

        2026-10-24 第二次事故（L4 堵转）：只保护"转动"不够——L4 已顶在下限时，
        **平移**仍要求 IK 继续重构，3 s 内把 L5/L3 拖过限位、L4 堵转（FAULT_STALL）。
        所以这里必须用完整 twist，并覆盖全部 6 个关节。
        """
        if not self._pin_ok or self.q6 is None:
            return 1.0
        tw = np.concatenate([v_base, w_base])
        if float(np.linalg.norm(tw)) < 1e-6:
            return 1.0
        for i in range(6):                       # 已越限 → 整拍停
            if self.q6[i] < self._plo[i] - 1e-3 or self.q6[i] > self._phi[i] + 1e-3:
                self.get_logger().error(
                    f"{self._pnames[i]} 实测 {self.q6[i]:+.3f} 已越 URDF 限位 "
                    f"[{self._plo[i]:+.3f},{self._phi[i]:+.3f}] → 停发（需人工退出限位）",
                    throttle_duration_sec=2.0)
                return 0.0
        # 贴限位硬门（与方向无关）：任一关节余量 < joint_margin/2 就整拍停。
        # 为什么需要与方向无关：关节顶在限位上时，笛卡尔目标还会继续积分，
        # 一旦 IK 换支，瞬时雅可比方向可能"看起来朝外"，但实际会把别的关节一起拖过去
        # （2026-10-24 第二次事故就是 L4 顶住后把 L5 拖到 +1.66、L3 拖到 +0.03）。
        room_all = np.minimum(self.q6 - self._plo, self._phi - self.q6)
        i_min = int(np.argmin(room_all))
        if room_all[i_min] < 0.5 * self.joint_margin:
            self.get_logger().error(
                f"{self._pnames[i_min]} 余量仅 {math.degrees(room_all[i_min]):+.1f}° "
                f"（< {math.degrees(self.joint_margin) / 2:.1f}°）→ 整拍停，请先退回限位内",
                throttle_duration_sec=2.0)
            return 0.0
        try:
            self._pin.computeJointJacobians(self._pm, self._pd, self.q6)
            self._pin.updateFramePlacements(self._pm, self._pd)
            J = np.array(self._pin.getFrameJacobian(
                self._pm, self._pd, self._pfid, self._pin.LOCAL_WORLD_ALIGNED))
            qd = np.linalg.solve(J, tw)
        except Exception:  # noqa: BLE001  奇异/无解时不干预，交给伺服层
            return 1.0
        # 贴限位硬门（方向无关）：任一关节余量 < 半余量（默认 7.5°）就整拍停。
        # 为什么不能只靠方向判定：关节已经贴住限位时，Cartesian 目标会继续漂出去，
        # IK 随后"换支"式地大改关节（实测 1 s 内 L4 命令跳 1.08 rad），方向判定抓不住。
        room_all = np.minimum(self.q6 - self._plo, self._phi - self.q6)
        if float(np.min(room_all)) < 0.5 * self.joint_margin:
            i = int(np.argmin(room_all))
            self.get_logger().error(
                f"{self._pnames[i]} 余量仅 {math.degrees(room_all[i]):.1f}°（贴限位）→ 停发，"
                f"请先回 home/人工退出限位", throttle_duration_sec=2.0)
            return 0.0
        scale = 1.0
        half = 0.5 * self.joint_margin
        qmax = float(np.max(np.abs(qd)))
        if qmax < 1e-6:
            return 1.0
        # 只有"真的在动"的关节才参与判定：qd 里 1e-17 量级的数值噪声会被误判成
        # "朝限位走" → 把整拍命令归零（离线复现过）。
        floor = max(1e-3, 0.05 * qmax)
        for i in range(6):
            if abs(qd[i]) < floor:
                continue
            room = (self._phi[i] - self.q6[i]) if qd[i] > 0 else (self.q6[i] - self._plo[i])
            if room >= self.joint_margin:
                continue
            scale = min(scale, max(0.0, (room - half) / half))
            if scale <= 0.0:
                break
        return scale

    def _on_mode(self, msg: String) -> None:
        self.control_mode = msg.data

    def _on_enable(self, req: SetBool.Request, resp: SetBool.Response) -> SetBool.Response:
        if req.data == self.enabled:
            resp.success, resp.message = True, f"already {'enabled' if self.enabled else 'disabled'}"
            return resp
        self.enabled = bool(req.data)
        self.v_cmd[:] = 0.0
        self.w_cmd[:] = 0.0
        self._est = None
        self._stall_hist.clear()
        self._stall_until = 0.0
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.enabled:
            self.hold_until = 0.0
            resp.message = "face tracking ENABLED"
        else:
            self.hold_until = now + 1.0     # 失能后补发 1s 的 0，确保下游立刻停
            resp.message = "face tracking DISABLED"
        resp.success = True
        self.get_logger().warn(resp.message)
        self._publish_vision_mode()
        return resp

    def _publish_vision_mode(self) -> None:
        m = String()
        m.data = "face_track" if self.enabled else "off"
        self._vision_mode_pub.publish(m)
        self._vm_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_reload_jacobian(self, _req: Trigger.Request,
                            resp: Trigger.Response) -> Trigger.Response:
        self.jac = self._load_jacobian(self.jac_file)
        if self.jac is not None and self.z_ref_auto and self.jac_z0 is not None:
            self.z_ref = self.jac_z0     # 固定距离默认跟随新标定距离
        resp.success = self.jac is not None
        resp.message = ("已加载雅可比标定" if self.jac is not None
                        else f"未找到/不可用：{os.path.expanduser(self.jac_file)}")
        return resp

    def _use_jacobian(self) -> bool:
        return self.jac is not None and self.mapping in ("auto", "jacobian")

    def _load_jacobian(self, path: str):
        """读实测雅可比（归一化图像误差 / 基座米）。缺失或退化时返回 None。"""
        fp = os.path.expanduser(path)
        try:
            with open(fp, encoding="utf-8") as f:
                d = yaml.safe_load(f)
            J = np.array(d["jacobian"], dtype=float)
            if J.shape != (2, 2):
                raise ValueError(f"shape={J.shape}")
            det = float(np.linalg.det(J))
            if abs(det) < 1e-9:
                raise ValueError(f"退化 det={det:.3g}")
            d_mm = d.get("distance_mm")
            self.jac_z0 = float(d_mm) / 1000.0 if d_mm else None
            # v2 标定：J 与 X 轴都在标定帧下解释，运行时按 R_now·R_calᵀ 旋转。
            # 这样 L1 转动后"后退"方向会跟着末端转，而不是永远沿基座 X。
            self.r_cal = None
            self.depth_dir = 1.0
            if str(d.get("frame", "base")) == "cal":
                m = d.get("r_base_ee")
                if m is not None and len(m) == 9:
                    self.r_cal = np.array(m, dtype=float).reshape(3, 3)
                dd = float(d.get("depth_dir", 1))
                self.depth_dir = 1.0 if dd >= 0 else -1.0
            # 角雅可比（转动跟踪）：d(u,v)/d(θ_y,θ_z)，单位 1/rad
            self.jac_a = None
            ja = d.get("angular_jacobian")
            if ja is not None:
                A = np.array(ja, dtype=float)
                if A.shape == (2, 2) and abs(float(np.linalg.det(A))) > 1e-3:
                    self.jac_a = A
        except FileNotFoundError:
            self.get_logger().info(f"无雅可比标定（{fp}）→ 走解析外参映射")
            return None
        except Exception as e:      # noqa: BLE001
            self.jac_z0 = None
            self.get_logger().warn(f"雅可比不可用（{e}）→ 走解析外参映射")
            return None
        self.get_logger().warn(
            "使用实测雅可比 [du/dY du/dZ; dv/dY dv/dZ] = "
            f"[{J[0, 0]:+.1f} {J[0, 1]:+.1f}; {J[1, 0]:+.1f} {J[1, 1]:+.1f}] /m，"
            f"det={det:+.1f}，标定距离 Z0="
            f"{(f'{self.jac_z0 * 1000:.0f} mm' if self.jac_z0 else '未知（不做 Z 缩放）')}，"
            f"深度轴 {'标定帧 X（depth_dir=%+.0f）' % self.depth_dir if self.r_cal is not None else '未标定'}，"
            f"来源 {fp}")
        if self.jac_a is not None:
            self.get_logger().warn(
                "角雅可比 d(u,v)/d(pitch,yaw) = "
                f"[{self.jac_a[0, 0]:+.2f} {self.jac_a[0, 1]:+.2f}; "
                f"{self.jac_a[1, 0]:+.2f} {self.jac_a[1, 1]:+.2f}] /rad")
        return J

    # -------------------------------------------------------------- 主控制环
    def _gate_reason(self, now: float) -> str:
        """返回空串表示可动；否则返回不可动原因（用于日志与零速）。"""
        if self.target is None:
            return "no target yet"
        dt = now - self.target_t
        if dt > self.target_timeout:
            return "target stale"
        if not self.target.detected:
            return "no face"
        if not self.target.valid:
            return "target invalid (coasting)"
        # 人脸贴边时地标被裁切，中心误差会突变 → 不采信（进出画面晃动的主因之一）
        if max(abs(self.target.image_error.x), abs(self.target.image_error.y)) > self.edge_hold:
            return "face near edge"
        if self.arm_state not in ("READY", "SERVO"):
            return f"arm state={self.arm_state or '?'}"
        if self.control_mode == "ZERO_TORQUE":
            return "control_mode=ZERO_TORQUE"
        if (now - self.ee_t) > self.ee_timeout:
            return "ee_pose stale"
        z = self.target.xyz_mm.z / 1000.0
        if not (self.z_min <= z <= self.z_max):
            return "z out of range"
        return ""

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        # 1 Hz 重发视觉模式：显示层可能晚于本节点启动，错过一次性消息
        if now - self._vm_t > 1.0:
            self._publish_vision_mode()

        if not self.enabled:
            if now < self.hold_until:
                self._publish(np.zeros(3), np.zeros(3), now)
            return

        reason = self._gate_reason(now)
        if reason:
            if reason != self._last_reason:
                self.get_logger().warn(f"hold：{reason}")
                self._last_reason = reason
            self.v_cmd[:] = 0.0
            self.w_cmd[:] = 0.0
            self._est = None          # 停发 → servo 会重锚，目标估计作废
            self._stall_hist.clear()
            self._publish(self.v_cmd, self.w_cmd, now)
            return
        if self._last_reason:
            self.get_logger().info(f"恢复跟踪（原 hold：{self._last_reason}）")
            self._last_reason = ""

        # 追不动保护：停发期间 servo 已重锚，这里从实测重新起算
        if now < self._stall_until:
            self.v_cmd[:] = 0.0
            self.w_cmd[:] = 0.0
            self._publish(self.v_cmd, self.w_cmd, now)
            return
        if self._est is None and self.ee_pos is not None:
            self._est = self.ee_pos.copy()

        t = self.target
        if not self._hist:
            return
        # 逐分量中位数（抗单帧跳变）+ 一致性门：最近几帧离散太大 = 检测不稳 → 停发
        last = list(self._hist)[-self.median_n:]
        ex = float(np.median([h[0] for h in last]))
        ey = float(np.median([h[1] for h in last]))
        x_mm = float(np.median([h[2] for h in last]))
        y_mm = float(np.median([h[3] for h in last]))
        z_mm = float(np.median([h[4] for h in last]))
        spread = max(max(h[0] for h in last) - min(h[0] for h in last),
                     max(h[1] for h in last) - min(h[1] for h in last))
        if spread > self.agree_tol:
            self.v_cmd[:] = 0.0
            self.w_cmd[:] = 0.0
            self._est = None
            self._reset_err_state()
            self._stall_hist.clear()
            self._publish(self.v_cmd, self.w_cmd, now)
            self.get_logger().warn(
                f"目标不一致（最近 {len(last)} 帧离散 {spread:.3f} > {self.agree_tol}）→ 停发",
                throttle_duration_sec=2.0)
            return
        # F191：目标中心偏移——把"当前记录的位置"当作 0 误差（水平/竖向标定）
        ex -= float(self.target_err[0])
        ey -= float(self.target_err[1])
        e_xy = max(abs(ex), abs(ey))
        dz_mm = z_mm - self.z_ref * 1000.0
        depth_ok = (not self.enable_depth) or abs(dz_mm) < self.dz_dead
        # 死区带滞环：进 2.5%、出 5.5%，避免在边界上来回启停
        thr = self.deadband_exit if self._in_dead else self.deadband
        self._in_dead = e_xy < thr and depth_ok
        in_dead = self._in_dead

        # ---- F188 误差整形（I 消稳态误差 / D 超前）----
        e_vec = np.array([ex, ey])
        if self._e_prev is not None and self.dt > 0.0:
            raw = (e_vec - self._e_prev) / self.dt
            a_lp = min(1.0, self.dt / self.d_tau)      # 一阶低通，压检测噪声
            self._de_lp += a_lp * (raw - self._de_lp)
        self._e_prev = e_vec.copy()
        if in_dead:
            # 死区内不再积分，并缓慢泄漏，避免积分长期"顶着"一个偏置
            self._i_term *= max(0.0, 1.0 - self.i_leak * self.dt)
        else:
            self._i_term = np.clip(self._i_term + self.ki * e_vec * self.dt,
                                   -self.i_max, self.i_max)
        term_d = np.clip(self.kd * self._de_lp, -self.d_max, self.d_max)
        e_ctl = e_vec + self._i_term + term_d

        if in_dead:
            v_des = np.zeros(3)
            w_des = np.zeros(3)
            self._stall_hist.clear()

        # 追不动保护：窗口内 |e| 没下降，且一直在使劲 → 目标大概已顶到够不到的地方。
        # 停发 stall_hold（> servo twist_timeout）让它重锚实测，避免满速硬顶 + IK 抖动。
        else:
            en = float(np.hypot(ex, ey))
            # 无进展守卫（与速度无关）：只要"走了却没让误差变小"，就停发重锚。
            # 误差改善 → 重新计程；否则累计已发出的位移/转角，超限即判"空转"。
            if self._prog_e_best is None or en < self._prog_e_best - self.stall_progress:
                self._prog_e_best = en
                self._prog_travel = 0.0
                self._prog_rot = 0.0
                self._prog_t0 = now
            else:
                self._prog_travel += float(np.linalg.norm(self.v_cmd)) * self.dt
                self._prog_rot += float(np.linalg.norm(self.w_cmd)) * self.dt
                span = now - self._prog_t0
                if (span > 0.5 * self.stall_window
                        and (self._prog_travel > self.stall_travel
                             or self._prog_rot > self.stall_rot)):
                    self._stall_until = now + self.stall_hold
                    self._stall_last = now
                    self._stall_hist.clear()
                    self._prog_e_best = None
                    self.v_cmd[:] = 0.0
                    self.w_cmd[:] = 0.0
                    self._est = None
                    self._reset_err_state()
                    self.get_logger().warn(
                        f"空转（无进展）：{span:.1f}s 内位移 {self._prog_travel*1e3:.0f} mm / "
                        f"转角 {self._prog_rot:.2f} rad，|e| 未降（{en:.3f}）"
                        f" → 停发 {self.stall_hold:.1f}s 重锚")
                    self._publish(np.zeros(3), np.zeros(3), now)
                    return

        if in_dead:
            pass
        elif self._use_jacobian():
            # 实测雅可比映射：v = -kp · J⁻¹ · e（J: 归一化图像/米）
            try:
                v_lat = -self.kp * np.linalg.solve(self.jac, e_ctl)
            except np.linalg.LinAlgError:
                v_lat = np.zeros(2)
            # 转动跟踪时以"转"为主：平移只做残差补偿，避免双环叠加过冲
            if self.enable_angular:
                v_lat *= self.lin_frac
            # 雅可比正比于 1/Z：人脸走近/走远时按 Z/Z0 缩放，避免"近了过冲、远了迟钝"。
            # Z 只用比值 → 深度单位未标定也不影响（比例误差抵消）。
            if self.jac_z0 is not None and z_mm > 50.0:
                zr = float(np.clip((z_mm / 1000.0) / self.jac_z0, 0.5, 2.5))
                v_lat *= zr
            # 角速度（标定帧）：绕 Y = pitch，绕 Z = yaw。
            # 有实测角雅可比时用它解耦；否则用"画面右→绕 Z 反向"的解析近似。
            if self.enable_angular:
                if self.jac_a is not None:
                    try:
                        wyz = -self.ka * np.linalg.solve(self.jac_a, e_ctl)
                    except np.linalg.LinAlgError:
                        wyz = np.zeros(2)
                    w_cal = np.array([0.0, wyz[0], wyz[1]])
                else:
                    w_cal = np.array([0.0, -self.ka * ey, -self.ka * ex])
            else:
                w_cal = np.zeros(3)
            if self.r_cal is not None:
                # v2：J 的 Y/Z 与 depth 的 X 都定义在**标定帧**，再按当前末端姿态转到 base。
                # 好处：L1 转了 90° 后"后退/左右"会跟着末端一起转，不会打到错误轴上。
                v_cal = np.array([0.0, v_lat[0], v_lat[1]])
                if self.enable_depth:
                    e_d = self.z_ref - z_mm / 1000.0      # >0 = 太近，需要后退
                    if abs(e_d) > self.dz_dead / 1000.0:
                        v_cal[0] = float(np.clip(self.kz * e_d * self.depth_dir,
                                                 -self.v_max, self.v_max))
                v_des = self.r_base_ee @ (self.r_cal.T @ v_cal)
                w_des = self.r_base_ee @ (self.r_cal.T @ w_cal)
            else:
                # 旧格式（frame=base）：X 固定不动，角速度按 base 解释
                v_des = np.array([0.0, v_lat[0], v_lat[1]])
                w_des = w_cal
        else:
            v_cam = np.array([
                self.kp * x_mm / 1000.0,
                self.kp * y_mm / 1000.0,
                (self.kz * dz_mm / 1000.0) if self.enable_depth else 0.0,
            ])
            w_cam = np.zeros(3)
            if self.enable_angular:
                w_cam = np.array([-self.ka * ey, self.ka * ex, 0.0])
            # 相机系 → base 系（眼在手上；角度来自 cam_rpy_deg 解析外参）
            r_base_cam = self.r_base_ee @ self.r_ee_cam
            v_des = r_base_cam @ v_cam
            w_des = r_base_cam @ w_cam

        # 重新捕获后增益从 0 线性升到 1（track_age 在丢脸后归零）：
        # 人脸从画面边上重新出现时不会立刻按满速冲过去，消除"进出画面晃动"
        scale = min(1.0, max(0.0, t.track_age_s / self.settle_time))
        v_des *= scale
        w_des *= scale

        # 工作空间虚拟墙：贴边时不再朝外推（否则目标被夹在边界面而实测跟不上，
        # 目标越积越远 → IK 大量失败、手臂乱扭）。也顺带避免"往上够不到还硬顶"。
        # 判据用「估计的伺服目标」而非实测位姿：目标跑飞时实测往往还停在原地。
        wall_ref = self._est if self._est is not None else self.ee_pos
        clamped = []
        if wall_ref is not None:
            for i in range(3):
                if v_des[i] > 0.0 and wall_ref[i] >= self.ws_max[i] - self.ws_margin:
                    v_des[i] = 0.0
                    clamped.append("xyz"[i] + "+")
                elif v_des[i] < 0.0 and wall_ref[i] <= self.ws_min[i] + self.ws_margin:
                    v_des[i] = 0.0
                    clamped.append("xyz"[i] + "-")
        if clamped:
            self.get_logger().warn(
                f"工作空间挡板：{'/'.join(clamped)} 方向已置零（目标≈{np.round(wall_ref, 3)}）",
                throttle_duration_sec=2.0)

        # ---- 俯仰辅助：竖向平移无解时改由"抬腕/低头"继续追竖向误差 ----
        # 触发（同时满足）：pitch_assist 开、整体转动环关、竖向误差出死区，
        #   且竖向平移被虚拟墙挡死（clamped 含 z±）或最近发生过"追不动"。
        z_blocked = any(c.startswith("z") for c in clamped)
        recent_stall = (self.pitch_stall_win > 0.0
                        and (now - self._stall_last) < self.pitch_stall_win)
        if (self.pitch_assist and not self.enable_angular
                and (z_blocked or recent_stall) and abs(ey) > self.deadband_exit):
            wp = float(np.clip(-self.pitch_gain * ey, -self.pitch_max, self.pitch_max))
            if self.r_cal is not None:
                w_des = w_des + self.r_base_ee @ (self.r_cal.T @ np.array([0.0, wp, 0.0]))
            else:
                w_des = w_des + np.array([0.0, wp, 0.0])
            self.get_logger().warn(
                f"俯仰辅助：竖向平移受限（{'wall' if z_blocked else 'stall'}），"
                f"e_v={ey:+.3f} → w_pitch={wp:+.3f} rad/s",
                throttle_duration_sec=2.0)

        # F187 关节限位软门：平移和转动都可能把某个关节顶到限位，整拍一起收缩
        gs = self._limit_scale(v_des, w_des)
        if gs < 1.0:
            v_des = v_des * gs
            w_des = w_des * gs
            self.get_logger().warn(
                f"接近关节限位 → 整拍命令 ×{gs:.2f}", throttle_duration_sec=1.0)

        n = float(np.linalg.norm(v_des))
        if n > self.v_max:
            v_des *= self.v_max / n
        # 逐拍斜坡（线速度）
        dv = v_des - self.v_cmd
        max_dv = self.ramp * self.dt
        if np.linalg.norm(dv) > max_dv:
            dv *= max_dv / float(np.linalg.norm(dv))
            v_des = self.v_cmd + dv
        # 角速度限幅 + 斜坡（转动跟踪）
        nw = float(np.linalg.norm(w_des))
        if nw > self.w_max:
            w_des *= self.w_max / nw
        dw = w_des - self.w_cmd
        max_dw = self.w_ramp * self.dt
        if np.linalg.norm(dw) > max_dw:
            dw *= max_dw / float(np.linalg.norm(dw))
            w_des = self.w_cmd + dw
        self.v_cmd = v_des
        self.w_cmd = w_des
        # 累计「伺服目标」估计（只有真的在发非零速度时才推进）
        if self._est is not None:
            self._est = self._est + self.v_cmd * self.dt

        self._publish(self.v_cmd, self.w_cmd, now)

    def _publish(self, v: np.ndarray, w: np.ndarray, now: float) -> None:
        msg = TwistStamped()
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int((now - int(now)) * 1e9)
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, w)
        self.pub.publish(msg)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = FaceTrackController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前补一帧 0，避免下游保持最后一拍速度
        try:
            node._publish(np.zeros(3), np.zeros(3),
                          node.get_clock().now().nanoseconds * 1e-9)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
