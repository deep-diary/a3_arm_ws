#!/usr/bin/env python3
"""F194 统一视觉显示：原始画面 + 结构化人脸标注 → /a3/vision/display。

为什么要这一层（而不是让检测节点直接出标注图）：
  - 检测节点（mediapipe）跑 ~10-15 Hz，若由它出 640x480 标注图，下游只能看到
    ~10 Hz 且要传 ~900 KB/帧；改为只发 ~1 KB 的 FaceOverlay（关键点/连线/框/文本），
    显示层用**原始 25 Hz 画面**做主底，视觉上流畅、带宽小、分辨率不降。
  - 所有标注（网格点/网格线/人脸轮廓/人脸框/画面中心/人脸中心/连线/文本）都在
    这一层画，统一管理；关掉跟踪时直接透传原始画面。

输入：
  /camera/image_raw      sensor_msgs/Image   (bgr8, BEST_EFFORT)
  /a3/face_overlay       a3_msgs/FaceOverlay (RELIABLE)
  /a3/vision/show_mesh   std_msgs/Bool       (网格/框开关，中心与中心连线始终保留)
  /a3/vision/mode        std_msgs/String     (face_track / off → 关时透传原图)
  /a3/ee_pose            geometry_msgs/PoseStamped (base_link→end_effector，**非跟踪时**左上角文本)
  /a3/teleop/mode        std_msgs/String     (当前操作模式，右上角第二行)
输出：
  /a3/vision/display     sensor_msgs/Image   (bgr8, BEST_EFFORT)

颜色约定（BGR，标签/数值分层）：
  画面中心十字 = 淡青；人脸中心 = 琥珀（跟踪中）/ 绿（误差进死区），中心连线弱白；
  左上角文本块（两种模式同款）：标签灰、位置/尺寸近白、角度青，块下垫 40% 黑底；
  跟踪时 4 行 = x/y/z(mm) + roll/pitch/yaw(deg) + ex/ey，非跟踪时 3 行 = x/y/z(m) + RPY；
  右上角：第一行帧率（绿）、第二行操作模式（跟踪中琥珀）；网格/轮廓/框整体压暗。
"""

from __future__ import annotations

import array
import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

try:
    from a3_msgs.msg import FaceOverlay
except ImportError:
    print("[x] 找不到 a3_msgs.msg.FaceOverlay：先 colcon build --packages-select a3_msgs")
    raise

# ── 配色（BGR）──────────────────────────────────────────────────────────────
# 组织原则：**标签灰 / 数值亮 / 角度青**，状态只用于"点/数值"本身，几何线压暗——
# 这样一屏里文字始终最跳眼，网格与轮廓退成背景，不会互相抢。
LABEL = (184, 178, 170)            # 灰   RGB(170,178,184)：字段名（x: / roll: …）
VALUE = (246, 242, 238)            # 近白 RGB(238,242,246)：位置/尺寸数值
ANGLE = (250, 200, 90)             # 青   RGB(90,200,250) ：角度数值
FPS_COLOR = (150, 225, 110)        # 绿   RGB(110,225,150)：帧率
MODE_TRACK = (60, 170, 255)        # 琥珀 RGB(255,170,60) ：跟踪中 / 操作模式
STATE_LOCK = (130, 220, 90)        # 绿   RGB(90,220,130) ：误差进死区（已到位）
STATE_TRACK = MODE_TRACK           # 琥珀：跟踪中
STATE_ERR = (80, 80, 255)          # 红   RGB(255,80,80)  ：无人脸
CENTER_COLOR = (235, 220, 120)     # 淡青 RGB(120,220,235)：画面中心十字
LINK_COLOR = (205, 200, 195)       # 弱白：中心 ↔ 人脸中心连线
MESH_COLOR = (80, 110, 60)         # 暗绿：网格线（压暗，退成背景）
POINT_COLOR = (95, 140, 70)        # 暗亮绿：网格点
SILHOUETTE_COLOR = (90, 190, 200)  # 暗金：人脸轮廓
BOX_COLOR = (200, 150, 120)        # 灰蓝：人脸框
DARK = (0, 0, 0)
PLATE_ALPHA = 0.45                 # 文本块底衬不透明度


def quat_to_rpy(x: float, y: float, z: float, w: float):
    """四元数 → (roll, pitch, yaw) 弧度，约定 R = Rz·Ry·Rx（与 face_track_controller 一致）。

    只取 RPY 显示：单个四元数分量（尤其 w）没有独立物理含义，人看不出姿态。
    """
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return (0.0, 0.0, 0.0)
    x, y, z, w = x / n, y / n, z / n, w / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


class VisionDisplayMux(Node):
    def __init__(self) -> None:
        super().__init__("vision_display_mux")

        self.declare_parameter("raw_topic", "/camera/image_raw")
        self.declare_parameter("overlay_topic", "/a3/face_overlay")
        self.declare_parameter("output_topic", "/a3/vision/display")
        self.declare_parameter("show_mesh_topic", "/a3/vision/show_mesh")
        # 跟踪开关（face_track_controller 发布）：off 时直接透传原始画面
        self.declare_parameter("vision_mode_topic", "/a3/vision/mode")
        # 人脸标注超过此时长没更新 → 视为跟踪已关，透传原图（避免残留旧框）
        self.declare_parameter("overlay_timeout_s", 1.0)
        # 非跟踪（正常）模式：左上角叠加末端位姿（F175 /a3/ee_pose）
        self.declare_parameter("ee_pose_topic", "/a3/ee_pose")
        self.declare_parameter("draw_ee_pose", True)
        self.declare_parameter("ee_timeout_s", 1.0)
        # 末端工作半径 R=|p| 的告警线（与 servo 的 reach_max 0.50 对齐；接近即变色）
        self.declare_parameter("reach_warn_m", 0.48)
        # CPU 负载显示：控制环 200 Hz + SocketCAN 读写线程，CPU 一满就反馈掉帧 / TX 队列满 /
        # 命令台阶（2026-10-26 实测 load 28、空闲 2.5% 时三种症状同时出现）。
        self.declare_parameter("show_cpu", True)
        self.declare_parameter("cpu_warn_pct", 60.0)     # 黄：开始挤压控制环
        self.declare_parameter("cpu_danger_pct", 80.0)   # 红：已出现掉帧/猛动的量级
        # 右下角第二行：当前操作模式（ps4_mapper 1 Hz 重发）
        self.declare_parameter("teleop_mode_topic", "/a3/teleop/mode")
        self.declare_parameter("qos_depth", 5)
        self.declare_parameter("draw_fps", True)
        self.declare_parameter("show_mesh", True)
        # 与 face_track.yaml 的 deadband（0.05）对齐：误差进死区即"到位"变绿
        self.declare_parameter("settle_err_u", 0.05)
        self.declare_parameter("settle_err_v", 0.05)

        p = self.get_parameter
        self._overlay_timeout = float(p("overlay_timeout_s").value)
        self._fps_on = bool(p("draw_fps").value)
        self._show_mesh = bool(p("show_mesh").value)
        self._settle_u = abs(float(p("settle_err_u").value))
        self._settle_v = abs(float(p("settle_err_v").value))
        self._draw_ee = bool(p("draw_ee_pose").value)
        self._ee_timeout = float(p("ee_timeout_s").value)
        self._reach_warn = abs(float(p("reach_warn_m").value))
        self._show_cpu = bool(p("show_cpu").value)
        self._cpu_warn = abs(float(p("cpu_warn_pct").value))
        self._cpu_danger = max(self._cpu_warn, abs(float(p("cpu_danger_pct").value)))
        self._cpu_pct = 0.0
        self._load1 = 0.0
        self._stat_prev = None
        self._nproc = os.cpu_count() or 1

        depth = int(p("qos_depth").value)
        img_qos = QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        rel_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE)

        self._overlay = None
        self._overlay_t = 0.0
        self._mode = None          # None=未收到模式话题（按标注新鲜度决定）
        self._ee_xyz = None
        self._ee_rpy = None
        self._ee_t = 0.0
        self._teleop_mode = ""
        self._frames = 0
        self._out_fps = 0.0
        self._t_rate = time.monotonic()

        self.pub = self.create_publisher(Image, str(p("output_topic").value), img_qos)
        self.create_subscription(Image, str(p("raw_topic").value), self._on_raw, img_qos)
        self.create_subscription(FaceOverlay, str(p("overlay_topic").value),
                                 self._on_overlay, rel_qos)
        self.create_subscription(Bool, str(p("show_mesh_topic").value),
                                 self._on_show_mesh, rel_qos)
        self.create_subscription(String, str(p("vision_mode_topic").value),
                                 self._on_mode, rel_qos)
        self.create_subscription(PoseStamped, str(p("ee_pose_topic").value),
                                 self._on_ee, rel_qos)
        self.create_subscription(String, str(p("teleop_mode_topic").value),
                                 self._on_teleop_mode, rel_qos)
        self.get_logger().info(
            f"vision_display_mux 就绪：{p('raw_topic').value} + "
            f"{p('overlay_topic').value} → {p('output_topic').value}")

    # ------------------------------------------------------------------ 订阅
    def _on_overlay(self, msg: FaceOverlay) -> None:
        self._overlay = msg
        self._overlay_t = time.monotonic()

    def _on_show_mesh(self, msg: Bool) -> None:
        self._show_mesh = bool(msg.data)
        self.get_logger().info(f"网格/人脸框 → {'开' if self._show_mesh else '关'}")

    def _on_mode(self, msg: String) -> None:
        if msg.data != self._mode:
            self._mode = msg.data
            self.get_logger().warn(
                f"视觉模式 → {msg.data}（{'叠加标注' if self._tracking() else '透传原图'}）")

    def _on_ee(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self._ee_xyz = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self._ee_rpy = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._ee_t = time.monotonic()

    def _on_teleop_mode(self, msg: String) -> None:
        self._teleop_mode = msg.data

    def _tracking(self) -> bool:
        """是否处于"跟踪开"状态；模式话题未出现时按标注新鲜度决定。"""
        return self._mode is None or self._mode == "face_track"

    def _on_raw(self, msg: Image) -> None:
        if msg.encoding != "bgr8":
            self.get_logger().warn(
                f"仅支持 bgr8，收到 {msg.encoding}", throttle_duration_sec=10.0)
            return
        h, w = msg.height, msg.width
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if buf.size < h * msg.step:
            return
        frame = buf.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)

        ov = None
        if (self._tracking() and self._overlay is not None
                and (time.monotonic() - self._overlay_t) <= self._overlay_timeout):
            ov = self._overlay
        self._publish(self._render(frame, ov), msg.header)

    # ------------------------------------------------------------------ 绘制
    def _render(self, frame: np.ndarray, ov) -> np.ndarray:
        img = np.ascontiguousarray(frame)
        h, w = img.shape[:2]
        scale = max(0.42, 0.5 * (w / 640.0))
        thick = max(1, int(round(scale * 2)))

        if ov is not None:
            self._draw_face(img, ov, w, h, scale)

        # 画面中心十字（始终显示，青）
        cv2.drawMarker(img, (w // 2, h // 2), CENTER_COLOR, cv2.MARKER_CROSS,
                       int(26 * scale / 0.5), thick)

        if ov is not None and ov.detected:
            color = self._state_color(ov)
            cu = int(round(ov.center_norm.x * w))
            cv = int(round(ov.center_norm.y * h))
            # 中心 ↔ 人脸中心连线：一眼看出偏了多少
            cv2.line(img, (w // 2, h // 2), (cu, cv), LINK_COLOR, thick, cv2.LINE_AA)
            # 人脸中心：实心圆 + 十字（实心圆保证在网格上也能看清状态色）
            r = max(4, int(6 * scale / 0.5))
            cv2.circle(img, (cu, cv), r, color, -1, cv2.LINE_AA)
            cv2.circle(img, (cu, cv), r + 1, DARK, 1, cv2.LINE_AA)
            cv2.drawMarker(img, (cu, cv), color, cv2.MARKER_CROSS,
                           int(24 * scale / 0.5), thick)
            self._draw_face_block(img, ov, scale, thick, color)
        elif ov is not None:
            self._draw_no_face(img, scale, thick)

        # 非跟踪（正常）模式：左上角叠加末端位姿——关掉跟踪时最想看的量
        if (self._draw_ee and not self._tracking() and self._ee_xyz is not None
                and (time.monotonic() - self._ee_t) <= self._ee_timeout):
            self._draw_ee_pose(img, scale, thick)

        if self._fps_on:
            self._draw_fps(img, w, h, scale, thick)
        return img

    def _state_color(self, ov: FaceOverlay):
        """误差进死区 → 绿（已到位），否则琥珀（跟踪中）。"""
        if abs(ov.err_u) <= self._settle_u and abs(ov.err_v) <= self._settle_v:
            return STATE_LOCK
        return STATE_TRACK

    def _draw_face(self, img: np.ndarray, ov: FaceOverlay, w: int, h: int,
                   scale: float) -> None:
        pts = np.asarray(ov.landmarks, dtype=np.float32)
        if pts.size < 2:
            return
        px = pts.reshape(-1, 2) * np.array([w, h], dtype=np.float32)

        if self._show_mesh and len(ov.edges) >= 2:
            # mediapipe 官方三角剖分（2556 条，左右对称）。一次性 polylines 画完；
            # 用 LINE_8 而不是 LINE_AA：全量边的 AA 版本实测 ~29 ms/帧，非 AA 只要 ~7 ms。
            e = np.asarray(ov.edges, dtype=np.int32).reshape(-1, 2)
            ok = (e >= 0).all(axis=1) & (e < len(px)).all(axis=1)
            e = e[ok]
            if e.size:
                segs = px[e].astype(np.int32)          # (N, 2, 2)
                cv2.polylines(img, list(segs), False, MESH_COLOR, 1, cv2.LINE_8)
            # 网格点：直接写像素（比逐点 cv2.circle 快一个数量级）
            xi = np.rint(px[:, 0]).astype(np.int32)
            yi = np.rint(px[:, 1]).astype(np.int32)
            vis = (xi >= 0) & (xi < w) & (yi >= 0) & (yi < h)
            img[yi[vis], xi[vis]] = POINT_COLOR

        if len(ov.silhouette) >= 6:
            sil = (np.asarray(ov.silhouette, dtype=np.float32)
                   .reshape(-1, 2) * np.array([w, h], dtype=np.float32)).astype(np.int32)
            cv2.polylines(img, [sil.reshape(-1, 1, 2)], True, SILHOUETTE_COLOR,
                          max(2, int(2 * scale / 0.5)), cv2.LINE_AA)

        if self._show_mesh and ov.bbox_w_norm > 0.0 and ov.bbox_h_norm > 0.0:
            bw = ov.bbox_w_norm * w
            bh = ov.bbox_h_norm * h
            x0 = int(ov.center_norm.x * w - bw / 2.0)
            y0 = int(ov.center_norm.y * h - bh / 2.0)
            cv2.rectangle(img, (x0, y0), (int(x0 + bw), int(y0 + bh)),
                          BOX_COLOR, max(1, int(round(1 * scale / 0.5))), cv2.LINE_AA)

    # ------------------------------------------------------------------ 文本
    def _plate(self, img: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> None:
        """文本底衬：40% 黑，亮背景（白墙/屏幕）下也能读。"""
        h, w = img.shape[:2]
        x0, x1 = max(0, min(x0, w - 1)), max(0, min(x1, w))
        y0, y1 = max(0, min(y0, h - 1)), max(0, min(y1, h))
        if x1 <= x0 or y1 <= y0:
            return
        roi = img[y0:y1, x0:x1]
        cv2.addWeighted(np.zeros_like(roi), PLATE_ALPHA, roi, 1.0 - PLATE_ALPHA, 0.0, roi)

    def _block(self, img: np.ndarray, rows, scale: float, thick: int) -> None:
        """左上角两列对齐文本块。

        rows = [(左标签, 左值, 左值色, 右标签|None, 右值, 右值色), …]
        两列的**值**都对齐到各自固定 x（Hershey 不是等宽字体，靠空格对不齐）；
        标签统一灰色、数值按语义着色——同一行里"名、值"颜色不同，扫读快。
        """
        sf = scale / 0.5
        u = lambda v: int(round(v * sf))
        x0, y0, gap, dy = u(10), u(26), u(5), u(20)
        fnt = cv2.FONT_HERSHEY_SIMPLEX

        def tw(t: str) -> int:
            return cv2.getTextSize(t, fnt, scale, thick)[0][0]

        lw1 = max(tw(r[0]) for r in rows) + gap
        vw1 = max(tw(r[1]) for r in rows)
        col2 = x0 + lw1 + vw1 + u(16)
        right = [r for r in rows if r[3]]
        lw2 = max((tw(r[3]) for r in right), default=0) + gap
        vw2 = max((tw(r[4]) for r in right), default=0)

        x1 = col2 + lw2 + vw2 + gap if right else x0 + lw1 + vw1 + gap
        self._plate(img, x0 - gap, y0 - u(18), x1, y0 + dy * (len(rows) - 1) + u(8))
        for i, (l1, v1, c1, l2, v2, c2) in enumerate(rows):
            yy = y0 + i * dy
            self._put(img, l1, (x0, yy), scale, LABEL, thick)
            self._put(img, v1, (x0 + lw1, yy), scale, c1, thick)
            if l2:
                self._put(img, l2, (col2, yy), scale, LABEL, thick)
                self._put(img, v2, (col2 + lw2, yy), scale, c2, thick)

    def _reach_row(self):
        """R = |p|（base_link → 末端），两种模式都显示：超过 reach_warn 变琥珀。

        用途：直接对着伺服侧的 `reach_max`（0.50 m 半径硬限）看——跟踪到边界时会停在
        球面上，R 会贴着告警线不再增长。
        """
        if self._ee_xyz is None or (time.monotonic() - self._ee_t) > self._ee_timeout:
            return None
        r = math.sqrt(sum(v * v for v in self._ee_xyz))
        color = MODE_TRACK if r > self._reach_warn else VALUE
        return ("r:", f"{r * 1000.0:5.0f} mm", color, None, "", VALUE)

    def _draw_ee_pose(self, img: np.ndarray, scale: float, thick: int) -> None:
        """非跟踪：末端位置（base_link，**mm**）+ 姿态（RPY，deg），3 行两列。

        单位用 mm：工作空间 <1 m，m 的小数位在现场读起来别扭（0.132 vs 132 mm）。
        不显示四元数：单个分量（尤其 w）没有独立物理含义；RPY 才是能对着
        RViz / 示教点位核对的值（约定 R = Rz·Ry·Rx，roll/pitch/yaw 即绕基座
        X/Y/Z 的三次旋转，与 `tf2::getRPY` 同源）。
        """
        x, y, z = self._ee_xyz
        roll, pitch, yaw = (math.degrees(a) for a in self._ee_rpy)
        rows = [
            ("x:", f"{x * 1000.0:+5.0f} mm", VALUE, "roll:", f"{roll:+6.1f} deg", ANGLE),
            ("y:", f"{y * 1000.0:+5.0f} mm", VALUE, "pitch:", f"{pitch:+6.1f} deg", ANGLE),
            ("z:", f"{z * 1000.0:+5.0f} mm", VALUE, "yaw:", f"{yaw:+6.1f} deg", ANGLE),
        ]
        r_row = self._reach_row()
        if r_row is not None:
            rows.append(r_row)
        self._block(img, rows, scale, thick)

    def _draw_face_block(self, img: np.ndarray, ov: FaceOverlay, scale: float,
                         thick: int, state_color) -> None:
        """跟踪中：相机系 XYZ（mm）+ 姿态（deg）+ 图像误差 + 末端半径 R（与 EE 块同款）。"""
        rows = [
            ("x:", f"{ov.x_mm:+5.0f} mm", VALUE, "roll:", f"{ov.roll_deg:+6.1f} deg", ANGLE),
            ("y:", f"{ov.y_mm:+5.0f} mm", VALUE, "pitch:", f"{ov.pitch_deg:+6.1f} deg", ANGLE),
            ("z:", f"{ov.z_mm:5.0f} mm", VALUE, "yaw:", f"{ov.yaw_deg:+6.1f} deg", ANGLE),
            ("ex:", f"{ov.err_u:+.3f}", state_color, "ey:", f"{ov.err_v:+.3f}", state_color),
        ]
        r_row = self._reach_row()
        if r_row is not None:
            rows.append(r_row)
        self._block(img, rows, scale, thick)

    def _draw_no_face(self, img: np.ndarray, scale: float, thick: int) -> None:
        sf = scale / 0.5
        x0, y0 = int(round(10 * sf)), int(round(26 * sf))
        text = "NO FACE"
        tw = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)[0][0]
        self._plate(img, x0 - int(round(5 * sf)), y0 - int(round(18 * sf)),
                    x0 + tw + int(round(5 * sf)), y0 + int(round(8 * sf)))
        self._put(img, text, (x0, y0), scale, STATE_ERR, thick)

    def _put(self, img: np.ndarray, text: str, org, scale: float, color,
             thick: int) -> None:
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, DARK,
                    thick + 2, cv2.LINE_AA)
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                    thick, cv2.LINE_AA)

    def _update_cpu(self) -> None:
        """每秒一次：CPU 占用率（/proc/stat 差分）+ 1 分钟 load average。

        为什么要显示：控制环是 200 Hz + SocketCAN 读写线程，CPU 一满就会
        反馈掉帧（stale）/ TX 队列满（CAN write failed）/ dt 抖动（命令台阶）。
        2026-10-26 实测 load 28、空闲 2.5% 时三种症状同时出现，帧率也从 24 掉到 10。
        """
        try:
            with open("/proc/stat", "r") as f:
                vals = [int(x) for x in f.readline().split()[1:]]
            idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
            total = sum(vals)
            if self._stat_prev is not None:
                di = idle - self._stat_prev[0]
                dtot = total - self._stat_prev[1]
                if dtot > 0:
                    self._cpu_pct = max(0.0, min(100.0, 100.0 * (1.0 - di / dtot)))
            self._stat_prev = (idle, total)
            with open("/proc/loadavg", "r") as f:
                self._load1 = float(f.read().split()[0])
        except Exception:  # noqa: BLE001
            pass

    def _draw_fps(self, img: np.ndarray, w: int, h: int, scale: float,
                  thick: int) -> None:
        """右上角两行右对齐：帧率、当前模式（不带 `mode` 前缀）。

        第二行 = 操作模式（`/a3/teleop/mode`：normal / track / teach…）；
        人脸跟踪若是**服务单独**开的（操作模式没跟着切），补 `+track`，
        避免"画面在跟踪、右上角写 normal"的错觉。
        """
        mode = self._teleop_mode or (self._mode if self._mode else "-")
        if self._tracking() and "track" not in mode:
            mode += " +track"
        sf = scale / 0.5
        u = lambda v: int(round(v * sf))
        fnt = cv2.FONT_HERSHEY_SIMPLEX
        lines = [(f"{self._out_fps:.1f} fps", FPS_COLOR),
                 (mode, MODE_TRACK if self._tracking() else VALUE)]
        if self._show_cpu:
            cpu_color = (STATE_ERR if self._cpu_pct >= self._cpu_danger
                         else MODE_TRACK if self._cpu_pct >= self._cpu_warn
                         else FPS_COLOR)
            lines.append((f"cpu {self._cpu_pct:.0f}%  ld {self._load1:.1f}/{self._nproc}",
                          cpu_color))
        x_right, y = w - u(12), u(26)
        widest = max(cv2.getTextSize(t, fnt, scale, thick)[0][0] for t, _ in lines)
        self._plate(img, x_right - widest - u(5), y - u(18), w - u(3),
                    y + u(20) * (len(lines) - 1) + u(8))
        for text, color in lines:
            tw = cv2.getTextSize(text, fnt, scale, thick)[0][0]
            self._put(img, text, (x_right - tw, y), scale, color, thick)
            y += u(20)

    # ------------------------------------------------------------------ 发布
    def _publish(self, img: np.ndarray, header) -> None:
        out = Image()
        out.header = header
        h, w = img.shape[:2]
        out.height, out.width = h, w
        out.encoding = "bgr8"
        out.is_bigendian = False
        out.step = w * 3
        # array.array('B')：直接赋 bytes 会走逐元素转换，640x480 掉到个位数帧率（LL-168）
        out.data = array.array("B", img.tobytes())
        self.pub.publish(out)

        self._frames += 1
        now = time.monotonic()
        span = now - self._t_rate
        if span >= 1.0:
            self._out_fps = self._frames / span
            self._frames = 0
            self._t_rate = now
            if self._show_cpu:
                self._update_cpu()
            if self._fps_on:
                overlay_state = ("overlay" if (self._tracking() and self._overlay is not None
                                               and (now - self._overlay_t) <= self._overlay_timeout)
                                 else "passthrough")
                self.get_logger().info(
                    f"display {self._out_fps:.1f} fps ({overlay_state}, "
                    f"mesh {'on' if self._show_mesh else 'off'})",
                    throttle_duration_sec=5.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionDisplayMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
