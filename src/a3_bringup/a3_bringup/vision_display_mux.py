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
输出：
  /a3/vision/display     sensor_msgs/Image   (bgr8, BEST_EFFORT)

颜色约定（BGR）：
  画面中心十字 = 青；人脸中心 + 坐标文本 = 橙（跟踪中）/ 绿（误差进死区）；
  两点之间白色细连线，一眼看出「脸偏在哪」。
"""

from __future__ import annotations

import array
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

try:
    from a3_msgs.msg import FaceOverlay
except ImportError:
    print("[x] 找不到 a3_msgs.msg.FaceOverlay：先 colcon build --packages-select a3_msgs")
    raise

CENTER_COLOR = (255, 255, 0)      # 青：画面中心
LINK_COLOR = (255, 255, 255)      # 白：中心连线
TRACK_COLOR = (0, 165, 255)       # 橙：跟踪中（未到位）
LOCK_COLOR = (0, 255, 0)          # 绿：已到位（误差在死区内）
MESH_COLOR = (0, 150, 0)          # 深绿：网格线
POINT_COLOR = (0, 220, 0)         # 亮绿：网格点
OVAL_COLOR = (0, 255, 255)        # 黄：人脸轮廓
BOX_COLOR = (255, 160, 0)         # 蓝：人脸框
DARK = (0, 0, 0)


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

        depth = int(p("qos_depth").value)
        img_qos = QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        rel_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE)

        self._overlay = None
        self._overlay_t = 0.0
        self._mode = None          # None=未收到模式话题（按标注新鲜度决定）
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
            # 中心 ↔ 人脸中心白色连线：一眼看出偏了多少
            cv2.line(img, (w // 2, h // 2), (cu, cv), LINK_COLOR, thick, cv2.LINE_AA)
            # 人脸中心：实心圆 + 十字（实心圆保证在网格上也能看清橙/绿状态）
            r = max(4, int(6 * scale / 0.5))
            cv2.circle(img, (cu, cv), r, color, -1, cv2.LINE_AA)
            cv2.circle(img, (cu, cv), r + 1, DARK, 1, cv2.LINE_AA)
            cv2.drawMarker(img, (cu, cv), color, cv2.MARKER_CROSS,
                           int(24 * scale / 0.5), thick)
            self._draw_text(img, ov, w, h, scale, color)
        elif ov is not None:
            self._put(img, "NO FACE", (int(12 * scale / 0.5), int(30 * scale / 0.5)),
                      scale, (0, 0, 255), thick)

        if self._fps_on:
            self._draw_fps(img, w, h, scale, thick)
        return img

    def _state_color(self, ov: FaceOverlay):
        """误差进死区 → 绿（已到位），否则橙（跟踪中）。"""
        if abs(ov.err_u) <= self._settle_u and abs(ov.err_v) <= self._settle_v:
            return LOCK_COLOR
        return TRACK_COLOR

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
            cv2.polylines(img, [sil.reshape(-1, 1, 2)], True, OVAL_COLOR,
                          max(2, int(2 * scale / 0.5)), cv2.LINE_AA)

        if self._show_mesh and ov.bbox_w_norm > 0.0 and ov.bbox_h_norm > 0.0:
            bw = ov.bbox_w_norm * w
            bh = ov.bbox_h_norm * h
            x0 = int(ov.center_norm.x * w - bw / 2.0)
            y0 = int(ov.center_norm.y * h - bh / 2.0)
            cv2.rectangle(img, (x0, y0), (int(x0 + bw), int(y0 + bh)),
                          BOX_COLOR, max(1, int(round(1 * scale / 0.5))), cv2.LINE_AA)

    def _draw_text(self, img: np.ndarray, ov: FaceOverlay, w: int, h: int,
                   scale: float, color) -> None:
        thick = max(1, int(round(scale * 2)))
        lines = [
            f"XYZ  x {ov.x_mm:+5.0f} | y {ov.y_mm:+5.0f} | z {ov.z_mm:4.0f} mm",
            f"RPY  yaw {ov.yaw_deg:+5.1f} | pitch {ov.pitch_deg:+5.1f} | "
            f"roll {ov.roll_deg:+5.1f} deg",
            f"BOX  {int(ov.bbox_w_norm * w):3d} x {int(ov.bbox_h_norm * h):3d} px | "
            f"ERR  x {ov.err_u:+.3f} | y {ov.err_v:+.3f}",
        ]
        y = int(30 * scale / 0.5)
        for line in lines:
            self._put(img, line, (int(12 * scale / 0.5), y), scale, color, thick)
            y += int(22 * scale / 0.5)

    def _put(self, img: np.ndarray, text: str, org, scale: float, color,
             thick: int) -> None:
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, DARK,
                    thick + 2, cv2.LINE_AA)
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                    thick, cv2.LINE_AA)

    def _draw_fps(self, img: np.ndarray, w: int, h: int, scale: float,
                  thick: int) -> None:
        text = f"{self._out_fps:.1f} fps"
        (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
        self._put(img, text, (w - tw - int(12 * scale / 0.5),
                              int(30 * scale / 0.5)), scale, LOCK_COLOR, thick)

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
