#!/usr/bin/env python3
"""F183 人脸检测节点：sensor_msgs/Image → a3_msgs/FaceTarget。

  /camera/image_raw (bgr8, BEST_EFFORT) → mediapipe FaceLandmarker(478 点)
      → 6 个控制参数：X/Y/Z(mm) + YAW/PITCH/ROLL(deg)
      → One-Euro 滤波 + 外点剔除 + coasting
      → /a3/face_target (a3_msgs/FaceTarget)

设计依据（见 docs/edge/requirements/F183-face-tracking-servo.md）：
  - 姿态用 mediapipe 自带的 facial_transformation_matrix；手工 6 点 solvePnP 在近距
    （<0.5m）会解出 ±47° 镜像解，仅保留在离线脚本里作对比。
  - 深度必须用 depth_units_mm / depth_scale 标定一次（单目绝对尺度）。
  - 本节点必须跑在 ~/.a3/venv/face_track（mediapipe），且 venv 的 site-packages 要在
    PYTHONPATH 最前，否则系统 pyparsing 2.4.7 会压过 venv 版本导致导入失败。launch 已处理。
"""

from __future__ import annotations

import array
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

try:
    import cv2
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision
except ImportError as exc:  # noqa: BLE001
    print(f"[x] 导入失败：{exc}\n"
          "    本节点需要 mediapipe（装在 ~/.a3/venv/face_track），并用 "
          "scripts/setup_face_track_env.sh 准备环境。", file=sys.stderr)
    raise

try:
    from a3_msgs.msg import FaceOverlay, FaceTarget
except ImportError:  # 未编译 a3_msgs 时给出明确提示
    print("[x] 找不到 a3_msgs.msg.FaceTarget/FaceOverlay："
          "先 colcon build --packages-select a3_msgs", file=sys.stderr)
    raise


# F194：网格三角剖分连线（mediapipe 官方 FACEMESH_TESSELATION 全量 2556 条）。
# 注意不能做 `sorted(...)[::N]` 这类抽样：按字典序抽会把左右对称的边抽得不一致，
# 显示出来的网格就会"一侧密一侧疏"（用户 2026-10-25 反馈）。显示层用 LINE_8
# 画全量只需 ~7 ms/帧，没必要为省 CPU 破坏对称性。
try:
    import mediapipe as _mp
    MESH_EDGES = [int(v) for pair in _mp.solutions.face_mesh.FACEMESH_TESSELATION
                  for v in pair]
except Exception:  # noqa: BLE001
    MESH_EDGES = []

# 人脸椭圆轮廓（按顺序闭合）：显示层画 mask 边界
FACE_OVAL_IDX = (10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288,
                 397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136,
                 172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109)


# ------------------------------------------------------------------ 一阶/One-Euro
class OneEuro:
    """低通 + 速度自适应截止：静止强平滑去抖，快速运动时自动放松不滞后。"""

    def __init__(self, freq: float, min_cutoff: float = 1.0, beta: float = 0.05,
                 d_cutoff: float = 1.0) -> None:
        self.freq = max(1e-3, freq)
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev: float | None = None
        self.y: float | None = None
        self.dy: float | None = None

    @staticmethod
    def _alpha(cutoff: float, freq: float) -> float:
        tau = 1.0 / (2.0 * math.pi * max(1e-6, cutoff))
        return 1.0 / (1.0 + tau * freq)

    def __call__(self, x: float) -> float:
        if self.x_prev is None:
            self.x_prev, self.y, self.dy = x, x, 0.0
            return x
        dx = (x - self.x_prev) * self.freq
        a_d = self._alpha(self.d_cutoff, self.freq)
        self.dy = dx if self.dy is None else self.dy + a_d * (dx - self.dy)
        cutoff = self.min_cutoff + self.beta * abs(self.dy)
        a = self._alpha(cutoff, self.freq)
        self.y = x if self.y is None else self.y + a * (x - self.y)
        self.x_prev = x
        return self.y

    def reset(self) -> None:
        self.x_prev = self.y = self.dy = None


def pose_from_matrix(m: np.ndarray) -> tuple[float, float, float]:
    """mediapipe 姿态阵 → (距离[规范单位], yaw, pitch)[deg]。

    R 的第三列是人脸朝向；mediapipe 的 y 轴向上（光学系向下），故 pitch 取其 y 分量正值。
    roll 由调用方用双眼连线在图像里求（更直观）。
    """
    rot = m[:3, :3]
    t = m[:3, 3]
    n = rot @ np.array([0.0, 0.0, 1.0])
    yaw = math.degrees(math.atan2(n[0], n[2]))
    pitch = math.degrees(math.atan2(n[1], math.hypot(n[0], n[2])))
    return float(np.linalg.norm(t)), yaw, pitch


class FaceDetectNode(Node):
    def __init__(self) -> None:
        super().__init__("face_detect_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/a3/face_target")
        # F194：结构化标注（关键点/连线/轮廓/框/文本），由显示层叠加到原始画面上
        self.declare_parameter("overlay_topic", "/a3/face_overlay")
        self.declare_parameter("publish_overlay", True)
        self.declare_parameter("debug_image", False)
        self.declare_parameter("debug_topic", "/a3/face_track/debug_image")
        self.declare_parameter("model_path", "~/.a3/models/face_landmarker.task")
        self.declare_parameter("hfov_deg", 60.0)
        # 深度标定：z_mm = 规范单位 × depth_units_mm × depth_scale
        self.declare_parameter("depth_units_mm", 10.0)
        self.declare_parameter("depth_scale", 1.0)
        self.declare_parameter("min_cutoff", 1.0)
        self.declare_parameter("filter_beta", 0.05)
        self.declare_parameter("coast_timeout_s", 0.30)
        self.declare_parameter("max_rate_hz", 20.0)
        # 负载优化：默认只在跟踪使能（/a3/vision/mode == face_track）时跑 MediaPipe。
        # 未使能时仍保留节点与最新帧，服务一开即可用，但不付推理成本。
        self.declare_parameter("vision_mode_topic", "/a3/vision/mode")
        self.declare_parameter("require_tracking_mode", True)
        self.declare_parameter("min_confidence", 0.4)
        self.declare_parameter("jump_reject_frac", 0.25)
        # F191 前后（深度）标定：把脸放在已知距离处按一下，按 z 中位数缩放 depth_units_mm
        self.declare_parameter("calib_known_distance_mm", 200.0)
        self.declare_parameter("calib_file", "~/.a3/camera/face_calib.yaml")

        p = self.get_parameter
        self.image_topic = p("image_topic").value
        self.debug_image = bool(p("debug_image").value)
        self.publish_overlay = bool(p("publish_overlay").value)
        self.hfov = float(p("hfov_deg").value)
        self._units = float(p("depth_units_mm").value)
        self._scale = float(p("depth_scale").value)
        self.units_mm = self._units * self._scale
        self.coast_timeout = float(p("coast_timeout_s").value)
        self.min_conf = float(p("min_confidence").value)
        self.jump_frac = float(p("jump_reject_frac").value)
        min_cutoff = float(p("min_cutoff").value)
        beta = float(p("filter_beta").value)
        max_rate = max(1.0, float(p("max_rate_hz").value))

        model_path = os.path.expanduser(str(p("model_path").value))
        if not os.path.isfile(model_path):
            self.get_logger().error(
                f"模型不存在：{model_path}\n"
                "  运行 scripts/setup_face_track_env.sh 下载 face_landmarker.task")

        sensor_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        rel_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(FaceTarget, str(p("output_topic").value), rel_qos)
        # 标注图：发布器常建、按 debug_image 开关决定是否发布 → 可运行时 ros2 param set 打开
        self.pub_dbg = self.create_publisher(Image, str(p("debug_topic").value), sensor_qos)
        self.pub_ov = self.create_publisher(
            FaceOverlay, str(p("overlay_topic").value), rel_qos)
        self.add_on_set_parameters_callback(self._on_params)
        # F191：前后（深度）标定——按已知距离缩放 depth_units_mm
        self._z_hist = deque(maxlen=20)
        self._calib_dist = float(p("calib_known_distance_mm").value)
        self._calib_file = str(p("calib_file").value)
        self.create_service(Trigger, "/a3/face_track/calibrate_depth",
                            self._on_calibrate_depth)

        self._lock = threading.Lock()
        self._frame = None            # (stamp_ns, bgr ndarray)
        self._seq = 0
        self._consumed = 0
        self._stop = threading.Event()
        self._new_frame = threading.Event()

        self._freq = max_rate
        self._min_period = 1.0 / max_rate
        self._last_proc_t = 0.0
        self._f_u = OneEuro(self._freq, min_cutoff, beta)
        self._f_v = OneEuro(self._freq, min_cutoff, beta)
        self._f_z = OneEuro(self._freq, min_cutoff, beta)
        self._f_yaw = OneEuro(self._freq, min_cutoff, beta)
        self._f_pitch = OneEuro(self._freq, min_cutoff, beta)
        self._f_roll = OneEuro(self._freq, min_cutoff, beta)
        self._last_center: tuple[float, float] | None = None
        self._hold = dict(u=0.0, v=0.0, z=0.0, yaw=0.0, pitch=0.0, roll=0.0)
        self._have_hold = False
        self._last_detect_t = 0.0
        self._require_mode = bool(p("require_tracking_mode").value)
        self._vision_on = not self._require_mode
        self._track_start_t: float | None = None
        self._n_out = 0
        self._t_rate = time.monotonic()
        self._dbg_fps = 0.0          # 标注图帧率（EMA）
        self._dbg_t = 0.0

        self.create_subscription(Image, self.image_topic, self._on_image, sensor_qos)
        self.create_subscription(
            String, str(p("vision_mode_topic").value), self._on_vision_mode, rel_qos)

        opts = vision.FaceLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            running_mode=vision.RunningMode.VIDEO,
            num_faces=1,
            min_face_detection_confidence=self.min_conf,
            min_face_presence_confidence=self.min_conf,
            min_tracking_confidence=self.min_conf,
            output_facial_transformation_matrixes=True,
        )
        self._lm = vision.FaceLandmarker.create_from_options(opts)

        self._thread = threading.Thread(target=self._worker, name="face_detect", daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"face_detect 就绪：{self.image_topic} → {p('output_topic').value}，"
            f"模型 {os.path.basename(model_path)}，深度 {self.units_mm:.1f} mm/单位，"
            f"限速 {max_rate:.0f} Hz")

    # ---------------------------------------------------------------- 参数回调
    def _on_params(self, params):
        for prm in params:
            if prm.name == "debug_image":
                self.debug_image = bool(prm.value)
                self.get_logger().info(
                    f"标注图发布 → {'开' if self.debug_image else '关'}")
            elif prm.name in ("depth_units_mm", "depth_scale"):
                if prm.name == "depth_units_mm":
                    self._units = float(prm.value)
                else:
                    self._scale = float(prm.value)
                self.units_mm = self._units * self._scale
                self.get_logger().warn(f"深度比例 → {self.units_mm:.2f} mm/单位")
        return SetParametersResult(successful=True)

    # ---------------------------------------------------------------- 订阅回调
    def _on_vision_mode(self, msg: String) -> None:
        on = str(msg.data) == "face_track"
        if on != self._vision_on:
            self._vision_on = on
            if on:
                # 重新使能：清掉上次停用前的 coasting/滤波历史
                self._last_detect_t = 0.0
                self._have_hold = False
            self.get_logger().info(f"人脸检测 → {'开' if on else '空闲'}")

    def _on_image(self, msg: Image) -> None:
        if msg.encoding != "bgr8":
            self.get_logger().warn(f"仅支持 bgr8，收到 {msg.encoding}", once=True)
            return
        h, w = msg.height, msg.width
        buf = np.frombuffer(msg.data, dtype=np.uint8)   # array.array 支持 buffer 协议，零拷贝
        if buf.size < h * msg.step:
            return
        frame = buf.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self._lock:
            self._frame = (stamp_ns, frame)   # 只留最新帧，旧帧直接丢
            self._seq += 1
        self._new_frame.set()

    # ------------------------------------------------------------------- 主循环
    def _worker(self) -> None:
        while not self._stop.is_set():
            if not self._new_frame.wait(timeout=0.2):
                continue
            self._new_frame.clear()
            with self._lock:
                item, seq = self._frame, self._seq
            if item is None or seq == self._consumed:
                continue
            self._consumed = seq
            stamp_ns, frame = item
            try:
                self._process(stamp_ns, frame)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"处理帧失败：{exc}", throttle_duration_sec=2.0)

    def _process(self, stamp_ns: int, frame: np.ndarray) -> None:
        if not self._vision_on:
            return          # 跟踪未使能：不跑 MediaPipe（省 ~30% 单核）
        now_m = time.monotonic()
        if now_m - self._last_proc_t < self._min_period:
            return          # max_rate_hz 节流（相机 25 Hz，检测上限按参数）
        self._last_proc_t = now_m
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self._lm.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), stamp_ns // 1_000_000)

        msg = FaceTarget()
        msg.header = Header()
        msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        msg.header.frame_id = "camera_optical_frame"

        landmarks = res.face_landmarks[0] if res.face_landmarks else None
        now = time.monotonic()
        if landmarks is None:
            msg.detected = False
            coasting = (now - self._last_detect_t) < self.coast_timeout and self._last_detect_t > 0
            msg.valid = bool(coasting)      # coasting 期内沿用旧值，但标记未验证
            self._fill_held(msg, w, h, coasting)
            self.pub.publish(msg)
            self._publish_overlay(msg.header, None, msg)
            if self.debug_image:
                self._publish_debug(frame, None, msg)
            return

        self._last_detect_t = now
        if self._track_start_t is None:
            self._track_start_t = now

        xs = [pt.x * w for pt in landmarks]
        ys = [pt.y * h for pt in landmarks]
        bx0, bx1, by0, by1 = min(xs), max(xs), min(ys), max(ys)
        cu, cv_ = 0.5 * (bx0 + bx1), 0.5 * (by0 + by1)

        z = yaw = pitch = float("nan")
        if res.facial_transformation_matrixes:
            dist_u, yaw, pitch = pose_from_matrix(
                np.array(res.facial_transformation_matrixes[0]).reshape(4, 4))
            z = dist_u * self.units_mm          # mm
        e_l, e_r = landmarks[33], landmarks[263]
        roll = math.degrees(math.atan2((e_r.y - e_l.y) * h, (e_r.x - e_l.x) * w))

        # 跳变剔除：中心突然跳 >25% 画宽 → 认为换脸/误检，重置滤波
        if self._last_center is not None and abs(cu - self._last_center[0]) > self.jump_frac * w:
            for f in (self._f_u, self._f_v, self._f_z, self._f_yaw, self._f_pitch, self._f_roll):
                f.reset()
        self._last_center = (cu, cv_)

        fu, fv = self._f_u(cu), self._f_v(cv_)
        if math.isfinite(z):
            self._hold["z"] = self._f_z(z)
            self._hold["yaw"] = self._f_yaw(yaw)
            self._hold["pitch"] = self._f_pitch(pitch)
            self._hold["roll"] = self._f_roll(roll)
        self._hold["u"], self._hold["v"] = fu, fv
        self._have_hold = True

        fx = (w / 2.0) / math.tan(math.radians(self.hfov) / 2.0)
        cx, cy = w / 2.0, h / 2.0
        zf = self._hold["z"]
        msg.detected = True
        msg.valid = math.isfinite(zf)
        if msg.valid:
            self._z_hist.append(float(zf))   # F191 深度标定样本
        msg.confidence = 1.0   # tasks API 不暴露单脸分数，检出即 1.0（仅作占位）
        msg.center_norm.x, msg.center_norm.y = fu / w, fv / h
        msg.xyz_mm.x = (fu - cx) * zf / fx if msg.valid else 0.0
        msg.xyz_mm.y = (fv - cy) * zf / fx if msg.valid else 0.0
        msg.xyz_mm.z = zf if msg.valid else 0.0
        msg.yaw_deg = self._hold["yaw"]
        msg.pitch_deg = self._hold["pitch"]
        msg.roll_deg = self._hold["roll"]
        msg.image_error.x = (fu - cx) / (w / 2.0)
        msg.image_error.y = (fv - cy) / (h / 2.0)
        msg.bbox_w_norm = (bx1 - bx0) / w
        msg.bbox_h_norm = (by1 - by0) / h
        msg.track_age_s = now - self._track_start_t
        self.pub.publish(msg)
        self._publish_overlay(msg.header, landmarks, msg)

        if self.debug_image:
            self._publish_debug(frame, landmarks, msg)

        self._n_out += 1
        if now - self._t_rate >= 5.0:
            self.get_logger().info(
                f"检出 {self._n_out / (now - self._t_rate):.1f} Hz；"
                f"X {msg.xyz_mm.x:+.0f} Y {msg.xyz_mm.y:+.0f} Z {msg.xyz_mm.z:.0f} mm，"
                f"yaw {msg.yaw_deg:+.1f} pitch {msg.pitch_deg:+.1f} roll {msg.roll_deg:+.1f}°")
            self._n_out, self._t_rate = 0, now

    def _fill_held(self, msg: FaceTarget, w: int, h: int, coasting: bool) -> None:
        """丢帧/coasting：沿用上一次滤波值并标记 valid。"""
        if not coasting or not self._have_hold:
            msg.center_norm.x = msg.center_norm.y = 0.0
            msg.image_error.x = msg.image_error.y = 0.0
            msg.xyz_mm.x = msg.xyz_mm.y = msg.xyz_mm.z = 0.0
            msg.yaw_deg = msg.pitch_deg = msg.roll_deg = 0.0
            msg.track_age_s = 0.0
            self._track_start_t = None
            self._have_hold = False
            return
        cx, cy = w / 2.0, h / 2.0
        fx = (w / 2.0) / math.tan(math.radians(self.hfov) / 2.0)
        zf = self._hold["z"]
        msg.center_norm.x, msg.center_norm.y = self._hold["u"] / w, self._hold["v"] / h
        msg.image_error.x = (self._hold["u"] - cx) / cx
        msg.image_error.y = (self._hold["v"] - cy) / cy
        msg.xyz_mm.x = (self._hold["u"] - cx) * zf / fx
        msg.xyz_mm.y = (self._hold["v"] - cy) * zf / fx
        msg.xyz_mm.z = zf
        msg.yaw_deg = self._hold["yaw"]
        msg.pitch_deg = self._hold["pitch"]
        msg.roll_deg = self._hold["roll"]
        msg.track_age_s = (time.monotonic() - self._track_start_t) if self._track_start_t else 0.0

    def _on_calibrate_depth(self, _req: Trigger.Request,
                            resp: Trigger.Response) -> Trigger.Response:
        """F191 前后（深度）标定：脸放已知距离，按当前 z 中位数缩放 depth_units_mm。

        TRACK 模式「Square 长按」触发。距离基准是参数 calib_known_distance_mm
        （默认 200 mm）——把人脸/标定板放到该距离再按。
        """
        known = self._calib_dist
        if known <= 0.0:
            resp.success = False
            resp.message = "calib_known_distance_mm 必须 > 0"
            return resp
        if len(self._z_hist) < 5:
            resp.success = False
            resp.message = (f"深度样本不足（{len(self._z_hist)}/5）：把脸放到 "
                            f"{known:.0f} mm 处并保持约 1 s 再按")
            return resp
        z_med = float(np.median(self._z_hist))
        if not math.isfinite(z_med) or z_med <= 1.0:
            resp.success = False
            resp.message = f"深度读数无效（z={z_med:.1f}）"
            return resp
        old = self._units
        self._units *= known / z_med
        self.units_mm = self._units * self._scale
        path = self._persist_depth()
        resp.success = True
        resp.message = (f"depth_units_mm {old:.2f} → {self._units:.2f}"
                        f"（z {z_med:.0f} mm @ 已知 {known:.0f} mm）"
                        + (f" → {path}" if path else "（未持久化）"))
        self.get_logger().warn(resp.message)
        self._z_hist.clear()
        return resp

    def _persist_depth(self) -> str:
        """把 depth_units_mm/depth_scale 写回 ~/.a3/camera/face_calib.yaml（保留其它键）。"""
        fp = os.path.expanduser(self._calib_file)
        try:
            data = {}
            try:
                with open(fp, encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
            except FileNotFoundError:
                pass
            block = data.setdefault("/face_detect_node", {})
            params = block.setdefault("ros__parameters", {})
            params["depth_units_mm"] = float(self._units)
            params["depth_scale"] = float(self._scale)
            os.makedirs(os.path.dirname(fp), exist_ok=True)
            tmp = fp + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
            os.replace(tmp, fp)
            return fp
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"写深度标定失败（{exc}）")
            return ""

    def _publish_overlay(self, header, landmarks, msg: FaceTarget) -> None:
        """F194：把标注要素打包给显示层（不含像素图，~1 KB/帧）。"""
        if not self.publish_overlay:
            return
        ov = FaceOverlay()
        ov.header = header
        if landmarks is None:
            ov.detected = False
            self.pub_ov.publish(ov)
            return
        ov.detected = True
        flat = []
        for pt in landmarks:
            flat.append(float(pt.x))
            flat.append(float(pt.y))
        ov.landmarks = flat
        ov.edges = MESH_EDGES
        sil = []
        for i in FACE_OVAL_IDX:
            sil.append(float(landmarks[i].x))
            sil.append(float(landmarks[i].y))
        ov.silhouette = sil
        ov.center_norm.x = msg.center_norm.x
        ov.center_norm.y = msg.center_norm.y
        ov.bbox_w_norm = msg.bbox_w_norm
        ov.bbox_h_norm = msg.bbox_h_norm
        ov.x_mm, ov.y_mm, ov.z_mm = msg.xyz_mm.x, msg.xyz_mm.y, msg.xyz_mm.z
        ov.yaw_deg, ov.pitch_deg, ov.roll_deg = (
            msg.yaw_deg, msg.pitch_deg, msg.roll_deg)
        ov.err_u, ov.err_v = msg.image_error.x, msg.image_error.y
        self.pub_ov.publish(ov)

    def _publish_debug(self, frame: np.ndarray, landmarks, msg: FaceTarget) -> None:
        img = frame.copy()
        h, w = img.shape[:2]
        CYAN = (255, 255, 0)     # BGR：青
        DARK = (0, 0, 0)

        def text(t: str, org, scale: float = 0.55, color=CYAN, thick: int = 1) -> None:
            cv2.putText(img, t, org, cv2.FONT_HERSHEY_SIMPLEX, scale, DARK, thick + 2, cv2.LINE_AA)
            cv2.putText(img, t, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)

        if landmarks is not None:
            for pt in landmarks:
                cv2.circle(img, (int(pt.x * w), int(pt.y * h)), 1, (0, 200, 0), -1)
        cv2.drawMarker(img, (w // 2, h // 2), (230, 230, 230), cv2.MARKER_CROSS, 22, 1)
        if msg.detected:
            cu, cvv = int(msg.center_norm.x * w), int(msg.center_norm.y * h)
            cv2.drawMarker(img, (cu, cvv), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
            cv2.arrowedLine(img, (cu, cvv), (w // 2, h // 2), (0, 200, 255), 1,
                            cv2.LINE_AA, tipLength=0.06)
            # 控制目标：u、v → 0（人脸居中），Z → z_ref（固定距离）
            # 注意 cv2.putText 的 Hershey 字体只有 ASCII，不能写中文
            lines = [
                f"XYZ   X {msg.xyz_mm.x:+5.0f} | Y {msg.xyz_mm.y:+5.0f} | "
                f"Z {msg.xyz_mm.z:4.0f} mm",
                f"HEAD  YAW {msg.yaw_deg:+5.1f} | PITCH {msg.pitch_deg:+5.1f} | "
                f"ROLL {msg.roll_deg:+5.1f} deg",
                f"ERR   u {msg.image_error.x:+.3f} | v {msg.image_error.y:+.3f}  "
                f"(0=center){'' if msg.valid else '  COASTING'}",
            ]
            for i, t in enumerate(lines):
                text(t, (10, 24 + i * 22))
        else:
            text("NO FACE", (10, 24), 0.7, (0, 0, 255), 2)
        out = Image()
        out.header = msg.header
        out.height, out.width = h, w
        out.encoding = "bgr8"
        out.is_bigendian = False
        out.step = w * 3
        out.data = array.array("B", img.tobytes())   # 同 usb_camera_node：bytes 会掉到个位数帧率
        self.pub_dbg.publish(out)

    def destroy_node(self) -> None:
        self._stop.set()
        self._new_frame.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        try:
            self._lm.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = FaceDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
