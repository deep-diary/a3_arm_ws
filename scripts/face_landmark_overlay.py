#!/usr/bin/env python3
"""离线视频 → MediaPipe FaceLandmarker → 6 自由度控制参数叠加可视化。

用途：不启动机械臂，用一段实拍视频先验证「人脸检测 / 关键点 / 单目深度 /
转头角 / 滤波效果」，输出带标注的视频，便于肉眼判断参数是否可用。

运行（mediapipe 装在隔离 venv，且必须清掉本仓 PYTHONPATH 顺序，
否则系统 pyparsing 2.4.7 会抢在 venv 前面导致 mediapipe 导入失败）：

    env -u PYTHONPATH ~/.a3/venv/face_track/bin/python \
        scripts/face_landmark_overlay.py capture/face/videos/vid_20261023_200148.avi

常用参数：
    --z-ref 0.35        期望人脸距离 [m]，用来看 dz
    --units-mm 10       姿态矩阵距离单位换算（默认按 cm，见文末深度标定）
    --depth-scale 1.0   额外深度修正系数
    --pose-source pnp   改用 6 点 solvePnP（近距有镜像解风险，仅作对比）
    --no-filter         关闭 One-Euro，对比原始抖动
    --show              额外弹窗实时播放（需要 DISPLAY）
    --save-frames 3     导出若干标注帧 PNG 便于检查

输出：默认与输入同目录 <name>_overlay.mp4，可用 --out 指定。

坐标与参数约定（符号已用实拍视频验证）：
  - 相机光学系：x 右、y 下、z 前
  - 6 个控制参数 = X/Y/Z (mm，人脸中心相对光轴) + YAW/PITCH/ROLL (deg，人脸朝向)
    yaw>0 人脸朝画面右侧；pitch>0 抬头；roll>0 头部向画面右下歪
  - 另外打印相机系建议 twist (vx, vy, vz)，即外环控制器要发的速度

姿态来源（重要）：
  默认用 mediapipe 自带 facial_transformation_matrix。实测 6 点手工模型 solvePnP
  在近距（<0.5m）会出 ±47° 镜像解、重投影误差 8~12px；自带矩阵在正脸帧上 yaw
  稳定在 ±6° 内且与实际朝向一致，故 PnP 仅保留作对比。

深度标定：
  单目绝对深度依赖人脸规范模型的尺度，必须标一次。把脸放到已知距离 D（如 200mm），
  读出当前 Z 读数 R，调 --units-mm / --depth-scale 使读数变为 D。
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import time
from collections import deque

import cv2
import numpy as np

try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision
except ImportError as exc:  # noqa: BLE001
    sys.exit(f"[x] 无法导入 mediapipe（{exc}）。\n"
             "    请用隔离 venv 运行：env -u PYTHONPATH ~/.a3/venv/face_track/bin/python ...")

# ----------------------------------------------------------------- 人脸几何模型
# PnP 兜底用的 6 点模型（mm）：原点=两眼外角中点，x 右、y 下、z 朝相机。
# 比例按外眼角间距 90mm、鼻尖前凸 20mm、眼→下巴 96mm 取实测值。
# 注意：近距（<0.5m）+ 少点数时 PnP 会出现镜像解，默认不用它，见 --pose-source。
FACE_MODEL_MM = np.array([
    [0.0, 24.0, 20.0],      # 0 鼻尖     landmark 1
    [0.0, 96.0, 5.0],       # 1 下巴     landmark 152
    [-45.0, 0.0, 0.0],      # 2 左眼外角 landmark 33
    [45.0, 0.0, 0.0],       # 3 右眼外角 landmark 263
    [-27.0, 66.0, 8.0],     # 4 左嘴角   landmark 61
    [27.0, 66.0, 8.0],      # 5 右嘴角   landmark 291
], dtype=np.float64)
FACE_LM_IDX = [1, 152, 33, 263, 61, 291]
FACE_LM_NAME = ["nose", "chin", "eyeL", "eyeR", "mouthL", "mouthR"]

# 绘制用：眼/嘴/眉轮廓（只取少量稳定子集，避免 478 点糊成一片）
OUTLINE_IDX = {
    "face_oval": [10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288, 397,
                  365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136, 172, 58,
                  132, 93, 234, 127, 162, 21, 54, 103, 67, 109, 10],
}

C_TEXT = (240, 240, 240)
C_OK = (120, 255, 120)
C_WARN = (60, 200, 255)
C_BAD = (90, 90, 255)
C_RAW = (150, 150, 150)
C_UV = (0, 220, 255)
C_Z = (255, 160, 60)


def fnum(v: float, spec: str) -> str:
    return "  n/a " if not math.isfinite(v) else format(v, spec)


# --------------------------------------------------------------------- 滤波器
class LowPass:
    def __init__(self) -> None:
        self.y = None

    def __call__(self, x: float, alpha: float) -> float:
        self.y = x if self.y is None else self.y + alpha * (x - self.y)
        return self.y


class OneEuro:
    """One-Euro 滤波：静止时强平滑（去抖），快速运动时自动放松（不滞后）。"""

    def __init__(self, freq: float, min_cutoff: float = 1.0, beta: float = 0.05,
                 d_cutoff: float = 1.0) -> None:
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev: float | None = None
        self.dx_lp = LowPass()
        self.x_lp = LowPass()

    @staticmethod
    def _alpha(cutoff: float, freq: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau * freq)

    def __call__(self, x: float) -> float:
        if self.x_prev is None:
            self.x_prev = x
            self.x_lp.y = x
            self.dx_lp.y = 0.0
            return x
        dx = (x - self.x_prev) * self.freq
        dx_hat = self.dx_lp(dx, self._alpha(self.d_cutoff, self.freq))
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        x_hat = self.x_lp(x, self._alpha(cutoff, self.freq))
        self.x_prev = x
        return x_hat

    def reset(self) -> None:
        self.x_prev = None
        self.dx_lp.y = None
        self.x_lp.y = None


# ----------------------------------------------------------------------- 工具
def intrinsics_from_hfov(w: int, h: int, hfov_deg: float) -> tuple[float, float, float, float]:
    fx = (w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return fx, fx, w / 2.0, h / 2.0


def pose_from_matrix(m: np.ndarray) -> tuple[float, float, float]:
    """mediapipe facial_transformation_matrix → (距离[canonical 单位], yaw, pitch)[deg]。

    mediapipe 的旋转阵把人脸规范模型映射到相机：第三列是人脸朝向，
    但其 y 轴向上（与光学系 y 向下相反），故 pitch 直接取其 y 分量的正值。
    roll 改用图像里双眼连线求（更直观，不依赖欧拉角符号约定）。
    """
    rot = m[:3, :3]
    t = m[:3, 3]
    n = rot @ np.array([0.0, 0.0, 1.0])
    yaw = math.degrees(math.atan2(n[0], n[2]))
    pitch = math.degrees(math.atan2(n[1], math.hypot(n[0], n[2])))
    return float(np.linalg.norm(t)), yaw, pitch


def face_pose(landmarks, w: int, h: int, k: tuple[float, float, float, float],
              depth_scale: float):
    """478 点 → solvePnP → (z_m, yaw, pitch, reproj_err_px)。仅 --pose-source pnp 时使用。"""
    fx, fy, cx, cy = k
    img_pts = np.array([[landmarks[i].x * w, landmarks[i].y * h] for i in FACE_LM_IDX],
                       dtype=np.float64)
    cam_mtx = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist = np.zeros((4, 1))
    model = FACE_MODEL_MM / 1000.0
    ok, rvec, tvec = cv2.solvePnP(model, img_pts, cam_mtx, dist, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        return None
    tvec = tvec.reshape(3) * depth_scale
    rot, _ = cv2.Rodrigues(rvec)
    n = rot @ np.array([0.0, 0.0, 1.0])
    yaw = math.degrees(math.atan2(n[0], n[2]))
    pitch = math.degrees(math.atan2(-n[1], math.hypot(n[0], n[2])))
    proj, _ = cv2.projectPoints(model, rvec, tvec / depth_scale, cam_mtx, dist)
    err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)))
    return tvec, yaw, pitch, err


def draw_panel(img, lines, origin=(10, 10), font_scale=0.52, pad=8):
    """左上角半透明信息面板。"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    thick = 1
    sizes = [cv2.getTextSize(t, font, font_scale, thick)[0] for t, _c in lines]
    w = max(s[0] for s in sizes) + pad * 2
    lh = int(round(cv2.getTextSize("Ag", font, font_scale, thick)[0][1] * 1.85))
    h = lh * len(lines) + pad * 2
    x, y = origin
    roi = img[y:y + h, x:x + w]
    if roi.size:
        roi[:] = (roi * 0.25).astype(roi.dtype)
    for i, (text, color) in enumerate(lines):
        cv2.putText(img, text, (x + pad, y + pad + lh * i + lh - 8), font, font_scale,
                    color, thick, cv2.LINE_AA)
    return h


def draw_trace(img, box, series, vmin, vmax, labels):
    """底部滚动曲线：每条 series = (值列表, 颜色, 线宽)。"""
    x0, y0, bw, bh = box
    roi = img[y0:y0 + bh, x0:x0 + bw]
    if roi.size:
        roi[:] = (roi * 0.2).astype(roi.dtype)
    cv2.rectangle(img, (x0, y0), (x0 + bw, y0 + bh), (70, 70, 70), 1)
    mid = y0 + bh // 2
    cv2.line(img, (x0, mid), (x0 + bw, mid), (70, 70, 70), 1)
    span = max(1e-6, vmax - vmin)
    for vals, color, thick in series:
        if len(vals) < 2:
            continue
        pts = []
        n = len(vals)
        for i, v in enumerate(vals):
            px = x0 + int((i / max(1, n - 1)) * (bw - 1))
            py = y0 + bh - 1 - int((v - vmin) / span * (bh - 2))
            pts.append((px, max(y0 + 1, min(y0 + bh - 2, py))))
        cv2.polylines(img, [np.array(pts, np.int32)], False, color, thick, cv2.LINE_AA)
    for i, (txt, color) in enumerate(labels):
        cv2.putText(img, txt, (x0 + 6 + i * 62, y0 + 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, color, 1, cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description="MediaPipe 人脸 6DoF 参数叠加视频")
    ap.add_argument("video")
    ap.add_argument("--out", default="")
    ap.add_argument("--model", default=os.path.expanduser("~/.a3/models/face_landmarker.task"))
    ap.add_argument("--hfov", type=float, default=60.0, help="水平视场角（无内参兜底）")
    ap.add_argument("--depth-scale", type=float, default=1.0, dest="depth_scale",
                    help="深度修正系数（20cm 处或实测标定）")
    ap.add_argument("--pose-source", choices=["matrix", "pnp"], default="matrix",
                    dest="pose_source",
                    help="matrix=mediapipe 自带姿态阵（近距稳定，默认）；pnp=6 点 solvePnP")
    ap.add_argument("--units-mm", type=float, default=10.0, dest="units_mm",
                    help="matrix 模式的距离单位换算 mm/单位（默认 10，按 cm 解释；20cm 标定改这里）")
    ap.add_argument("--z-ref", type=float, default=0.35, dest="z_ref", help="期望距离 m")
    ap.add_argument("--min-cutoff", type=float, default=1.0, dest="min_cutoff")
    ap.add_argument("--beta", type=float, default=0.05)
    ap.add_argument("--deadband", type=float, default=0.025, help="死区，占画面宽比例")
    ap.add_argument("--kp", type=float, default=0.9, help="像素误差→速度增益 1/s")
    ap.add_argument("--kz", type=float, default=1.2, help="深度误差→速度增益 1/s")
    ap.add_argument("--vmax", type=float, default=0.05, help="建议 twist 限速 m/s")
    ap.add_argument("--no-filter", action="store_true", dest="no_filter")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--save-frames", type=int, default=0, dest="save_frames")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"[x] 打不开视频：{args.video}")
        return 2
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    n_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
    k = intrinsics_from_hfov(w, h, args.hfov)

    out_path = args.out or os.path.splitext(args.video)[0] + "_overlay.mp4"
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

    opts = vision.FaceLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=args.model),
        running_mode=vision.RunningMode.VIDEO,
        num_faces=1,
        min_face_detection_confidence=0.4,
        min_face_presence_confidence=0.4,
        min_tracking_confidence=0.4,
        output_facial_transformation_matrixes=True,
    )
    lm = vision.FaceLandmarker.create_from_options(opts)

    f_u = OneEuro(fps, args.min_cutoff, args.beta)
    f_v = OneEuro(fps, args.min_cutoff, args.beta)
    f_z = OneEuro(fps, args.min_cutoff, args.beta)
    f_yaw = OneEuro(fps, args.min_cutoff, args.beta)
    f_pitch = OneEuro(fps, args.min_cutoff, args.beta)
    f_roll = OneEuro(fps, args.min_cutoff, args.beta)

    hist_raw_u: deque = deque(maxlen=240)
    hist_mid_u: deque = deque(maxlen=240)
    hist_raw_v: deque = deque(maxlen=240)
    hist_mid_v: deque = deque(maxlen=240)
    hist_z: deque = deque(maxlen=240)

    # 跨帧保持的滤波后状态（丢帧时沿用上一个有效值）
    fu = fv = fz = fy = fp = fr = float("nan")
    px = py = dz = 0.0
    vx = vy = vz = 0.0
    in_dead = True
    err = float("nan")

    stats = dict(frames=0, detected=0, lost=0, zs=[], yaws=[], pitches=[], rolls=[],
                 raw_u=[], flt_u=[], raw_v=[], flt_v=[], pnp_err=[], proc_ms=[],
                 n_matrix=0, n_fallback=0)
    prev_center = None
    saved_frames: list[str] = []
    lost_run = 0
    lost_frames_limit = max(2, int(round(0.3 * fps)))   # 0.3s 内允许 coasting

    idx = 0
    t_start = time.perf_counter()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        stats["frames"] += 1
        t0 = time.perf_counter()

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        res = lm.detect_for_video(mp_img, int(idx * 1000.0 / fps))
        idx += 1

        landmarks = res.face_landmarks[0] if res.face_landmarks else None
        lines = []
        if landmarks is not None:
            stats["detected"] += 1
            lost_run = 0

            xs = [p.x * w for p in landmarks]
            ys = [p.y * h for p in landmarks]
            bx0, bx1, by0, by1 = min(xs), max(xs), min(ys), max(ys)
            # 控制用的“人脸中心”：关键点包围盒中心（比鼻尖更稳，不受转头影响）
            cu, cv_ = 0.5 * (bx0 + bx1), 0.5 * (by0 + by1)

            # ---- 姿态：默认用 mediapipe 自带矩阵（近距稳定），PnP 仅作对比 ----
            err = float("nan")
            z = yaw = pitch = float("nan")
            if args.pose_source == "matrix" and res.facial_transformation_matrixes:
                dist_u, yaw, pitch = pose_from_matrix(
                    np.array(res.facial_transformation_matrixes[0]).reshape(4, 4))
                z = dist_u * args.units_mm / 1000.0 * args.depth_scale
                stats["n_matrix"] += 1
            else:
                pose = face_pose(landmarks, w, h, k, args.depth_scale)
                if pose is not None:
                    tvec, yaw, pitch, err = pose
                    z = float(tvec[2])
                    stats["pnp_err"].append(err)
                stats["n_fallback"] += 1
            # 头部左右倾斜直接用双眼连线（图像系：roll>0 = 头向画面右下歪）
            e_l, e_r = landmarks[33], landmarks[263]
            roll = math.degrees(math.atan2((e_r.y - e_l.y) * h, (e_r.x - e_l.x) * w))
            if math.isfinite(z):
                stats["zs"].append(z)
                stats["yaws"].append(yaw)
                stats["pitches"].append(pitch)
                stats["rolls"].append(roll)

            # 跳变剔除：中心突然跳 >25% 画宽 → 重新初始化滤波
            if prev_center is not None and abs(cu - prev_center[0]) > 0.25 * w:
                for f in (f_u, f_v, f_z, f_yaw, f_pitch, f_roll):
                    f.reset()
            prev_center = (cu, cv_)
            stats["raw_u"].append(cu)
            stats["raw_v"].append(cv_)

            if args.no_filter:
                fu, fv = cu, cv_
                if math.isfinite(z):
                    fz, fy, fp, fr = z, yaw, pitch, roll
            else:
                fu, fv = f_u(cu), f_v(cv_)
                if math.isfinite(z):
                    fz, fy, fp, fr = f_z(z), f_yaw(yaw), f_pitch(pitch), f_roll(roll)
            stats["flt_u"].append(fu)
            stats["flt_v"].append(fv)

            # ---- 6 个控制参数：X/Y（相对光轴，mm）、Z（深度 mm）、YPR（deg）----
            fx, fy_, cx, cy = k
            e_norm = max(abs(fu - cx) / w, abs(fv - cy) / h)
            in_dead = e_norm < args.deadband
            if math.isfinite(fz):
                px = (fu - cx) * fz / fx * 1000.0
                py = (fv - cy) * fz / fy_ * 1000.0
                dz = (fz - args.z_ref) * 1000.0
                # 建议 twist（相机系）：把人脸往画面中心拉 + 把距离拉到 z_ref
                vx = 0.0 if in_dead else args.kp * px / 1000.0
                vy = 0.0 if in_dead else args.kp * py / 1000.0
                vz = max(-args.vmax, min(args.vmax, args.kz * dz / 1000.0))
                nv = math.hypot(vx, vy)
                if nv > args.vmax:
                    vx, vy = vx * args.vmax / nv, vy * args.vmax / nv
                hist_z.append(fz * 1000.0)

            hist_raw_u.append(cu)
            hist_mid_u.append(fu)
            hist_raw_v.append(cv_)
            hist_mid_v.append(fv)

            # ---- 绘制 ----
            for i in OUTLINE_IDX["face_oval"]:
                p = landmarks[i]
                cv2.circle(frame, (int(p.x * w), int(p.y * h)), 1, (90, 200, 90), -1)
            for p in landmarks:
                cv2.circle(frame, (int(p.x * w), int(p.y * h)), 1, (0, 200, 0), -1)
            for i, name in zip(FACE_LM_IDX, FACE_LM_NAME):
                p = landmarks[i]
                pt = (int(p.x * w), int(p.y * h))
                cv2.circle(frame, pt, 4, (0, 0, 255), -1)
                cv2.putText(frame, name, (pt[0] + 5, pt[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 255), 1, cv2.LINE_AA)

            cv2.rectangle(frame, (int(bx0), int(by0)), (int(bx1), int(by1)), (0, 200, 255), 1)
            cv2.drawMarker(frame, (int(fu), int(fv)), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)

            # 画面中心 + 死区
            cx_i, cy_i = int(cx), int(cy)
            cv2.drawMarker(frame, (cx_i, cy_i), C_TEXT, cv2.MARKER_CROSS, 24, 1)
            db_w = int(args.deadband * w)
            db_h = int(args.deadband * h)
            cv2.rectangle(frame, (cx_i - db_w, cy_i - db_h), (cx_i + db_w, cy_i + db_h),
                          (140, 140, 140), 1)
            cv2.arrowedLine(frame, (int(fu), int(fv)), (cx_i, cy_i),
                            C_OK if in_dead else C_WARN, 1, cv2.LINE_AA, tipLength=0.06)

            lines = [
                (f"FACE 6-DOF  camera frame  {'raw' if args.no_filter else '1euro'}",
                 C_TEXT),
                (f" X {fnum(px, '+7.1f')} mm     YAW   {fnum(fy, '+6.1f')} deg",
                 C_OK if in_dead else C_WARN),
                (f" Y {fnum(py, '+7.1f')} mm     PITCH {fnum(fp, '+6.1f')} deg",
                 C_OK if in_dead else C_WARN),
                (f" Z {fnum(fz * 1000.0, '7.1f')} mm     ROLL  {fnum(fr, '+6.1f')} deg",
                 C_OK if in_dead else C_WARN),
                (f" dz {fnum(dz, '+7.1f')} mm  z_ref {args.z_ref * 1000:.0f} mm"
                 f"  {'DEADBAND' if in_dead else ''}",
                 C_OK if abs(dz) < 30 else C_WARN),
                (f" twist vx{vx:+.3f} vy{vy:+.3f} vz{vz:+.3f} m/s", C_TEXT),
                (f" src {args.pose_source}"
                 + (f"  reproj {fnum(err, '4.2f')} px" if args.pose_source == "pnp"
                    else f"  z_scale {args.units_mm * args.depth_scale:.1f} mm/u"),
                 C_TEXT),
            ]
        else:
            stats["lost"] += 1
            lost_run += 1
            lines = [("FACE tracking", C_TEXT),
                     (" NO FACE" if lost_run > lost_frames_limit else " coasting...", C_BAD),
                     (f" lost {lost_run} frames", C_TEXT)]

        panel_h = draw_panel(frame, lines)
        # 底部滚动曲线（避开人脸）：左 u/v（画面中心为 0，±100px），右 z（±150mm 绕 z_ref）
        cx_i0, cy_i0 = int(k[2]), int(k[3])
        ty = h - 86
        draw_trace(frame, (10, ty, 300, 76),
                   [([v - cx_i0 for v in hist_raw_u], C_RAW, 1),
                    ([v - cx_i0 for v in hist_mid_u], C_UV, 2),
                    ([v - cy_i0 for v in hist_raw_v], (110, 110, 160), 1),
                    ([v - cy_i0 for v in hist_mid_v], (230, 130, 255), 2)],
                   -100.0, 100.0,
                   [("u raw", C_RAW), ("u flt", C_UV), ("v flt", (230, 130, 255))])
        draw_trace(frame, (320, ty, 170, 76),
                   [(list(hist_z), C_Z, 2)],
                   args.z_ref * 1000.0 - 150.0, args.z_ref * 1000.0 + 150.0,
                   [("z mm", C_Z)])

        stats["proc_ms"].append((time.perf_counter() - t0) * 1000.0)
        writer.write(frame)

        if args.save_frames and n_total:
            step = max(1, n_total // args.save_frames)
            if (stats["frames"] - 1) % step == 0 and len(saved_frames) < args.save_frames:
                p = os.path.splitext(out_path)[0] + f"_f{stats['frames']:04d}.png"
                cv2.imwrite(p, frame)
                saved_frames.append(p)

        if args.show:
            cv2.imshow("face 6dof", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

    cap.release()
    writer.release()
    lm.close()
    if args.show:
        cv2.destroyAllWindows()

    dur = time.perf_counter() - t_start
    print(f"[+] 输出视频：{out_path}")
    print(f"    帧数 {stats['frames']}（检测到人脸的帧 {stats['detected']}，"
          f"丢失 {stats['lost']}），处理 {dur:.1f}s，"
          f"平均 {1000 * dur / max(1, stats['frames']):.1f} ms/帧")
    print(f"    姿态来源：matrix {stats['n_matrix']} 帧 / 回退 {stats['n_fallback']} 帧")
    if stats["zs"]:
        print(f"    深度 Z：中位 {statistics.median(stats['zs']) * 1000:.0f} mm，"
              f"范围 {min(stats['zs']) * 1000:.0f}–{max(stats['zs']) * 1000:.0f} mm")
        print(f"    Yaw {min(stats['yaws']):+.1f}..{max(stats['yaws']):+.1f}°，"
              f"Pitch {min(stats['pitches']):+.1f}..{max(stats['pitches']):+.1f}°，"
              f"Roll {min(stats['rolls']):+.1f}..{max(stats['rolls']):+.1f}°")
        if stats["pnp_err"]:
            print(f"    PnP 重投影误差 中位 {statistics.median(stats['pnp_err']):.2f} px")
        print("    深度标定提示：把脸放到已知距离 D(mm) 处，按当前读数 R(mm) 设 "
              "--units-mm / --depth-scale，使 R×k = D")
        ru = np.array(stats["raw_u"])
        fu = np.array(stats["flt_u"])
        rv = np.array(stats["raw_v"])
        fv2 = np.array(stats["flt_v"])
        if len(ru) > 10 and not args.no_filter:
            print(f"    抖动（二阶差分 RMS，越小越稳）：u {np.std(np.diff(ru, 2)):.2f} → "
                  f"{np.std(np.diff(fu, 2)):.2f} px，v {np.std(np.diff(rv, 2)):.2f} → "
                  f"{np.std(np.diff(fv2, 2)):.2f} px")
    return 0


if __name__ == "__main__":
    sys.exit(main())
