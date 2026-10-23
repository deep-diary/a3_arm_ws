#!/usr/bin/env python3
"""F182 — 独立相机采集小工具（实时预览 / 拍照 / 录制）。

脱离 ROS 栈：直接用 OpenCV(V4L2) 打开 USB 摄像头，PyQt5 窗口显示画面。
产物默认落 <repo>/capture/face/{images,videos}/，供后续人脸算法取素材。

用法：
  python3 scripts/camera_capture_ui.py                      # 自动选 by-id 设备
  python3 scripts/camera_capture_ui.py --out capture/face/zhangsan
  python3 scripts/camera_capture_ui.py --width 1280 --height 720 --fps 30
快捷键：Space 拍照，R 开始/停止录制。

注意：真机栈的 usb_camera_node 会占用同一台相机；两者同时开只会有一个能拿到设备。
"""

import argparse
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
from PyQt5.QtCore import Qt, QThread, QTimer, QUrl, pyqtSignal
from PyQt5.QtGui import QImage, QKeySequence, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QShortcut,
    QVBoxLayout,
    QWidget,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
PREVIEW_SIZE = (720, 540)


def default_device() -> str:
    """优先 /dev/v4l/by-id 下的 index0（稳定路径，插拔/多摄不跳号），否则退回 /dev/video0。"""
    by_id = sorted(Path("/dev/v4l/by-id").glob("*video-index0"))
    return str(by_id[0]) if by_id else "/dev/video0"


class CaptureWorker(QThread):
    """独占相机读帧 + 录制写盘；最新帧加锁共享给 GUI，避免主线程阻塞。"""

    opened = pyqtSignal(int, int, float)  # 实际宽 / 高 / 帧率
    failed = pyqtSignal(str)
    record_started = pyqtSignal(str)
    record_stopped = pyqtSignal(str, int, float)  # 路径 / 帧数 / 秒
    stats = pyqtSignal(float)  # 采集帧率

    def __init__(self, device, width, height, fps, fourcc, video_dir, video_ext,
                 parent=None):
        super().__init__(parent)
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self.video_dir = Path(video_dir)
        self.video_ext = video_ext

        self._frame = None
        self._frame_lock = threading.Lock()
        self._running = True
        self._rec_path = None
        self._rec_stop = False

    def latest_frame(self):
        with self._frame_lock:
            return None if self._frame is None else self._frame.copy()

    def request_record_start(self, path):
        self._rec_path = Path(path)

    def request_record_stop(self):
        self._rec_stop = True

    def stop(self):
        self._running = False
        self.wait(2000)

    def _writer_fourcc(self):
        return cv2.VideoWriter_fourcc(*("mp4v" if self.video_ext == "mp4" else "MJPG"))

    def run(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.failed.emit(
                f"打不开相机设备：{self.device}\n"
                "可能未插好或被占用（真机栈 usb_camera_node 会占相机，先 ./scripts/a3_stack.sh stop）"
            )
            cap.release()
            return

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(cap.get(cv2.CAP_PROP_FPS) or self.fps)
        self.opened.emit(actual_w, actual_h, actual_fps)

        writer = None
        rec_path = None
        rec_frames = 0
        rec_t0 = 0.0
        fps_n = 0
        fps_t0 = time.monotonic()

        while self._running:
            ok, frame = cap.read()
            if not ok:
                self.failed.emit("读帧失败：设备掉线或被其他进程抢占")
                break
            with self._frame_lock:
                self._frame = frame

            if writer is None and self._rec_path is not None:
                path, self._rec_path = self._rec_path, None
                writer = cv2.VideoWriter(
                    str(path), self._writer_fourcc(), actual_fps, (actual_w, actual_h)
                )
                if writer.isOpened():
                    rec_path, rec_frames, rec_t0 = path, 0, time.monotonic()
                    self._rec_stop = False
                    self.record_started.emit(str(path))
                else:
                    writer = None
                    self.failed.emit(f"无法创建视频文件：{path}")

            if writer is not None:
                writer.write(frame)
                rec_frames += 1
                if self._rec_stop:
                    self._rec_stop = False
                    writer.release()
                    writer = None
                    self.record_stopped.emit(
                        str(rec_path), rec_frames, time.monotonic() - rec_t0
                    )

            fps_n += 1
            now = time.monotonic()
            if now - fps_t0 >= 0.5:
                self.stats.emit(fps_n / (now - fps_t0))
                fps_n, fps_t0 = 0, now

        if writer is not None:
            writer.release()
            self.record_stopped.emit(str(rec_path), rec_frames, time.monotonic() - rec_t0)
        cap.release()


class CaptureWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.out_dir = Path(args.out).expanduser().resolve()
        self.photo_dir = self.out_dir / "images"
        self.video_dir = self.out_dir / "videos"
        self.photo_dir.mkdir(parents=True, exist_ok=True)
        self.video_dir.mkdir(parents=True, exist_ok=True)

        self.recording = False
        self.rec_t0 = 0.0
        self.capture_fps = 0.0
        self.resolution = "—"
        self.photo_count = 0
        self.video_count = 0

        self.setWindowTitle("A3 相机采集（F182）")
        self._build_ui()

        self.worker = CaptureWorker(
            args.device,
            args.width,
            args.height,
            args.fps,
            args.fourcc,
            self.video_dir,
            args.video_ext,
        )
        self.worker.opened.connect(self._on_opened)
        self.worker.failed.connect(self._on_failed)
        self.worker.record_started.connect(self._on_record_started)
        self.worker.record_stopped.connect(self._on_record_stopped)
        self.worker.stats.connect(self._on_stats)
        self.worker.start()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(33)  # 30 Hz 拉最新帧刷新预览

        self.status.setText(f"输出目录：{self.out_dir}")

    # ---------- UI ----------

    def _build_ui(self):
        self.preview = QLabel("正在打开相机…")
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setFixedSize(*PREVIEW_SIZE)
        self.preview.setStyleSheet("background:#111;color:#888;font-size:14px;")

        self.info = QLabel("设备：—    分辨率：—    采集帧率：—")
        self.info.setStyleSheet("color:#555;")

        self.photo_btn = QPushButton("拍照 (Space)")
        self.rec_btn = QPushButton("开始录制 (R)")
        self.open_btn = QPushButton("打开目录")
        for btn in (self.photo_btn, self.rec_btn, self.open_btn):
            btn.setMinimumHeight(40)

        self.photo_btn.clicked.connect(self.take_photo)
        self.rec_btn.clicked.connect(self.toggle_record)
        self.open_btn.clicked.connect(self.open_output_dir)

        self.status = QLabel("")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("color:#333;")

        buttons = QHBoxLayout()
        buttons.addWidget(self.photo_btn)
        buttons.addWidget(self.rec_btn)
        buttons.addWidget(self.open_btn)

        root = QVBoxLayout()
        root.addWidget(self.preview, alignment=Qt.AlignCenter)
        root.addLayout(buttons)
        root.addWidget(self.info)
        root.addWidget(self.status)

        central = QWidget()
        central.setLayout(root)
        self.setCentralWidget(central)

        QShortcut(QKeySequence("Space"), self, activated=self.take_photo)
        QShortcut(QKeySequence("R"), self, activated=self.toggle_record)

    # ---------- 采集回调 ----------

    def _on_opened(self, width, height, fps):
        self.resolution = f"{width}×{height} @ {fps:.0f}fps"
        self.info.setText(
            f"设备：{self.args.device}    分辨率：{self.resolution}    采集帧率：—"
        )

    def _on_failed(self, message):
        self.status.setText(message)
        self.status.setStyleSheet("color:#c0392b;")
        self.photo_btn.setEnabled(False)
        self.rec_btn.setEnabled(False)

    def _on_stats(self, fps):
        self.capture_fps = fps
        self.info.setText(
            f"设备：{self.args.device}    分辨率：{self.resolution}    "
            f"采集帧率：{fps:.1f} fps"
        )

    def _on_record_started(self, path):
        self.recording = True
        self.rec_t0 = time.monotonic()
        self.rec_btn.setText("停止录制 (R)")
        self.rec_btn.setEnabled(True)
        self.status.setStyleSheet("color:#333;")
        self.status.setText(f"录制中 → {path}")

    def _on_record_stopped(self, path, frames, seconds):
        self.recording = False
        self.rec_btn.setText("开始录制 (R)")
        self.rec_btn.setEnabled(True)
        self.video_count += 1
        self.status.setStyleSheet("color:#333;")
        self.status.setText(
            f"已保存视频：{path}（{frames} 帧 / {seconds:.1f}s，本次第 {self.video_count} 段）"
        )

    # ---------- 交互 ----------

    def take_photo(self):
        frame = self.worker.latest_frame()
        if frame is None:
            self.status.setText("还没有画面，稍等一下…")
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = self.photo_dir / f"img_{ts}.jpg"
        ok = cv2.imwrite(str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.args.jpeg_quality])
        if ok:
            self.photo_count += 1
            self.status.setStyleSheet("color:#333;")
            self.status.setText(f"已保存照片：{path}（本次第 {self.photo_count} 张）")
        else:
            self.status.setStyleSheet("color:#c0392b;")
            self.status.setText(f"保存失败：{path}")

    def toggle_record(self):
        if self.recording:
            self.rec_btn.setEnabled(False)
            self.status.setText("正在收尾录制…")
            self.worker.request_record_stop()
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self.video_dir / f"vid_{ts}.{self.args.video_ext}"
        self.rec_btn.setEnabled(False)
        self.status.setText(f"准备录制 → {path}")
        self.worker.request_record_start(path)

    def open_output_dir(self):
        from PyQt5.QtGui import QDesktopServices

        QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.out_dir)))

    # ---------- 预览 ----------

    def _on_tick(self):
        frame = self.worker.latest_frame()
        if frame is None:
            return
        disp = frame.copy()  # 叠加只画在显示副本上，拍照保存的是原始帧
        cv2.putText(
            disp, f"{self.capture_fps:.1f} fps", (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA,
        )
        if self.recording:
            elapsed = time.monotonic() - self.rec_t0
            cv2.circle(disp, (20, 56), 8, (0, 0, 255), -1)
            cv2.putText(
                disp, f"REC {elapsed:.1f}s", (36, 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA,
            )
        rgb = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        image = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(image).scaled(
            self.preview.size(), Qt.KeepAspectRatio, Qt.FastTransformation
        )
        self.preview.setPixmap(pixmap)

    def closeEvent(self, event):
        self.timer.stop()
        self.worker.stop()
        super().closeEvent(event)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="F182 独立相机采集小工具：预览 + 拍照 + 录制（不依赖 ROS）"
    )
    parser.add_argument(
        "-d", "--device", default=None,
        help="V4L2 设备路径，默认自动选 /dev/v4l/by-id/*-video-index0",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=25.0,
                        help="本机 640×480 上限 25fps，1280×720 才能 30fps（LL-168）")
    parser.add_argument("--fourcc", default="MJPG", help="采集像素格式：MJPG / YUYV")
    parser.add_argument(
        "--out", default=str(REPO_ROOT / "capture" / "face"),
        help="输出根目录（下建 images/ 与 videos/），默认 capture/face",
    )
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--video-ext", default="avi", choices=["avi", "mp4"],
                        help="录制容器：avi(MJPG) / mp4(mp4v)")
    args = parser.parse_args(argv)
    if args.device is None:
        args.device = default_device()
    return args


def main():
    args = parse_args()
    app = QApplication([])
    window = CaptureWindow(args)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
