# F182 — 独立相机采集小 UI（PyQt5 实时预览 + 拍照 + 录制）

- **说明：** F177 已把 USB 摄像头接进 ROS（`/camera/image_raw`），但取素材还得开 RViz 或写脚本。
  本需求提供一个**脱离 ROS 栈**的独立桌面小工具：直接打开 V4L2 设备实时预览，一键拍照、一键开始/停止
  录制，产物按时间戳落盘，供后续人脸算法（检测/识别/数据集）用。选 PyQt5 是因为板子已装
  `python3-opencv` + `PyQt5`（`DISPLAY=:0`），无需新增依赖，也不需要起 `a3_bringup` 占用相机。

- **验收标准：**
  1. `python3 scripts/camera_capture_ui.py` 直接出窗口，左侧/主区域实时预览相机画面，画面正常非花屏/全黑
  2. 「拍照」按钮（或 `Space`）保存当前帧为 JPEG 到 `<输出目录>/images/img_YYYYmmdd_HHMMSS_mmm.jpg`；
     保存的是**原始帧**（不带预览里的帧率/REC 叠加文字），状态栏回显完整路径
  3. 「开始录制」→ 按钮变「停止录制」（或按 `R`）→ 产物 `<输出目录>/videos/vid_YYYYmmdd_HHMMSS.avi`
     （MJPG，帧率跟随设备），停止后状态栏回显路径、帧数与时长；录制中预览角标显示 `REC` 与已录时长
  4. 默认输出目录 `capture/face/`（仓库根，`images/`/`videos/` 自动创建、已 gitignore）；
     `--out` 可改，例如按人分目录
  5. 设备默认自动选中 `/dev/v4l/by-id/*-video-index0`（稳定路径，插拔不跳号），可用 `--device` 覆盖；
     `--width/--height/--fps/--fourcc` 可调（本机 MJPG 640×480@25、1280×720@30）
  6. 设备打不开/被占用时窗口仍正常出，状态栏给出明确原因与提示（真机栈 `usb_camera_node` 会占相机），
     不崩溃、不刷屏

- **实现：**
  - `scripts/camera_capture_ui.py`：`CaptureWorker(QThread)` 独占 `cv2.VideoCapture(CAP_V4L2)` 读帧，
    最新帧加锁共享给 GUI；录制写盘也在该线程内做（不抢 GUI 主线程），`QTimer` 30 Hz 拉最新帧刷新预览；
    预览叠加（帧率 / REC 角标）只画在**显示副本**上，拍照保存原始帧
  - 输出目录默认 `<repo>/capture/face`，`/capture/` 入 `.gitignore`
  - 全程不依赖 rclpy：与 F177 的 `usb_camera_node` 二选一使用（同时开会报设备占用）

- **关联：** F177（ROS 侧 `usb_camera_node` / `/camera/image_raw`）、
  [LL-168](../../lessons_learned/LL-168-usb-camera-apt-pin-and-rclpy-bytes.md)（设备 by-id 路径与分辨率/帧率绑定）、
  F151/F150（后续人脸/检测素材来源）

- **状态：** `implemented`（2026-10-23；PyQt5 独立窗口，直接 V4L2 取流，拍照/录制落 `capture/face/`）
