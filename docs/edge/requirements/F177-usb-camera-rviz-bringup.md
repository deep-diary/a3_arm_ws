# F177 — USB 摄像头 ROS 接入（`a3_bringup/usb_camera_node` + RViz 显示）

- **说明：** 板上插了一支 USB（UVC）摄像头，需要能在 RViz 里直接看到画面（后续供口风琴/末端
  对位类应用取图）。本机（RK3588）**apt 装不上任何 ROS 相机驱动**：`rockchip-multimedia` PPA 的
  pin 故意把 `libav*-dev` 压到优先级 1 以护住 rkmpp 版 ffmpeg，而 `usb_cam` / `v4l2_camera` /
  `gscam` / `image_publisher` 都经 `cv-bridge → libopencv-dev → libav*-dev` 硬依赖（详见 LL-168）。
  因此不改动板子包，改为在本仓自带一个只依赖 `python3-opencv` + rclpy 的轻量采集节点。

- **验收标准：**
  1. `ros2 launch a3_bringup usb_cam.launch.py` 后存在话题 `/camera/image_raw`
     （`sensor_msgs/Image`，`encoding=bgr8`，**默认 640×480 MJPG @25fps**；可选 1280×720@30）与
     `/camera/camera_info`；发布循环节拍以 `camera_info` 为准（实测 25.1 Hz，贴设备 25fps 上限）
  2. 图像内容正确（抓帧存 PNG 可见真实画面，非花屏/全黑）；左上角烧有实时**发布**帧率
     （`overlay_fps` 默认 true，做视觉处理时置 false 避免污染图像）
  3. **真机栈自带**：`a3_stack.sh start/restart --rviz` 起栈后，栈的 RViz（`el_a3_view.rviz`）
     Displays 里出现且勾选 `USB Camera`，左下角实时显示画面，Global Status 仍为 Ok
     （`ros2 topic info /camera/image_raw` 订阅数 ≥ 1）
  4. 设备默认走稳定路径 `/dev/v4l/by-id/...-video-index0`；可用 `camera_device` /
     `camera_frame_id` / `use_camera`（真机栈）或 `video_device` / `image_width` /
     `image_height` / `framerate` / `pixel_format` / `camera_name` / `frame_id`（单独入口）覆盖。
     **注意分辨率和帧率是绑定的**：本机设备 MJPG 1280×720 才支持 30fps，640×480 上限 25fps
  5. 相机缺失/被占用时**只告警不阻断**：节点空转，栈其余部分正常
  6. 图像/内参话题用 sensor-data 惯例 QoS（**BEST_EFFORT**、KEEP_LAST 5）：重载下不积压、
     不拖发布端；RViz Image display 的 Topic 也必须选 Best Effort（否则 QoS 不兼容收不到）

- **性能实测（2026-10-16，8 核板，load≈22、rviz2 软件渲染 ~190–244% CPU）：**
  | 指标 | 640×480 MJPG（当前默认） | 1280×720 MJPG |
  |---|---|---|
  | 设备上限 | 25 fps | 30 fps |
  | 节点「采集+构建+发布」 | 25.0 fps / 单核 20% | 30.0 fps / 单核 41% |
  | 消费端实收（BEST_EFFORT，有竞争进程） | 15.4 fps | 13.2 fps |
  → 发布端能贴住设备上限，**自写节点不是瓶颈**；重载系统上大消息投递到消费者会被拖慢，
  且实测是**丢帧不是积压**（端到端延迟中位仅 13–16 ms）。详见 LL-168「性能」一节。

- **⚠️ 别把"图像左上角的帧率"当成 RViz 显示帧率：**
  - 图像左上角 `pub NN.N fps` = 节点**发布帧率**（= 采集节拍）；
  - RViz 自己的**渲染帧率**在**窗口右下角状态栏**（`xx fps`）。本机软件渲染
    （`LIBGL_ALWAYS_SOFTWARE=1`）全屏 3770×2096 时实测**只有 1–2 fps**，窗口缩到 1600×1000
    （像素 1/5）才 4 fps → RViz 里图像面板的刷新被主渲染循环拖到个位数，观感必然远低于 25。
    图像面板是"跟着主渲染循环刷新"的，所以**发布 25fps ≠ 看到 25fps**。
  - `Global Options / Frame Rate: 20`（F177 起由 30 降为 20）只是渲染上限，当前根本跑不到上限，
    因此它不是限制因素；要提升只能从渲染侧下手（GPU 渲染 / 缩小窗口 / 简化场景），见 LL-168。

- **实现：**
  - `src/a3_bringup/a3_bringup/usb_camera_node.py`：cv2(`CAP_V4L2`) 采集 → 独立线程发布
    `Image`(bgr8) + `CameraInfo`，QoS 用 sensor-data（BEST_EFFORT/KEEP_LAST 5）；
    `msg.data` **必须**用 `array.array('B', …)`（直接赋 `bytes` 会被 rclpy 逐元素转换，
    640×480 一帧 ~150 ms → 掉到 ~6 fps，见 LL-168）；默认 640×480@25（消息 921KB，消费端压力小；
    要 30fps 切 1280×720）；`overlay_fps` 在画面左上角烧发布帧率；
    打不开设备时记 error 后空转（不抛异常，避免拖垮整栈）
  - `src/a3_bringup/launch/a3_bringup.launch.py`：**真机栈内建该节点**（`use_camera` 默认 true，
    namespace `camera`；`camera_device` 默认走 by-id 稳定路径）
  - `src/a3_description_6j/config/el_a3_view.rviz`、`src/a3_description/config/el_a3_view.rviz`：
    Displays 增加 `USB Camera`（`rviz_default_plugins/Image` → `/camera/image_raw`，raw transport；
    Image display 不依赖 TF，故栈的 Fixed Frame `base_link` 下无需相机 TF）
  - `src/a3_bringup/launch/usb_cam.launch.py` + `config/usb_cam.rviz`：脱离栈单独看相机的入口
    （额外发 `map→camera_optical_frame` 静态 TF）
  - `src/a3_bringup/setup.py` / `package.xml`：注册 `usb_camera_node` 入口，`exec_depend`
    增加 `tf2_ros`、`python3-opencv`（**不加** `usb_cam`）

- **关联：** [LL-168](../../lessons_learned/LL-168-usb-camera-apt-pin-and-rclpy-bytes.md)（板载 rkmpp pin 与
  rclpy `bytes` 赋值性能坑）、F176（口风琴实测点，后续取图应用）、
  [shared/TOPIC_CONTRACT.md](../../shared/TOPIC_CONTRACT.md)（图像话题登记）

- **状态：** `implemented`（2026-10-16；真机栈内 `usb_camera_node` 随栈启动，栈 RViz 的
  "USB Camera" 面板出图；默认 640×480@25 + BEST_EFFORT + 左上角发布帧率叠加，实测发布 25.1 Hz）
