# LL-168 — 板载 rkmpp ffmpeg 把 ROS 相机驱动全挡死；自写 cv2 节点又被 rclpy `bytes` 赋值拖到 6fps

> **日期：** 2026-10-16  
> **产品线：** Edge  
> **环境：** lubancat RK3588（Ubuntu 22.04 + ROS 2 Humble，rockchip-multimedia / panfork PPA）

## 现象

在 RK3588 板上装 USB（UVC）摄像头并用 RViz 看图像，遇到两个独立坑：

1. **装不上**：`apt-get install ros-humble-usb-cam` 报
   ```
   libavcodec-dev : 依赖: libavcodec58 (= 7:4.4.2-0ubuntu0.22.04.1)
   libavutil-dev  : 依赖: libavutil56  (= 7:4.4.2-0ubuntu0.22.04.1) 但是 7:4.4.2-0ubuntu0.22.04.1+rkmpp20230327 正要被安装
   E: 无法修正错误…
   ```
   换 `ros-humble-v4l2-camera` / `gscam` / `image-publisher` 同样失败；`--no-install-recommends` 也无效。

2. **跑不快**：改用自写 cv2 + rclpy 节点后，`ros2 topic hz` 只有 **~6 fps**（设备标称 25 fps），
   但同一设备在**独立 Python 进程**里 `cap.read()` 循环可稳定 25 fps。

## 根因

**坑 1（apt）**：`/etc/apt/preferences.d/rockchip-multimedia-ppa` 里有一条显式 pin：

```
Package: ffmpeg … libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libswresample-dev …
Pin: release o=LP-PPA-liujianfeng1994-rockchip-multimedia
Pin-Priority: 1
```

即**故意压制该 PPA 的 -dev 包**，以免拉进 ffmpeg 6 与板载 4.4.2+rkmpp 运行时冲突。
而所有 ROS 相机驱动（usb_cam / v4l2_camera / gscam / image_publisher）都经
`ros-humble-cv-bridge → libopencv-dev → libopencv-videoio-dev → libav*-dev` 硬依赖链条，
`libavcodec-dev (= 7:4.4.2-0ubuntu0.22.04.1)` 又要求把已装的 `+rkmpp20230327` 版本降级 →
**在这块板上 apt 装任何 ROS 相机驱动都不可能**（除非破坏板子的 rkmpp 视频栈，不可取）。

**坑 2（性能）**：`sensor_msgs/Image.data`（`uint8[]`）用
```python
msg.data = frame.tobytes()      # bytes 对象 → 149 ms/帧！
```
会被 rclpy 走**逐元素转换**；改用
```python
msg.data = array.array('B', frame.tobytes())   # 0.11 ms/帧
```
即可。同尺寸消息用构造器传 `data=`（`Image(..., data=b)`）也是快的，唯独「先建对象再赋 bytes」慢。
实测明细：`read 3.4 / build 179 / publish 0.4 ms`——瓶颈全在 build 这一步，`publish` 本身没问题，
所以别被「publish 慢」的直觉带偏，要做分段计时。

## 正确做法 / 规避

1. **不要在 RK3588 上试图 apt 装 ROS 相机驱动**。先查 pin：
   `cat /etc/apt/preferences.d/rockchip-multimedia-ppa`；用 `apt-cache depends <pkg>` 顺藤摸到
   `libav*-dev` 就可以直接放弃。**不要**为装驱动去降级/替换 rkmpp 版 ffmpeg。
2. 本仓方案：`a3_bringup` 自带 `usb_camera_node`（仅依赖已装好的 `python3-opencv` + rclpy），
   发布 `/camera/image_raw`(bgr8) + `/camera/camera_info`；真机栈（`a3_bringup.launch.py`）已内建
   该节点（`use_camera` 默认 true），栈 RViz 配置（`a3_description_6j` 与 `a3_description` 两包
   的 `el_a3_view.rviz`）里也加了 "USB Camera" Image display；另留 `usb_cam.launch.py` +
   `config/usb_cam.rviz` 供脱离栈单看。
   Image display **不依赖 TF**，所以栈 Fixed Frame `base_link` 下不必给相机补 TF。
   设备用 **`/dev/v4l/by-id/...-video-index0`** 稳定路径，不要写 `/dev/video20`（插拔/多摄像头会变）。
3. 写 rclpy 图像发布时**必须** `array.array('B', ...)`；任何大 `uint8[]` 字段同理。
4. 采集放独立线程，消息构建要分段计时（`read` / `build` / `publish`）再定位瓶颈。
5. 无 X 直接截图时注意：GNOME 合成器下 `import -window <root/desktop>` 会拿到**陈旧/空白**帧，
   要 `wmctrl -l` 找到目标窗口 id 后 `import -window <winid>`（本机可用，见 LL-164 同款技巧）。

## 性能与帧率（2026-10-16 实测，换机/排查帧率问题时先看这段）

**分辨率与帧率在本机设备上绑死**（`v4l2-ctl --list-formats-ext`）：

| 模式 | 设备上限 | 节点「采集+构建+发布」 | 单核 CPU |
|---|---|---|---|
| MJPG 640×480 | 25 fps | 25.0 fps | 20% |
| MJPG 1280×720 | **30 fps** | 30.0 fps | 41% |
| YUYV 1280×720 | 10 fps | — | — |

- **要 30fps 只能用 1280×720 MJPG**；640×480 无论怎么调都是 25fps（设备硬上限）。
- 自写 Python 节点的余量很大：25fps 只吃单核 20%、30fps 吃 41%（8 核板）→ **发布端不是瓶颈**。
- **瓶颈在消费端**：same-process 实测（load≈22、rviz2 软件渲染 ~244% CPU）
  `camera_info`(小) 27.8fps vs `image_raw`(2.7MB) 13.2fps → 大消息投递/反序列化被重载拖慢；
  端到端延迟中位仅 13–16ms，说明是**丢帧不是积压**。
- 推论/做法：
  - 图像 QoS 用 **sensor-data 惯例（BEST_EFFORT + KEEP_LAST 5）**，避免重载下积压；
    消费者必须同选 Best Effort（RViz Image display 的 Topic 也改了），否则 QoS 不兼容收不到。
    CLI 量速率：`ros2 topic hz /camera/image_raw --qos-reliability best_effort`。
  - **别只看 `ros2 topic hz /camera/image_raw` 判帧率**：Python 订阅端本身就可能只有 10–15fps。
    量"发布节拍"看同循环的小消息 `camera_info`。
  - 若消费端仍不够，先降发布分辨率（2.7MB→921KB）或降 rviz 渲染 Frame Rate，
    再考虑处理节点改 C++（rclpy 反序列化 2.7MB 有实打实成本）。
  - 整机负载是前提：rviz2 软件渲染单进程可占 ~2.5 核，load 长期 > 核数时帧率必然掉。

### ⚠️ RViz 里"看到的帧率"和"发布帧率"是两回事

- 图像面板左上角烧的 `pub NN.N fps`（节点参数 `overlay_fps`）= **发布帧率**（采集节拍）；
  RViz 自己的**渲染帧率**看**窗口右下角状态栏**（`xx fps`）。
- 本机实测：**软件渲染**（`LIBGL_ALWAYS_SOFTWARE=1`）全屏 3770×2096 时渲染**仅 1–2 fps**
  （缩到 1600×1000 → 4 fps）；**2026-10-16 起默认改用 panfrost 硬件 GL（LL-027 复测）→ 同场景 11–15 fps**。
  图像面板**跟着主渲染循环刷新**，所以"发布 25fps"却"看着不完全 25"是正常的，**不是相机链路的问题**。
- `Global Options / Frame Rate` 只是渲染**上限**；想提升只能从渲染侧下手：
  ① GPU 渲染（**已默认**，见 LL-027 复测：1–2 → 11–15 fps）；② 缩小窗口；③ 简化模型网格。
  排查顺序：先读右下角 fps，再决定动不动相机。

## 相关路径

- `src/a3_bringup/a3_bringup/usb_camera_node.py`
- `src/a3_bringup/launch/usb_cam.launch.py`
- `src/a3_bringup/config/usb_cam.rviz`
- `/etc/apt/preferences.d/rockchip-multimedia-ppa`（本机，不入库）
- 需求：`docs/edge/requirements/F177-usb-camera-rviz-bringup.md`
