# LL-187 — 人脸标注图两路并行：低分辨率 + 双 fps + 闪烁

> **日期：** 2026-10-25
> **产品线：** Edge
> **环境：** RK3588 lubancat + ROS 2 Humble + mediapipe（~/.a3/venv/face_track）

## 现象

- 开人脸跟踪时 RViz 里画面明显变糊、只有 ~10 Hz；关掉跟踪后原图清楚但帧率不同
- 右上角同时出现两个 fps（原图接收 fps 与"检测 fps"），数值互相打架（25 与 8.5）
- 曾用 `debug_scale 0.5`（320×240）+ QoS depth 1 缓解，但两端仍在 1–2 Hz 与 12 Hz 间跳

## 根因

检测节点把 640×480 标注图（921 KB/帧）也当图像发出来，与原始图**两路大消息**在
RK3588 上互相抢带宽/CPU；且检测只有 10–15 Hz，标注图天然比原图慢 → 显示端在两路
之间"选源"，超时切换就闪。帧率文字又分别在相机节点和检测节点各画一处。

## 正确做法 / 规避

1. **检测端只发结构化标注**：`a3_msgs/FaceOverlay`（478 个归一化点 + 三角剖分连线索引
   + 轮廓 + 框 + XYZ/RPY/误差，约 1 KB/帧），不发像素图。
2. **显示端做唯一出口**：`a3_bringup/vision_display_mux` 用原始 640×480 做底，在最后
   一层画全部标注；未开跟踪时透传原图。所有标注/文本/帧率只有这一处。
3. 判断"画面卡"先看 `ros2 topic hz /camera/image_raw`（用 `camera_info` 这种小消息或
   发布端日志，不要用 Python 订阅大图判帧率，LL-168）。
4. 图像类 QoS 用 BEST_EFFORT，但深度别压到 1（12 Hz 级别的标注会因严格丢帧而抖）。

## 相关路径

- `src/a3_msgs/msg/FaceOverlay.msg`
- `src/a3_bringup/a3_bringup/vision_display_mux.py`
- `src/a3_perception/a3_perception/face_detect_node.py`
- `src/a3_description/config/el_a3_view.rviz`
