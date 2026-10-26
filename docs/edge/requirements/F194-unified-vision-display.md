# F194 统一视觉显示（原始画面 + 结构化人脸标注）

## 背景

人脸跟踪早期由检测节点直接输出 640×480 标注图（`/a3/face_track/debug_image`）：

- 检测端 mediapipe 只有 ~10–15 Hz → 标注图也只能这个帧率，且画面被缩放到 320×240
  才能勉强跟上，观感"又慢又糊"
- 921 KB/帧 × 2 路（原图 + 标注图）在 RK3588 上互相挤占，RViz 里出现两个 fps、来回跳

## 方案（pipeline 统一管理）

```
/camera/image_raw (25 Hz, 640×480) ─┐
                                     ├─→ vision_display_mux ─→ /a3/vision/display (25 Hz)
/a3/face_overlay (~1 KB/帧)  ────────┘         │
/a3/vision/mode (跟踪开关) ────────────────────┤
/a3/vision/show_mesh (网格/框开关) ────────────┤
/a3/ee_pose (末端位姿, 10 Hz) ─────────────────┤
/a3/teleop/mode (操作模式, 1 Hz) ──────────────┘
```

- **检测端只做检测**：`face_detect_node` 发 `a3_msgs/FaceOverlay`（478 点归一化坐标 +
  三角剖分连线索引 + 人脸轮廓 + 框 + XYZ/RPY/误差），不再出像素图
- **显示端统一叠加**：`vision_display_mux` 用原始分辨率画面做底，画出全部标注与文本；
  未开跟踪时直接透传原图
- **单一帧率**：右上角只显示最终 `/a3/vision/display` 的帧率，不带 "display" 前缀

## 显示约定

| 元素 | 颜色 | 开关 |
|---|---|---|
| 画面中心十字 | 淡青 | 始终 |
| 人脸中心十字 + 实心点 | 琥珀（跟踪中）/ 绿（误差进死区） | 始终 |
| 中心 ↔ 人脸中心连线 | 弱白 | 始终 |
| 网格点 + 网格连线（mediapipe tessellation 全量 2556 条） | 暗亮绿 / 暗绿（压暗当背景） | `show_mesh` |
| 人脸轮廓（mask 边界） | 暗金 | `show_mesh` |
| 人脸框 | 灰蓝 | `show_mesh` |
| 文本块（跟踪 4 行 / 非跟踪 3 行，两列对齐） | 标签灰 / 位置值近白 / 角度值青 / 误差随状态色 | 见下 |
| 右上角第一行帧率 | 绿 | `draw_fps` |
| 右上角第二行操作模式（`normal` / `track`…，服务单独开跟踪时补 `+track`） | 跟踪中琥珀、否则近白 | `draw_fps` |

文本块格式（两列各自把"值"对齐到固定 x；标签与数值**不同色**，扫读更快；
整块下垫 40% 黑底，亮背景下也清晰）：

```
跟踪中（相机系 mm + deg + 图像误差 + 末端半径）
x:   -27 mm   roll:    -6.7 deg
y:   +60 mm   pitch:   -0.1 deg
z:   321 mm   yaw:     +3.9 deg
ex: -0.146    ey:     +0.431
r:   398 mm                     ← 末端到基座 \|p\|，超 reach_warn(0.48) 变琥珀

非跟踪（末端 base_link mm + deg + 半径）
x:  -191 mm   roll:   +102.5 deg
y:   +22 mm   pitch:    +0.6 deg
z:  +235 mm   yaw:     +81.8 deg
r:   398 mm
```

> **R 行（2026-10-26 加）**：两种模式都显示，用于对着伺服侧 `reach_max`（0.50 m 半径硬限）
> 直接看工作半径——跟踪到边界时臂停在球面上，R 贴着告警线不再增长。

> **单位用 mm**：工作空间 <1 m，`0.132` 这种小数在现场不好读；`132 mm` 一眼可对。
> 不再显示 `EE` / `RPY` 表头与 `BOX … px`（框在画面上已画出，冗余）。
> 早期人脸跟踪的 `XYZ … | RPY … | BOX …` 单行拼接格式一并废弃。

> **为什么不显示四元数**：单个分量（尤其 `w`）没有独立物理含义，`w=0.54` 现场看不出
> 姿态；RPY 才能对着 RViz / 示教点位核对。约定 `R = Rz(yaw)·Ry(pitch)·Rx(roll)`，
> 与 `face_track_controller` 的 `quat_to_matrix` 同源；`roll/pitch/yaw` 与
> `R_x/R_y/R_z`（绕基座固定轴 X→Y→Z 依次转）数值等价，只是 ROS/tf2 惯例叫 RPY，
> 便于和 RViz 的 `getRPY` 直接对照。俯仰到 ±90° 附近 RPY 会退化（万向锁），
> 此时以 RViz 姿态显示为准。

> **网格对称性（2026-10-25 修）**：连线索引必须是 mediapipe 的
> `FACEMESH_TESSELATION` **全量**（2556 条）。早期为省 CPU 做了
> `sorted(...)[::4]` 抽样——按字典序每 4 条取 1 条不具镜像对称性，画面会出现
> "一侧密一侧疏"。显示层改用 `cv2.LINE_8` 画全量只需 ~7 ms/帧（AA 版要 ~29 ms），
> 不需要牺牲对称性。另：头偏转/倾斜时检测点的左右位移本来就不对称，属正常。

## 按键

TRACK 模式下 **Circle 短按** = 网格点/网格线/人脸框开关；
画面中心与人脸中心、连线始终保留。Circle 长按留给标定（F191），两者不会误触。

## 验收标准

1. `ros2 topic hz /a3/vision/display` ≈ 相机帧率（640×480@25 → 20+ Hz），
   RViz Image 面板显示 640×480，不再有缩放模糊
2. 未开跟踪：`/a3/vision/display` 与原图一致（无标注）
3. 开跟踪：网格连线呈对称人脸网格、轮廓闭合，中心淡青十字与人脸中心琥珀/绿色
   十字之间有弱白连线
4. TRACK 模式 Options 长按 → 网格点/框消失，中心与人脸中心保留；再长按恢复
5. `/a3/face_track/debug_image` 默认不再发布（`--face-debug` 可临时打开）
6. 非跟踪（正常）模式：左上角出现 3 行 `x/y/z (mm) + roll/pitch/yaw (deg)` 文本块
   （数据来自 `/a3/ee_pose` 10 Hz；无 `EE`/`RPY` 表头、两列数值上下对齐），
   开跟踪后立即换成 4 行（含 `ex/ey`）人脸块；右上角第二行只显示模式名（无 `mode` 前缀），
   服务单独开跟踪时带 `+track`

## 关联文档

- [F183](F183-face-tracking-servo.md)
- [F189](F189-ps4-mode-based-mapping.md)
- `src/a3_bringup/a3_bringup/vision_display_mux.py`
- `src/a3_msgs/msg/FaceOverlay.msg`
