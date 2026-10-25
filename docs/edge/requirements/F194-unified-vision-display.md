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
/a3/vision/show_mesh (网格/框开关) ────────────┘
```

- **检测端只做检测**：`face_detect_node` 发 `a3_msgs/FaceOverlay`（478 点归一化坐标 +
  三角剖分连线索引 + 人脸轮廓 + 框 + XYZ/RPY/误差），不再出像素图
- **显示端统一叠加**：`vision_display_mux` 用原始分辨率画面做底，画出全部标注与文本；
  未开跟踪时直接透传原图
- **单一帧率**：右上角只显示最终 `/a3/vision/display` 的帧率，不带 "display" 前缀

## 显示约定

| 元素 | 颜色 | 开关 |
|---|---|---|
| 画面中心十字 | 青 | 始终 |
| 人脸中心十字 | 橙（跟踪中）/ 绿（误差进死区） | 始终 |
| 中心 ↔ 人脸中心连线 | 白 | 始终 |
| 网格点 + 网格连线（mediapipe tessellation 全量 2556 条） | 亮绿 / 深绿 | `show_mesh` |
| 人脸轮廓（mask 边界） | 黄 | `show_mesh` |
| 人脸框 | 蓝 | `show_mesh` |
| 文本 XYZ / RPY / BOX+ERR | 随人脸中心状态色 | 检测到人脸时 |

文本格式（`|` 分隔，误差含 x/y 两轴）：

```
XYZ  x  +12 | y  -30 | z  452 mm
RPY  yaw  +1.2 | pitch  -3.4 | roll  +0.5 deg
BOX  204 x 202 px | ERR  x +0.120 | y -0.060
```

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
3. 开跟踪：网格连线呈对称人脸网格、轮廓闭合，中心青色十字与人脸中心橙色/绿色
   十字之间有白线
4. TRACK 模式 Options 长按 → 网格点/框消失，中心与人脸中心保留；再长按恢复
5. `/a3/face_track/debug_image` 默认不再发布（`--face-debug` 可临时打开）

## 关联文档

- [F183](F183-face-tracking-servo.md)
- [F189](F189-ps4-mode-based-mapping.md)
- `src/a3_bringup/a3_bringup/vision_display_mux.py`
- `src/a3_msgs/msg/FaceOverlay.msg`
