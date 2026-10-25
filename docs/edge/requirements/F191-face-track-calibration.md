# F191 人脸跟踪标定（目标中心 / 深度 / 雅可比）

## 背景

视觉伺服要"追到哪、离多远、误差怎么变成运动"，这三件事都依赖现场标定。
早期只有雅可比可热重载，缺"目标位置"和"单目深度比例"两类，导致：

- 目标只能锁在画面正中心（人站偏一点，臂就一直往中间顶）
- 深度单位靠 D435/离线估，单目绝对距离不准（前后跟踪会跑过头或够不到）

## 标定分类（都在 TRACK 模式按键触发）

| 类别 | 含义 | 触发 | 落盘 |
|---|---|---|---|
| **水平/竖向 + 前后目标** | 把"当前人脸在画面里的位置 + 当前距离"记为目标（误差 0 点） | **Circle 长按 1.5 s** | `~/.a3/camera/face_target.yaml` |
| **前后（深度）比例** | 单目深度绝对尺度：脸放已知距离（默认 200 mm），按 z 中位数缩放 `depth_units_mm` | **Square 长按 1.5 s** | `~/.a3/camera/face_calib.yaml` |
| **雅可比** | 归一化图像误差 → 基座平移/转角的映射（离线脚本 `scripts/a3_test/f183_calib_jacobian.py` 生成） | **Triangle 长按 2 s**（热重载） | `~/.a3/camera/face_jacobian.yaml` |
| **状态查询** | 打印 enabled / 当前 err / z / z_ref / target / jac 是否加载 | **Square 短按** | — |

> 相机外参 `cam_rpy_deg`、内参水平视场 `hfov_deg` 属参数级标定（`face_track.yaml`），
> 换相机支架时改参数重启，不占用按键。

## 实现要点

- 控制器新增 `target_err_x/y`、`calibration_file` 参数；控制律里 `e -= target_err`，
  相当于"把人站的位置当成 0 误差"（水平/竖向）。`z_ref` 同时取当前中位距离（前后）。
- 检测节点新增 `/a3/face_track/calibrate_depth`：取最近 20 个有效 z 的中位数，
  `depth_units_mm *= known / z_med`，写回 `face_calib.yaml`（保留其它键）。
- 服务全部用 `std_srvs/Trigger`，避免为一个"记录一下"再引入新消息。
- 记录/标定后清积分、微分、追不动计数并补发一帧零速，避免旧偏置带进新目标。

## 验收标准

1. `ros2 service call /a3/face_track/status std_srvs/srv/Trigger` 回
   `enabled=… err=(…) z=… z_ref=… target=(…) jac=1 jac_a=1 arm=…`
2. 脸在画面偏右时按 Circle 长按 → 返回 target≈当前 err，之后臂不再把脸往中心推
3. 脸放 200 mm 按 Square 长按 → `depth_units_mm` 变化、`face_calib.yaml` 更新；
   重启后新值仍生效
4. 未检测到脸时按这两个键 → `success=false` 且提示"先把脸放进画面"
5. Triangle 长按 → 日志出现"已加载雅可比标定"

## 关联文档

- [F183](F183-face-tracking-servo.md)
- [F189](F189-ps4-mode-based-mapping.md)
- `src/a3_perception/a3_perception/face_track_controller.py`
- `src/a3_perception/a3_perception/face_detect_node.py`
