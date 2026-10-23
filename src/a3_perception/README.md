# a3_perception — A3 视觉感知

F183 人脸跟踪视觉伺服。需求与实测依据见
[`docs/edge/requirements/F183-face-tracking-servo.md`](../../docs/edge/requirements/F183-face-tracking-servo.md)。

## 职责

| 节点 | 输入 | 输出 | 跑在哪 |
|---|---|---|---|
| `face_detect_node` | `/camera/image_raw`（bgr8 BEST_EFFORT） | `/a3/face_target`（`a3_msgs/FaceTarget`），可选 `/a3/face_track/debug_image` | `~/.a3/venv/face_track`（mediapipe） |
| `face_track_controller` | `/a3/face_target`、`/a3/ee_pose`、`/a3/arm_status`、`/a3/control_mode` | `/servo_node/delta_twist_cmds`（50 Hz TwistStamped，base 系） | 系统 python |

关键文件：

| 文件 | 说明 |
|---|---|
| `a3_perception/face_detect_node.py` | mediapipe FaceLandmarker(478) → 6 参数 → One-Euro 滤波 / 外点剔除 / coasting |
| `a3_perception/face_track_controller.py` | 相机系速度 → base 系 → 限速/斜坡/死区/门控 |
| `config/face_track.yaml` | 全部参数（`~/.a3/camera/face_calib.yaml` 可覆盖） |
| `launch/face_track.launch.py` | 用 venv 解释器起检测节点 + 系统 python 起控制器 |

## 用法

```bash
# 1) 只出检测结果与标注图，不动机械臂（推荐先用这个确认检测/深度/方向）
ros2 launch a3_perception face_track.launch.py debug_image:=true
ros2 topic echo /a3/face_target --once
rqt_image_view /a3/face_track/debug_image     # 或 rviz2 Image display

# 2) 确认无误后打开跟踪（默认关闭，必须显式使能）
ros2 service call /a3/face_track/enable std_srvs/srv/SetBool "{data: true}"

# 3) 关闭跟踪
ros2 service call /a3/face_track/enable std_srvs/srv/SetBool "{data: false}"
```

前置：真机栈已起（`./scripts/a3_stack.sh restart`），`/camera/image_raw`、`/a3/ee_pose` 有数据，
臂处于 READY 且门禁已开（否则 `a3_servo_target` 会静默重锚，表现为"发了 twist 不动"）。

## 随栈启停（推荐）

```bash
./scripts/a3_stack.sh restart --face-track   # 起栈 + 拉起人脸跟踪（仍默认未使能）
./scripts/a3_stack.sh status                 # 会单独列一节「人脸跟踪（F183）」
./scripts/a3_stack.sh stop                   # 先停跟踪、再停栈（避免残留 twist 发布者）
```

## 首次使用必做

1. **环境**：`./scripts/setup_face_track_env.sh`（建 venv + 装 mediapipe + 下模型）。
2. **深度标定**：把脸放已知距离（如 200 mm），读 `/a3/face_target` 的 `xyz_mm.z`，
   写入 `~/.a3/camera/face_calib.yaml`：

   ```yaml
   /face_detect_node:
     ros__parameters:
       depth_units_mm: <14.0 × 200 / 实测Z>
   ```

3. **45° 外参符号**：`cam_rpy_deg` 默认 `[0, 45, 0]`。使能后若人脸在画面里越跑越偏（正反馈），
   把 pitch 取反。首次务必把 `v_max` 临时调到 0.02 观察方向。

## 参数速查

| 参数 | 默认 | 含义 |
|---|---|---|
| `kp_img` / `kz` | 0.9 / 1.2 | 图像误差→速度、深度误差→速度（1/s） |
| `v_max` | 0.05 m/s | 速度上限（F181 已验证的人脸跟踪档位） |
| `deadband` | 0.025 | 归一化图像误差死区（2.5% 画幅） |
| `dz_deadband_mm` | 20 | 深度死区 |
| `z_ref_m` | 0.45 | 期望工作距离 |
| `enable_angular` | false | 是否叠加腕部转动（默认只平移） |
| `min_cutoff` / `filter_beta` | 1.0 / 0.05 | One-Euro：调小更平滑、更滞后 |

## 排障

| 现象 | 原因 / 处理 |
|---|---|
| 检测节点起不来，`cannot import name 'one_of' from 'pyparsing'` | PYTHONPATH 顺序：venv 的 site-packages 必须在 `/usr/lib/python3/dist-packages` 之前。launch 已前置；手动跑请用 `env -u PYTHONPATH` 或自行前置 |
| `找不到 a3_msgs.msg.FaceTarget` | 先 `colcon build --packages-select a3_msgs a3_perception && source install/setup.bash` |
| 发了 twist 但机械臂不动 | 检查 `/a3/arm_status.state` 是否 READY/SERVO、`/a3/control_mode` 非 ZERO_TORQUE、门禁是否开、`/a3/ee_pose` 是否在发；控制器日志会打印 hold 原因 |
| 人脸越跑越偏（发散） | 45° 外参符号错（`cam_rpy_deg` pitch 取反）或相机系手性不符 |
| 深度明显不对 | 未做 `depth_units_mm` 标定，或检测节点跑的还是旧参数（`~/.a3/camera/face_calib.yaml` 优先级更高） |
| CPU 吃紧 / 控制环卡顿 | MQTT 已默认关（2026-10-23）；再关 RViz（不加 `--rviz`）、`debug_image:=false` |
