# a3_bringup — 编排列车 + 桥接节点 + 各栈 launch

> 本包是**边缘主线的统一入口**：真机/仿真各一套 launch + 一堆"桥接/辅助"小节点。
> 栈启停优先用 `./scripts/a3_stack.sh {start|stop|restart|status} [--sim] [--rviz]`，不要手拼 launch。
> 需求索引：[docs/edge/REQUIREMENTS.md](../../docs/edge/REQUIREMENTS.md) ｜ 话题契约：[docs/shared/TOPIC_CONTRACT.md](../../docs/shared/TOPIC_CONTRACT.md)

## 关键文件

| 文件 | 职责 |
|------|------|
| `launch/a3_bringup.launch.py` | **真机全栈**（can_bridge + 编排 + 描述/MoveIt + 可选 RViz），`a3_stack.sh start` 的实体 |
| `launch/edge_teleop_full_sim.launch.py` | **仿真全栈**（domain 99），`a3_stack.sh start --sim` 的实体 |
| `launch/edge_web_sim.launch.py` | Web/遥测侧仿真底座 |
| `launch/edge_legacy_stack.launch.py` | 旧栈（reBot 壳）对照用 |
| `launch/usb_cam.launch.py` | **USB 相机单独入口**（F177）：`usb_camera_node` + 静态 TF + 可选 RViz；**真机栈已内置该节点**，此文件只用于脱离栈单独看相机 |
| `launch/servo.launch.py`、`launch/motor_maintenance.launch.py` | MoveIt Servo 链路 / 电机维护 |
| `usb_camera_node.py` | USB(UVC) → `/camera/image_raw`(bgr8) + `/camera/camera_info`，cv2 采集（见下） |
| `vision_display_mux.py` | **F194 统一视觉显示**：原图 + `a3_msgs/FaceOverlay` → `/a3/vision/display`，在最后一层画网格/轮廓/框/中心/连线/文本与唯一帧率；未开跟踪透传原图，并在左上角叠加末端位姿（`/a3/ee_pose` 的 x/y/z + RPY）、右上角第二行显示当前模式（`/a3/teleop/mode`） |
| `trajectory_bridge.py` | reBot 话题缝合（`/a3/joint_trajectory` → JTC） |
| `trajectory_spline.py` / `follow_joint_trajectory_action.py` | 轨迹插值/桥接辅助 |
| `gravity_torque_node.py` / `gravity_scale_calibration.py` | 重力前馈与比例标定（F49/F89） |
| `servo_mode_bridge.py` / `servo_anchor.py` | Servo 模式桥 + 绝对目标锚定（F118） |
| `move_to_pose_ik_node.py` / `ee_pose_publisher.py` | 笛卡尔 IK / 末端位姿话题（F175） |
| `sim_*.py`、`can_bus_monitor_node.py`、`topic_rate_monitor_node.py`、`systemd_watchdog_feed_node.py` | 仿真执行层 + 健康/看门狗 |
| `config/usb_cam.rviz` | 相机 RViz 配置（Fixed Frame `map` + Image display） |
| `config/diagnostics.yaml`、`config/cyclonedds.xml` | 诊断聚合、DDS 配置（经 `CYCLONEDDS_URI` 注入） |

## 常用入口

```bash
# 主栈（真机）
./scripts/a3_stack.sh start [--rviz]      # 仿真：--sim（domain 99）

# USB 相机（独立，不随主栈）
ros2 launch a3_bringup usb_cam.launch.py               # 只发话题
ros2 launch a3_bringup usb_cam.launch.py use_rviz:=true # 顺带开 RViz 看图
```

## USB 相机为什么不用 `usb_cam`

本机 RK3588 上 `apt` 装不上**任何** ROS 相机驱动（`usb_cam` / `v4l2_camera` / `gscam` /
`image_publisher`）：`rockchip-multimedia` PPA 的 pin 把 `libav*-dev` 压到优先级 1 以护住
rkmpp 版 ffmpeg，而这些包都要经 `cv-bridge → libopencv-dev → libav*-dev`。
故本包自带 `usb_camera_node.py`（仅依赖系统 `python3-opencv`）。
细节与另一个性能坑（`msg.data` 必须用 `array.array('B', …)`）见
[LL-168](../../docs/lessons_learned/LL-168-usb-camera-apt-pin-and-rclpy-bytes.md)。
