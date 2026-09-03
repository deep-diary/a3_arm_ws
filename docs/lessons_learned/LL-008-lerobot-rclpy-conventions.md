# LL-008 — LeRobot 插件接入 ROS 相机/录制：多节点 spin 冲突、特征约定、编码与门控

> **日期：** 2026-09-03
> **产品线：** Edge
> **环境：** RK3588 aarch64 + ROS 2 Humble + Python 3.10 venv（--system-site-packages）+ lerobot 0.4.4

## 现象

给 LeRobot 插件（lerobot_robot_a3）加 ROS 话题相机并跑录制时接连踩坑：

1. 相机后台 spin 线程抛 `ValueError: generator already executing`（rclpy/executors.py ... wait_for_ready_callbacks），相机收不到 /camera/color/image_raw，connect() warmup 超时。
2. 把 7 个关节状态声明成 observation_features={"state": (7,)} 时，录制管线把 state 当成一个 shape=(7,) 的「图像」相机。
3. lerobot-record CLI 一启动就等键盘右键，非交互/自动化测试跑不动。
4. 以为系统 ffmpeg 没有 libx264 就得用 mpeg4，结果 vcodec="mpeg4" 直接被拒；但其实录制视频能正常编码。
5. 新进程里 publisher 建好后立刻发的第一条轨迹，sim_executor 收不到，机械臂不动；连发多条时又正常。

## 根因

1. rclpy.spin_once(node) 用的是全局默认执行器，其回调生成器不可重入。backend 和 camera 各有一个后台线程并发 spin_once，共用同一个生成器 → 冲突。
2. lerobot 0.4.4 hw_to_dataset_features()（lerobot/datasets/utils.py）按「值是不是 tuple」分流：值为 float 的标量才聚合成 observation.state/action；值为 tuple 一律视为图像 observation.images.<key>（图像 tuple 必须 len==3）。
3. lerobot/scripts/lerobot_record.py 里 init_keyboard_listener() + events 门控每个 episode 开始，必须人工按键；自动化无法喂键。
4. 视频编码走的是 PyAV（av 包）自带的 ffmpeg 库，不是系统 /usr/bin/ffmpeg。PyAV 的 ffmpeg 带 libx264/libsvtav1；合法 vcodec 集合是 {h264, hevc, libsvtav1, auto, <HW encoders>}（datasets/video_utils.py），没有 mpeg4。
5. rclpy publisher 创建后，DDS discovery 需要时间与订阅端匹配；匹配前发出的首条消息被静默丢弃。

## 正确做法 / 规避

1. 每个被独立 spin 的节点用自己的 SingleThreadedExecutor：

```python
from rclpy.executors import SingleThreadedExecutor
self._executor = SingleThreadedExecutor(); self._executor.add_node(self._node)
# 后台线程: while ok: self._executor.spin_once(timeout_sec=0.1)
```

   服务响应也由这个后台 executor 处理；调用方 call_async() 后用 future.done() + time.sleep 等待即可，不要在主线程再对同一节点 spin_once（会和后台线程抢生成器）。disconnect() 时 executor.shutdown() 再 destroy_node()。

2. 关节特征声明成逐关节标量：{f"L{i}.pos": float for i in 1..7}；相机才用 tuple {cam_name: (H, W, 3)}。get_observation() 返回 {"L1.pos": float, ..., cam_name: img}，图像为 uint8 / HWC / RGB。数据集 key 自动变 observation.state(7)、action(7)、observation.images.<cam>。

3. 自动化录制测试不要用 CLI，直接 LeRobotDataset.create(repo_id, fps, root=..., robot_type=..., features=hw_to_dataset_features(...), use_videos=True, vcodec="h264")，循环 get_observation/send_action/add_frame → save_episode → finalize，再 LeRobotDataset(repo_id, root=...) 读回断言。脚本见 record_sim.py。人工用 CLI 时配一个虚拟遥操作（@TeleoperatorConfig.register_subclass）提供 action。

4. 视频编码器填 h264（PyAV 自带，arm64 可用）；判断可用编码器用 av.codec.Codec("libx264","w")，别看系统 ffmpeg。注意 LeRobotDataset.create(root=...) 要求 root 目录本身尚不存在（它自己 mkdir，存在则 FileExistsError）。带相机时 use_videos 不能为 False（会直接 raise）。

5. 建 publisher 后、发首条命令前，等订阅匹配：

```python
deadline = time.time() + timeout
while time.time() < deadline and pub.get_subscription_count() == 0:
    time.sleep(0.05)
```

## 相关路径

- src/lerobot_robot_a3/lerobot_robot_a3/ros_backend.py（独立 executor、首条命令等订阅匹配）
- src/lerobot_robot_a3/lerobot_robot_a3/ros_camera.py（独立 executor + 条件变量帧同步）
- src/lerobot_robot_a3/lerobot_robot_a3/robot.py（逐关节 float 特征、cameras 字段）
- src/lerobot_robot_a3/lerobot_robot_a3/record_sim.py（程序化录制/断言）
- .venv-lerobot/lib/python3.10/site-packages/lerobot/datasets/utils.py（hw_to_dataset_features）、.../datasets/video_utils.py（合法 vcodec）、.../scripts/lerobot_record.py（键盘门控）
