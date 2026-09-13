# LL-028 双 robot_state_publisher 同帧名互踩 + frame_prefix 用法三坑（RViz 双模型）

- **日期**：2026-09-13
- **产品线**：Edge（URDF 方向校验：实际反馈模型 + 目标 ghost 模型同屏）
- **严重度**：中（症状迷惑性强：实际模型「两个状态来回跳」，目标模型 No transform——根因都在 rsp 前缀配置）

## 现象

RViz 双 RobotModel（一个 TF Prefix 空读实际反馈，一个 TF Prefix `target` 读摆动目标）：实际模型在「真机位姿 ↔ 摆动目标位姿」两套变换间来回跳动（真机静止也跳）；目标模型报 `No transform from [target/base_link] to [...]`。录 4 s `/joint_states` 只有 ±0.0004 rad 编码器噪声——**反馈数据不跳，跳的是 TF**。

## 根因（三个叠加的坑）

1. **`tf_prefix` 是错误参数名**：ROS2 robot_state_publisher 的前缀参数叫 `frame_prefix`；`tf_prefix` 不是它的声明参数，被**静默忽略**（无任何告警）。两个 rsp 于是用同一批帧名发 /tf，tf2 缓冲在同名帧上交替收录两套数据 → 实际模型在两个状态间跳动。
2. **`frame_prefix` 直接拼接、不自动加 `/`**：`frame_prefix: "target"` 产出 `targetgripper_link`；而 RViz RobotModel 的 TF Prefix 查找时**自动加 `/`**（找 `target/gripper_link`）→ 仍 No transform。正确写法是 `frame_prefix: "target/"`。
3. **rsp 不发布根帧**（base_link 是 URDF 根、无父关节）：`frame_prefix` 后根帧 `target/base_link` 也不发，需一个恒等静态变换 `base_link → target/base_link` 把目标树接到 Fixed Frame 上（tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link target/base_link）。

## 修复

`urdf_dir_check.launch.py`：rsp_target 参数改 `frame_prefix: "target/"` + 新增 `target_base_link_static_tf` 恒等静态变换节点。验证：/tf 帧名两树分立（无重名）、`base_link→target/l1_link_urdf_asm`、`base_link→target/gripper_link` 链全部 resolve（Python tf2 listener 实测）。

## 注意

- 双模型/多机器人用 rsp 时，先确认「两个 rsp 的帧名空间不重叠」——**同帧名双源是「模型来回跳」的典型根因**（tf2 缓冲交替收录，且不报错）。
- 排查 TF 链用 Python `Buffer.lookup_transform` + 4 s 预热（/tf_static transient local 补发和监听器建连都要时间，短窗口会误报 "does not exist"）；`tf2_echo` 管道输出不可靠。
- 帧名含 `/` 在 tf2 里可用但属于命名空间语义，别裸写前缀——让 rsp（尾斜杠）和 RViz（自动加斜杠）各自按自己约定拼，实测两者拼出的名字一致即可。
- 查 rsp 参数以 `ros2 param`/源码为准，别照 ROS1 经验（ROS1 里才叫 tf_prefix）。

相关：[[LL-027]]
