# LL-197：Humble 的 `joint_state_broadcaster` 不支持 `publish_rate`（配置静默失效）→ `/joint_states` 实为 200 Hz，F104 频段永久误报

> **日期：** 2026-10-26  
> **产品线：** Edge  
> **环境：** RK3588 LubanCat + ROS 2 Humble（ros2_control）

## 现象

- `a3_description*/config/el_a3_controllers.yaml` 里 `joint_state_broadcaster.publish_rate: 100.0`，
  `ros2 param get /joint_state_broadcaster publish_rate` 也返回 `100.0`。
- 但 `ros2 topic hz /joint_states` 实测 **≈198 Hz**（≈`controller_manager.update_rate: 200`），
  max gap 14 ms。
- `/diagnostics` 中 `a3_topic_rate: /joint_states topic status` 长期 `WARN "Frequency too high"`：
  实际 187 Hz，允许上限 66 Hz（由默认 `[40,60]` + 10% 容差算出）。
- 多个 Python 订阅者（arm_controller / ps4_mapper / gravity / ee_pose / watchdog / topic_rate）
  各自为空转付 25–35% 单核，ROS 节点合计约 5.6 核。

## 根因

1. **Humble 的 JSB 没有 `publish_rate` 参数**：参数文件里的键被静默忽略（不报未知参数），
   JSB 每个 `update_rate` 周期都发布一次。验证：
   ```bash
   strings /opt/ros/humble/lib/libjoint_state_broadcaster.so \
     | grep -x 'publish_rate|use_local_topics|extra_joints|frame_id'
   # 只有 extra_joints/frame_id/use_local_topics，没有 publish_rate
   ```
   `ros2 param get` 能读到值，是因为 YAML 里的参数仍被设置到节点上（无论是否声明），
   不能作为“参数生效”的证据。
2. **F104 门限停留在 50 Hz 时代**：`topic_rate_monitor` 默认 `[40,60]`，控制周期提升到
   200 Hz 后没有同步，导致永久误报；误报本身还白烧一个 200 Hz Python 订阅（约 33%）。

## 正确做法 / 规避

1. **不要用 JSB `publish_rate` 降 `/joint_states` 频率**；它由 `controller_manager.update_rate`
   决定（本仓 200 Hz，控制链需要）。
2. 低频消费者走 `F196` 的 `/joint_states_lite`（C++ `joint_state_relay`，默认 50 Hz）；
   需要 100 Hz 观测时改 relay 的 `rate_hz`，不要动控制周期。
3. F104 按采样源分频段：lite `[40,60]`，原始 `[180,220]`。
4. 排查“配置写了但行为不符”时，先确认参数**是否被声明**：
   ```bash
   strings <lib>.so | grep -x <param_name>          # C++ 插件
   ros2 param describe <node> <param>               # 是否声明/只读/类型
   ```
   再对照实测话题频率，不要只看 YAML。

## 相关路径

- `src/a3_description*/config/el_a3_controllers.yaml`（`publish_rate` 无效项）
- `src/a3_trajectory_processing/src/joint_state_relay_node.cpp`（lite 中转）
- `src/a3_bringup/a3_bringup/topic_rate_monitor_node.py`（频段默认值）
- `src/a3_bringup/launch/a3_bringup.launch.py`（`use_joint_state_lite` 分路）
- `docs/edge/requirements/F196-load-budget-and-light-profile.md`
