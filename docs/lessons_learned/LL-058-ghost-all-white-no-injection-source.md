# LL-058 — 双模型 RViz ghost 全白/no transform：target rsp 没有任何 joint_states 注入源（只补静态 TF 不够）；修法 = launch 默认挂 ghost-only 注入 + 外来注入让位看门狗

> **日期：** 2026-09-18
> **产品线：** Edge
> **环境：** RK3588 lubancat + ROS 2 Humble + 双模型 RViz（`el_a3_dual_view.rviz`，真机栈与 `edge_web_sim` 同构）

## 现象

主 launch（`a3_bringup.launch.py`）双模型 RViz：实际模型正常，**目标 ghost 整臂纯白**，Display 状态 `ArmTarget_目标: Error`，全部 link 报 no transform；Fixed Frame `base_link` 正常。`/tf` 里**一个 `target/` 帧都没有**，`/tf_static` 只有恒等变换 `base_link -> target/base_link`。

## 根因（LL-055 机制的「零消息」特例）

- [LL-055](LL-055-rsp-missing-joint-no-tf.md)：`robot_state_publisher` **只对 joint_states 消息里出现过的关节名发 TF**。特例：**一条消息都没收到过时，整棵树（连固定关节）什么都不发**——固定关节在 `/tf_static` 的发布同样依赖首次 joint_states 回调触发的发布周期。
- 主 launch 当时只挂了 target rsp + 恒等静态 TF，**没有任何节点发布 `/a3/display_target_joint_states`**：`urdf_dir_check_pub` 只存在于 `urdf_dir_check.launch.py`，旧 QUICKSTART 还明确写着「主 launch 不启动它」（因为它默认往执行层发轨迹）。
- 恒等静态 TF 只把根帧 `target/base_link` 接进树；rsp 不出帧时，根以下全部 link 仍然断裂 → RViz 全白报错。

## 修法（已落地）

1. **注入器 ghost-only 模式**：`urdf_dir_check_pub` 加 `publish_traj`（默认 true 保持方向校验原行为）；false 时不创建轨迹发布器/电机状态订阅，只发 `/a3/display_target_joint_states`（默认 home 静止、7 名齐全），纯话题不碰执行层，断电安全。
2. **外来注入让位** `backoff_on_foreign_msgs`（默认 false，两个 launch 里置 true）：节点订阅自己的话题，凡内容与最近一次自发不同（`ros2 topic pub` 等外部注入）即停发让位 `foreign_backoff_s`（默认 3 s，外来消息滚动续期），停发后自动恢复 home。同名多发布者是 last-writer-wins。
3. **两个 launch 默认挂 ghost 三件套**（target rsp + 恒等静态 TF + ghost-only 注入）：`a3_bringup.launch.py`、`edge_web_sim.launch.py`，均可用 `use_target_ghost:=false` 整体关闭。sim launch 默认 RViz 是单模型配置，ghost 只在 `el_a3_dual_view.rviz` 下可见。
4. 重编 `--packages-select a3_bringup`（launch 改动必须重编，`install/share` 才更新）。

## 排查踩坑（本次验证全踩到）

- **`/tf` 是 VOLATILE、`/tf_static` 是 TRANSIENT_LOCAL**：给 `/tf` 套 TRANSIENT_LOCAL QoS 订阅会收不到任何数据并刷 `incompatible QoS ... DURABILITY`；两个话题必须分开配 QoS。
- **rclpy 验证必须 spin**：`TransformListener` + `time.sleep()` 暖机是空转，回调没执行、buffer 为空，所有 lookup 报 frame does not exist。用 `spin_once` 循环暖机。
- **`ros2 topic echo ... | grep` 管道里 python 块缓冲**，超时经常零输出；时序/存在性验证用 rclpy 脚本，别信 CLI 管道。
- **rsp 存活 ≠ 在发动态 TF**：joint_states 源死亡后动态 `/tf` 可停（本次 sim_motor 被杀，实际模型整链从 `/tf` 消失、断裂成两棵树），而固定关节仍在 latched `/tf_static`——排查要分别看两个话题，不能只 ps 到 rsp 就认为模型有数据。
- 独立手起 rsp 用 `--params-file` 时，YAML 必须是 `/**: {ros__parameters: {...}}` 两层包装，裸键值对启动即 SIGABRT（`Cannot have a value before ros__parameters`）。

## 教训

**rsp 的模型树不是「挂上就有」：它是 joint_states 驱动的，一条消息都没有时连固定关节都不发。双模型配置里 target rsp + 静态根 TF + 注入源三者缺一不可，缺注入源的表现就是 ghost 全白。**

## 关联

- [LL-055](LL-055-rsp-missing-joint-no-tf.md)（直接机制依据：缺关节名/零消息 → 不发 TF）
- [LL-028](LL-028-robot-state-publisher-frame-prefix.md)（`frame_prefix` 尾斜杠）
- [LL-054](LL-054-fastdds-ghost-participant-kill9.md)（注入器必须优雅退出，别 kill -9 污染 FastDDS）
- [LL-027](LL-027-rviz-panfrost-freeze-software-render.md)（RK3588 看 RViz 软件渲染）
- QUICKSTART「RViz 双模型（实际 vs 目标 ghost，真机主 launch 内置）」节
