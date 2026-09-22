# LL-095 — BEST_EFFORT 晚加入订阅在 FastDDS SHM 下可数秒收不到数据（早加入的订阅却正常）；使能类操作要容忍一次重试

> **日期：** 2026-09-23  
> **产品线：** Edge  
> **环境：** RK3588 lubancat + ROS 2 Humble + FastDDS 2.6.11（SHM 传输），motor_protocol /joint_states @50 Hz BEST_EFFORT

## 现象

F88 phase 2（legacy topic 后端）启动后：验收节点（launch 前就已建好 /joint_states 订阅）正常收到 50 Hz 反馈，`wait_js` 通过；但状态机节点（launch 中晚 ~1.5 s 启动，订阅同为 BEST_EFFORT）在自己 ready 后 1.6 s 仍一条都没收到，`/a3/arm/enable` 被正确地以「no /joint_states received yet」拒绝。同一域内两个 BEST_EFFORT 订阅，一个有数据流、一个没有。

## 根因

FastDDS 2.6.x SHM 传输下，BEST_EFFORT publisher 与晚加入 subscriber 的端点配对/共享内存交接存在窗口：发布者开始发流之后订阅者才完成发现时，早期数据不补（BEST_EFFORT 语义），且 SHM 交接建立可能再拖数拍，表现为新订阅者静默数秒。期间不存在 QoS 不兼容（BEST_EFFORT 两端兼容），不是连接问题。

叠加干扰项：phase 1 被 SIGKILL 的 mock JSB（RELIABLE/TRANSIENT_LOCAL）发现信息短暂残留，产生「incompatible QoS RELIABILITY」告警——该告警与本次拒绝无关，但很容易把排查引向 QoS。

## 正确做法 / 规避

1. 状态转换类操作（enable/init）收到「反馈尚未到达」类拒绝时，调用方应 spin 等待反馈后**重试**，而不是直接判失败；这是启动竞态，不是产品缺陷。
2. 需要启动即收数据的节点尽量在 launch 前/同时完成订阅；或在节点内部以「收到首条反馈」作为就绪门槛（FSM 已有此门槛，方向正确）。
3. 看到 QoS incompatible 告警先确认它指的是哪个端点对，别和数据缺失直接划等号；被杀节点的发现残留约数秒自然消失。
4. 真机上同样适用：使能编排（F83）各步骤间要有等反馈 + 有限重试，不能假设发现完成 = 数据立即可用。

## 相关路径

- `scripts/a3_test/f88_two_point_trajectory_acceptance.py`（`enable_to_ready` 对 no /joint_states 拒绝的等待重试）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（/joint_states BEST_EFFORT 订阅；enable 的反馈前置检查）
