# LL-083 — ros2_control 插件 read() 返回 ERROR 会静默 write()；on_activate 返回 ERROR 直接 abort controller_manager

> **日期：** 2026-09-22  
> **产品线：** Edge  
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble，ros2_control 2.54.0（joint_trajectory_controller 2.53.3）

## 现象

F81 要在硬件插件 `A3MITHardwareInterface` 内做单电机反馈断线保护。直觉写法是 `read()` 检测到某电机反馈超时就返回 `return_type::ERROR`，让框架处理故障。仿真实测该写法下：

- 断线后插件 CAN TX **全灭**：插件 `write()` 里的安全保持帧根本不发出，臂失支撑后在重力下静默下坠；
- stale 只在第一次打出日志（组件被转走后 read/write 都不再执行）；
- 上层看到的是「节点活着、无报错、臂掉了」。

另在实现启动门（激活前校验 7 电机应答）时实测：`on_activate()` 返回 `CallbackReturn::ERROR` 时 resource_manager 直接 throw `Failed to set the initial state of the component`，**ros2_control_node SIGABRT 退出**，controller_spawner 陷入 `Could not contact service` 循环——没有「Inactive 组件 + 其余栈存活」这种温和状态。

## 根因

对 ros2_control 2.54.0 源码核实（`hardware_interface/src/system.cpp`）：

1. `System::read()` 收到插件返回 ERROR 后调用组件的 `error()`；默认 `on_error()` 返回 SUCCESS，组件被强制转到 **PRIMARY_STATE_UNCONFIGURED**（不是停在 active）。
2. `System::write()` 在 UNCONFIGURED / FINALIZED 态**直接早退返回 OK**——框架认为这是正常的，插件自己的 write 一行都不执行。于是安全写静默消失，且没有任何上层错误信号。
3. `on_activate` 返回 ERROR 属于「组件无法进入 active」的致命错误，资源管理器把它当作启动失败处理，直接终止控制节点；错误检测发生得越晚（比如已经 enable 了 6 个电机之后），留下的危险状态越大。

## 正确做法 / 规避

- **`read()` 永远返回 `return_type::OK`**：故障只在插件内部锁存（per-motor stale + 整臂 latch），组件保持 active；保护动作放在 `write()` 里执行——F81 的整臂 freeze-hold：丢弃控制器新指令，7 路全发最后已知位置的位置保持帧（kp/kd 维持）。仿真 895+ 保持帧、关节零位移。
- staleness 用**标准通道**外发：`/diagnostics`（`a3_hardware:feedback_watchdog` + 每电机年龄 KeyValue）与 latched `/a3/hardware/feedback_stale`（`std_msgs/Bool`，TRANSIENT_LOCAL），由编排 FSM 做运动门禁；不要指望用插件返回值触发框架保护。
- 注意 open_loop JTC 的语义：即使 `open_loop_control: true`，JTC 在末点之后仍按**实际状态接口**校验 goal tolerance；`goal_time_tolerance=0` 时不会 abort、目标持续挂起。所以 freeze-hold 期间 FJT 目标是「接受但永不报成功」（验收断言 pending），不会产生盲态假成功。
- **启动 fail-fast 门的顺序**：reset-all（MIT reset 后电机处于 disabled/coast 安全态）→ 轮询证明 7/7 应答（500 ms）→ 任一缺失则返回 ERROR，**不发任何 enable 帧** → 全在才 enable-all。这样 on_activate 的致命 ERROR 触发时，所有电机仍安全 coast。

## 相关路径

- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（read / write freeze-hold / on_activate 门 / PublishHealth）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`feedback_stale_topic` 订阅、`_can_move` 门禁、disable stale 快路径）
- `scripts/a3_test/f81_feedback_stale_acceptance.py`、`scripts/a3_test/vcan_motor_sim.py`（`--silence-file`）
- `docs/shared/SAFETY.md`（单电机反馈断线保护 F81）、`docs/shared/TOPIC_CONTRACT.md`（硬件反馈看门狗话题）
