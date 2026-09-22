# LL-086 — Humble 硬件 INACTIVE 启动后 switch_controller 不会自动激活组件

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588 (lubancat) Ubuntu 22.04 + ROS 2 Humble + ros2_control

## 现象

F83 期望「操作员 enable」是总线上的第一个动作（boot→enable 零帧），因此把
硬件组件配置为 INACTIVE 启动。接入后出现三种与文档直觉不符的行为：

1. **默认（不设初始状态）时 controller_manager 极度积极**：ros2_control_node
   启动即 initialize→configure→**activate** 硬件，且在没有任何 active
   controller 时也按 200 Hz 周期调用 `write()`。
2. 组件 INACTIVE 启动后，spawn 并 active 一个只声明 **state 接口**的
   controller（joint_state_broadcaster）**不会**触发组件激活——符合预期，
   但容易被误以为「JSB active = 硬件已工作」。
3. **关键坑**：之后调用 `/controller_manager/switch_controller` 激活声明
   **command 接口**的 JTC，controller 状态照样变成 `active`，但组件的
   `on_activate()` **始终不执行**：总线上零编排帧、JTC 轨迹无任何输出，
   且没有任何错误返回（switch 响应 ok=true）。表象是「使能服务假成功、
   臂不动」，极难排查。

## 根因

Humble 的 controller_manager 只在两种场景自动激活硬件组件：启动时的默认
引导，以及（带命令接口声明的）组件 configure 流程；`switch_controller`
本身不驱动组件状态机，controller 的 `active` 与 hardware component 的
`ACTIVE` 是两条独立的生命周期。框架设计上组件生命周期应由部署方显式
管理，而不是由 controller 开关隐式触发。

另外 shell 层还踩了一个伪装成「服务挂死」的坑：
`export ROS_DOMAIN_ID=86 ... && setsid A & sleep 1 && setsid B` 中 export
只活在被 `&` 放后台的第一个子 shell，B 继承的是原始环境（domain 0），
跨域 CLI 调用永久挂起。判断手段：`tr '\0' '\n' < /proc/<pid>/environ |
grep ROS_DOMAIN_ID`。

## 正确做法 / 规避

1. 在 controller_manager 参数中显式设定初始状态，使操作员 enable 成为
   第一个激活点：

   ```yaml
   hardware_components_initial_state:
     inactive:
       - RsA3System
   ```

2. FSM 的 enable/disable 显式驱动组件生命周期（服务
   `/controller_manager/set_hardware_component_state`，
   `controller_manager_msgs/srv/SetHardwareComponentState`，
   `lifecycle_msgs/State` 主状态 id：INACTIVE=2 / ACTIVE=3）：

   - enable = set_hw ACTIVE（`on_activate` 同步阻塞约 1.2 s：reset-all →
     500 ms 反馈 7/7 应答证明 → 逐电机清故障/SetParamU8(0x7005,0)/enable
     （间隔 ≥30 ms）→ 重锚 → 一轮阻尼帧）→ switch_controller activate；
     switch 失败必须把硬件退回 INACTIVE。
   - disable = switch_controller deactivate → set_hw INACTIVE
     （`on_deactivate` 零增益刷新 + reset-all 自由滑行）。

3. 时序裕量：FSM 内 enable 最坏路径约 21 s（wait_service 3 + set_hw 10 +
   wait_service 3 + switch 5），harness 的服务超时必须按此留足（30 s），
   且先 `wait_for_service` 再 `call_async`；FSM 在每阶段打日志，避免「零
   日志挂死」无法定位。

4. 验收用 CAN sniffer 从 controller_manager 启动前开始抓帧，直接证明
   boot→enable 零帧；dark-motor 场景证明 `on_activate` 返回 ERROR 时
   没有任何一路电机被使能（见 LL-083）。

## 相关路径

- `src/a3_description/config/el_a3_controllers.yaml`（hardware_components_initial_state）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_cm_switch`）
- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（on_activate/on_deactivate）
- `scripts/a3_test/f83_enable_choreography_acceptance.py`（9/9 验收）
