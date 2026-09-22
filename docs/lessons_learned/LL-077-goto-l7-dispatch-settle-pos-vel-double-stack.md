# LL-077 — F75：goto/move_to 走 move_group 同样漏 L7 / 宽位置带无速度检查会在臂仍运动时失能 / 重复 launch 导致 controller 重名 FATAL

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，MoveIt 2.5.9 / ros2_control 2.54.0 / JTC 2.53.3，mock_components/GenericSystem（calculate_dynamics=true）
> **关联：** [[LL-076-moveit-arm-group-leaves-l7-jtc-clamps-one-sided-limits]]、[[LL-072-ros2-control-mock-jsb-order-jtc-single-point]]、F75

## 现象

F75 用 `edge_full_mock.launch.py`（全产品节点 + mock hardware，零自研 sim 节点）做工业级验收，连续三个「动作报告成功、结果不对」：

1. jog 把 L7 停在 0.45 后发 `goto ready/home`：move_group success、L1–L6 落点干净，但最终 7 关节误差 0.4502——全在 L7，夹爪保持 jog 的 0.45 没动。
2. disable safe-park：move_group 回 home 后按旧判据（7 关节位置 tol=0.15、保持 confirm_s=0.5 s）判定到位并 reset/失能，但失能瞬间实测残差 0.055–0.110 rad——臂实际还在运动，只是恰好扫过 home 附近的宽带。
3. 旧 F75 launch 未退净时再起一个新栈：新 spawner 直接 FATAL `Failed loading controller arm_controller`，controller_manager 报 `A controller named 'arm_controller' was already loaded inside the controller manager`。

## 根因

### 1. move_group 只规划请求组——所有走 move_group 的入口都必须补发组外关节，不止 safe-park

LL-076 已记录该机制（safe-park 场景）。F75 证明 `_goto_cb`、`_move_to_cb` 的 move_group 分支有完全相同的缺口：规划组 arm = L1–L6，L7 由 gripper JTC 上一个 goal 继续保位。凡是「move_group + 7 关节目标位姿」的路径，漏补发的表现完全一致——success 但组外关节误差等于上一动作残留。

### 2. 只有位置带、没有速度门的落定判据，会放行「穿越容差带」的运动臂

旧判据 `max|pos−home| ≤ 0.15 保持 0.5 s`：一条速度约 0.2 rad/s 的收尾轨迹穿过 0.15 宽带的窗口约 1.5 s，足以满足 0.5 s 保持。随后控制器 deactivate、命令接口释放。**mock GenericSystem 在接口释放时冻结最后命令值，不会自由积分**——所以失能后测到的 0.055–0.110 rad 残差是真实的「提前失能」，不是 mock 漂移。真机上等价于在臂还在动时撤 MIT 使能，靠抱闸/重力兜底，是掉臂风险路径。

### 3. controller 名字是 controller_manager 进程内全局唯一资源

两个 `ros2_control_node` 并存（同一 ROS_DOMAIN_ID）时，第二个 spawner 向（任一）CM 加载同名 controller 必然失败；而且 DDS 发现层让两个 CM 的服务/话题同名并存，请求路由不确定，排障时极易误判。根上的原因只是「旧 launch 没杀干净」——launch 进程退出不等于其子节点（ros2_control_node / RSP / move_group / 各产品节点）全部退出。

## 正确做法 / 规避

- **统一封装组外关节补发**：`_dispatch_l7_linear(target, speed)` 按当前 L7 反馈构造多点线性轨迹经 gripper FJT action 下发，`_goto_cb`/`_move_to_cb`/safe-park 三个 move_group 入口在规划成功后一律调用，整体时长取 `max(arm_duration, l7_duration)`。|Δ|≤1e-3 不下发。
- **工业落定判据 = 位置 AND 速度双条件 + 连续确认窗**：`err ≤ 0.02 rad` 且 `max|vel| ≤ 0.05 rad/s`，连续保持 `confirm_s` 才允许失能（参数 `disable_home_settle_tol_rad` / `disable_home_settle_vel_rad_s`）。超过规划时长 1 s 仍未落定，补一条 7 关节本地线性纠偏（仅一次）；超时仍未落定则转 FAULT、保持使能，绝不带运动失能。
- **重启栈之前先做节点清点**：按 PID 显式 kill 上一栈的 ros2_control_node / RSP / move_group / FSM / gripper / retime / mqtt 全部子进程（`pgrep -f` 列 PID，不要用会匹配自身 shell 的宽 `pkill -f` 模式），确认目标 ROS_DOMAIN_ID 上零节点后再 launch。
- 验收脚本必须检查「失能后残差」与「控制器状态」两件事：JTC inactive + FSM DISABLED 不够，残差逐关节 ≤ 容差才算 safe-park 成功。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_dispatch_l7_linear`、`_goto_cb`/`_move_to_cb` L7 补发、`_safe_park_then_disable` 双落定 + 纠偏）
- `src/a3_arm_controller/config/arm_controller.yaml`（`disable_home_settle_tol_rad` / `disable_home_settle_vel_rad_s`）
- `src/a3_bringup/launch/edge_full_mock.launch.py`、`scripts/a3_test/f75_full_mock_acceptance.py`（15 项验收）
