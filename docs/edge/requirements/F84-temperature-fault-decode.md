# F84 — 温度/故障码完整解码（temperature state interface + motor_health 诊断 + 复用 F44 FSM 门禁）


- **说明：** 对比官方参考仓库 EDULITE_A3（`el_a3_hardware/src/robstride_can_driver.cpp` 反馈解码：`mode_state=(can_id>>22)&0x03`、`fault_code=(can_id>>16)&0x3F`、温度 raw/10）发现：F72 插件的 `DecodeFeedback` 虽然解码出温度与五个故障 bool，但 RX 循环**只复制 pos/vel/effort，温度/mode/故障字全部丢弃**；插件也没有温度 state interface。更关键的断链：FSM 既有的 **F44** 温度保护（90 warn / 95 protect / 迟滞 5 / safe park → COOLING）与电机故障监视（fault_mask≠0 → reset → FAULT）只订阅 `a3_can_bridge/msg/MotorStates`（`/a3/motor/states`），而 ros2_control `hardware:=can` 栈不再启动 a3_can_bridge——**F44 在新栈里静默失效**（无错误、无日志，超温/故障都不会触发）。F84 不重造保护逻辑，只把插件解码出的状态接到 F44 既有入口。
- **改动：**
  1. `motor_model.hpp`（两份镜像同步）：`MotorFeedback` 新增 `uint8_t fault_code`（6 位完整故障字，五个 bool 保留）；两份 `protocol_codec.hpp` `DecodeFeedback` 填 `fault_code=(can_id>>16)&0x3F`。
  2. 插件 `JointMapping` 新增 `hw_temp / hw_mode / hw_fault`；RX 循环复制；`export_state_interfaces` 每关节新增 `temperature` state interface（pos/vel/effort/temperature ×4）。URDF 中该接口**仅真机条件声明**（`use_real_hardware`），mock GenericSystem 不导出 temperature，避免 mock 栈校验失败。
  3. `PublishHealth` 新增第二条诊断 `a3_hardware:motor_health`：每电机键 `motorN_temp_c / motorN_mode / motorN_fault`；level：fresh 关节 fault_code≠0 或温度 ≥ `temp_protect_c`(95) → ERROR，≥ `temp_warn_c`(90) → WARN，否则 OK（与 F44 参数同源；F82 聚合 `startswith: ["a3_hardware:"]` 自动收录）。
  4. 插件 health 节点新增 publisher 以 50 Hz 发 `a3_can_bridge/MotorStates`（包新增对 a3_can_bridge 的消息依赖）：7 条全发，`fresh` = 有反馈且 age ≤ feedback_timeout，`fault_mask=fault_code`，`enabled`=组件 active，温度/mode 填实测值——FSM F43/F44 代码零改动即恢复工作。
  5. `vcan_motor_sim.py` 新增 `--health-file`（默认 `/tmp/f84_health.json`）：`{"motor":3,"temp_c":92.0,"fault":4,"mode":1}`，温度/6 位故障字/mode 注入反馈帧；`{}` 清除。
- **验收标准（仿真，vcan；脚本 `scripts/a3_test/f84_thermal_fault_acceptance.py`）：**
  1. `ros2 control list_hardware_interfaces` 可见 7 路 `<joint>/temperature [state]`（未 claim）；常温 30 °C 时 `/a3/motor/states` 7 条 fresh=true、temperature_c≈30、fault_mask=0、enabled=true；`a3_hardware:motor_health` level OK、键齐全
  2. 注入 92 °C（motor 3）：motor_health WARN、`ArmStatus.temp_warn=true`；warn 不打断——小幅 quintic FJT 两控制器 error_code=0
  3. 注入 96 °C：FSM 走**既有 F44** 路径（READY → SAFE_PARK → 失能 → COOLING）；降温至 protect−hysteresis(90 °C) 前 enable 被拒；改注 80 °C 后 enable 成功回 READY
  4. 注入故障位（motor 5 `fault=4`）：FSM 走既有 fault_mask≠0 路径（紧急 reset → FAULT），motor_health ERROR
  5. mock 栈（GenericSystem，无 temperature 接口）回归正常：hardware:=mock bringup 不因缺 temperature 接口失败
- **关联：** F44（温度保护/COOLING，本项只补数据源不重写逻辑）、F43（力矩统计同源 MotorStates）、F72/F74（ros2_control 栈）、F82（聚合自动收录）、F83（使能编排后电机才 enabled=true）；官方 EDULITE_A3 `robstride_can_driver.cpp`；LL-087（ros2_control 栈切换后旁路话题静默断链）
- **状态：** 仿真验收通过 10/10（2026-09-22，vcan4；含 mock 栈回归）。真机验收待上电。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
