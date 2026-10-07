# F83 — 使能流程对齐厂商标准（清故障 → 整数写运行模式 → 使能 → 软启动阻尼接管）


- **说明：** 对比官方参考仓库 EDULITE_A3（`el_a3_sdk/interface.py` `EnableArm`）发现，插件当前 `on_activate` 仅 reset-all → enable-all，缺少厂商标准使能编排的两个关键步骤：①使能前先发 **Type 4 失能帧且 data[0]=1 清除故障锁存**（`clear_fault=True`），防止带历史故障位直接使能被电机拒入或带病运行；②使能前必须用 **Type 18 参数写把运行模式 0x7005 写为目标模式的整数值**（MOTION_CONTROL=0）——该参数是 uint8，走 float 编码会把低字节写成 0x00 导致模式损坏（厂商 `write_parameter_int` 专用整数路径）。F83 将 `on_activate` 重排为厂商顺序，在 F81 启动门（reset-all → 7/7 应答证明）通过后，**逐电机**执行：清故障（0x04, data[0]=1）→ 间隔 30 ms → 整数写 RUN_MODE=0（0x12, 0x7005, value=0）→ 间隔 30 ms → 使能（0x03）→ 间隔 30 ms（每电机约 90 ms，7 路合计 < 1 s 一次性成本）。任一电机未应答则保持 F81 fail-fast 语义，绝不部分使能。
- **软启动（我们的加固，非厂商行为，须如实标注）：** 厂商 `EnableArm(startup_kd=4.0)` 的 `startup_kd` 在其代码库中**声明但从未使用**（全库 grep 仅签名与文档串）。F83 落地该参数的语义：使能后前 `soft_start_cycles`（默认 10 个写周期 ≈ 50 ms @ 200 Hz）个 `write()` 周期，全部 7 路发**纯阻尼接管帧**——位置取实测位、kp=0、kd=`startup_kd`（默认 4.0，clamp 0–5）、速度/力矩前馈 0——先以阻尼「接住」机械臂再过渡到正常位置刚度（kp/kd 默认 80/2），避免使能瞬间弹簧阶跃。F81 stale freeze-hold 优先级仍最高（软启动期反馈断线照常冻结）；软启动结束后正常控制帧恢复。全部 7 个关节（含 L7 夹爪）统一 MOTION_CONTROL 模式——厂商给夹爪用 POSITION_PP，是因其夹爪走梯形 profile 接口；本仓夹爪栈依赖 MIT 运动控制帧（含力控特性），此处为有意偏差并记录。
- **接线：** `src/a3_hardware_interface/include/a3_hardware_interface/protocol_codec.hpp`（新增 `BuildClearFaultFrame`：Type 4 data[0]=1；新增 `kParamCanTimeout=0x7028` 备 F86；同步注释镜像到 `src/a3_can_bridge` 同名 codec）；`src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（on_init 新参数 `startup_kd` 默认 4.0 / `soft_start_cycles` 默认 10；on_activate 厂商编排；write() 软启动倒计时；on_deactivate 复位）；验收 `scripts/a3_test/f83_enable_choreography_acceptance.py`（CAN 原始帧序 + 软启动增益 + 运动回归）。
- **验收标准（仿真，vcan 原始帧嗅探）：**
  1. 使能时每路电机（id 1–7）在总线上**按序**出现：Type 4 且 data[0]=1（清故障）→ Type 18 且 param=0x7005、data[4]=0（MOTION_CONTROL）→ Type 3（使能），各类帧之间间隔 ≥ ~20 ms；F81 启动门仍在编排之前（先 reset + 7/7 应答）
  2. Type 3 之后每个电机的前 ≥3 个 Type 1 控制帧 kp 解码 < 1、kd 解码在 3.5–4.5（纯阻尼），随后 ≤ 0.5 s 内过渡到正常增益（kp 解码 70–90、kd≈2）
  3. 软启动结束后运动正常：小幅度 quintic FJT（L1–L6 0.10 rad / L7 0.05 rad，3 s）两 JTC 均 error_code=0，嗅探到位移 span；disable 干净（on_deactivate 零增益帧 + reset）
  4. 启动即缺电机时 F81 语义不回退：无任何清故障/模式/使能帧之外的部分使能（暗电机情况下整个编排不执行），on_activate 返回 ERROR
- **关联：** F81（启动门 / freeze-hold / read 永远 OK）、F72（硬件插件）、F74（ros2_control 执行后端）；官方 EDULITE_A3 `EnableArm`；[shared/SAFETY.md](../shared/SAFETY.md)（使能安全）
- **状态：** 仿真验收通过 9/9（2026-09-22，vcan1/vcan3；另含 boot→enable 零帧检查与 F83/LL-086 组件生命周期显式编排）。真机验收待上电。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
