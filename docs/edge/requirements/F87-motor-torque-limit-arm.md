# F87 — 电机侧力矩限制布防（0x700B Type-18 float 写入；使能编排内每电机逐路下发）


- **说明：** 统一栈（F78）下真机只走 `a3_hardware_interface` 插件；原 `a3_can_bridge` 的 `/a3/motor/set_param` 服务无人启动。F24 夹爪节点仍向该服务写固件力矩上限，服务永不就绪 → 重试静默失败，固件力矩限制在统一栈下从未被任何节点显式布防（出厂值虽为各型号峰值，状态不可验证；URDF 的 `torque_max` 逐关节下调也没有固件落地路径）。F87 把力矩上限布防收进使能编排：插件按已解析的逐关节 `torque_max`（JointMapping；RS00=14 Nm、EL05=6 Nm，可用 URDF 参数 `torque_max` 逐关节下调）用 Type-18 写 `0x700B`。协议手册五件事已核实：**float32、单位 Nm、范围 0~型号峰值（EL05 6 / RS00 14）、写入易失（掉电回出厂峰值）、运控模式下立即作为输出力矩钳位生效**。
- **改动：**
  1. `on_activate` 厂商编排每电机在 RUN_MODE=0 之后、0x7028 之前插入：`BuildSetParamFrame(0x700B, j.torque_max)`（IEEE754 float，不能复用 raw/u8 构造器），保持 30 ms 间隔；每次 activate 重写（易失，且 reset 后状态不确定）。
  2. `vcan_motor_sim.py` 参数存储泛化：0x700B 按 raw uint32 存储并在 Type-17 读回应答（与 0x7028 同一存储/应答路径）。
  3. 验收脚本 `scripts/a3_test/f87_torque_limit_acceptance.py`：raw socket 断言逐电机写帧的存在/编码/数值/帧序，Type-17 读回对照，编排回归（clear-fault/RUN_MODE/0x7028/enable 仍齐）。
  4. 夹爪节点对已退役 `/a3/motor/set_param` 的依赖处理放在 F87 第二步（L7 原生力控）；本提交不改变夹爪节点行为。
- **验收标准（仿真，vcan；脚本 `scripts/a3_test/f87_torque_limit_acceptance.py`）：**
  1. 默认使能：raw socket 抓到 7 路 Type-18 帧，`data[0..1]=0b 70`、`data[2..3]=00 00`、`data[4..8]` LE 解释为 IEEE754 float：motor 1–3 = 14.0 Nm、motor 4–7 = 6.0 Nm（±1%）；每路恰一帧，无重复无跳电机
  2. 帧序：每电机的 0x700B 写位于该电机 RUN_MODE 写（0x7005）之后、0x7028 写之前、enable（Type 3）之前
  3. Type-17 读回：harness 发读参数帧，7 路应答的 raw 值与对应 float 位模式逐位一致
  4. 编排回归：clear-fault（Type 4 data[0]=1）/ RUN_MODE / 0x7028=4000 / enable 帧计数与顺序仍满足 F83/F86
  5. 使能后小幅 quintic FJT error_code=0、运动正常（F83/F85/F86 回归）
- **关联：** F83（使能编排，本项在其中插步）、F86（同一 Type-18 编排，float vs uint32 编码对照）、F24（夹爪固件硬限；本项先布全臂，第二步退役夹爪节点的死服务依赖）；LL-089（寄存器类型/量纲以协议手册为准）
- **状态：** 仿真验收通过（vcan `f87_torque_limit_acceptance.py` 47/47，2026-09-23：7 路 float 写帧 14/14/14/6/6/6/6±1%、帧序 0x7005<0x700B<0x7028<enable、Type-17 读回逐位一致、编排回归、FJT error_code=0）。真机验收待上电。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
