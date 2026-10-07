# F86 — 电机侧通信超时配置（0x7028 Type-18 写入；与主机看门狗独立的纵深防御）


- **说明：** F81 的反馈 staleness 看门狗运行在主机上：主机掉电、系统卡死、CAN 线缆脱落时，看门狗本身随主机一起失效。MIT 固件内置 CAN 通信超时参数 `0x7028`（**uint32，单位约 50 µs，20000≈1 s；0=关闭**，出厂默认 0；Type-18 写入为易失性、掉电丢失）：超时未收到任何帧时电机自行进入 RESET 模式（失能/阻尼态，等同 Type 4）。官方 EDULITE_A3 在每次使能时**主动写 0 关闭**它、只依赖主机看门狗；F86 从纵深防御角度有意偏离官方——主机看门狗负责可恢复的瞬时冻结保持（F81），电机侧超时负责「主机已死」时最后一道独立卸力。默认 0.2 s（4000 counts）：远大于 200 Hz 指令环抖动（正常帧间隔 5 ms），又能在主机失死后快速切断力矩。
- **改动：**
  1. 插件 `on_init` 解析硬件参数：`motor_can_timeout_enabled`（bool，默认 true）、`motor_can_timeout_s`（double 秒，默认 0.2，clamp [0.05,10]）；换算 `counts = round(seconds * 20000)`，以 uint32 LE 经 `BuildSetParamRawFrame`（Type 18）下发——不能按 float 编码（旧 codec 注释「float, s」是错的，F86 修正）。
  2. `on_activate` 厂商编排每电机：clear-fault → 整数写 RUN_MODE=0 → **写 0x7028**（enabled 写配置 counts，disabled 显式写 0，与官方一致且状态确定）→ enable，各步保持 30 ms 间隔。
  3. `on_deactivate` 每电机在 reset 前补写一次 0x7028=0 解除武装，避免同一次上电周期内残留。
  4. xacro 真机插件块新增两参数；bringup（can）与 vcan launch 透传 `motor_can_timeout_enabled`（默认 true）；`motor_can_timeout_s` 走 xacro 默认。
- **验收标准（仿真，vcan；脚本 `scripts/a3_test/f86_can_timeout_acceptance.py`）：**
  1. 默认使能：raw socket 抓到 7 路 Type-18 帧，`data[0..1]=28 70`、`data[2..3]=00 00`、`data[4..8]` LE uint32 ≈ 4000（±10%）；帧序位于 RUN_MODE 写（0x7005）之后、enable（Type 3）之前
  2. Type-17 读回：harness 发读参数帧，电机应答 0x7028 值 ≈ 4000
  3. 端到端跳闸：杀掉整栈后，sim 状态文件显示 7/7 电机在 [0.18, 0.7] s 内进入 tripped（RESET），无一提前误跳
  4. `motor_can_timeout_enabled:=false` 重启：使能编排仍写 0x7028 但 counts=0；杀栈后 1 s 内无电机 tripped
  5. 使能后小幅 quintic FJT error_code=0、运动正常（F83/F85 编排回归）
- **关联：** F81（主机侧 staleness 看门狗，本项补电机侧独立防线）、F83（使能编排，本项在其中插步）；官方 EDULITE_A3 `can_driver.py::write_parameter_int` / `interface.py`（使能写 0）；LL-089（寄存器类型/量纲须以协议手册为准——0x7028 是 uint32 计数不是 float 秒；易失写入与 flash 保存的边界）
- **状态：** 仿真验收通过（vcan `f86_can_timeout_acceptance.py` 29/29，2026-09-22：布防帧/读回/无误跳/SIGKILL 后 7/7 在 0.202 s 跳闸/撤防重启无跳闸）。真机验收待上电。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
