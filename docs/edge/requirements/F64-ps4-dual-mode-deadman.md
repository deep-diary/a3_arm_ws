# F64 — PS4 双模式死人开关（L1 平移 / R1 旋转）+ D-pad 分通道调速


- **说明：** F60 的 R1「全速档」语义含混（按住 R1 把线速度从 0.35 拉满，旋转也同比例放大），且单速度档无法分别精细调节平移与旋转。用户 2026-09-21 提出并确认改为**双模式死人开关**：

  | 操作 | 动作 |
  |---|---|
  | **L1 按住** + 摇杆 | 末端**平移**：左摇杆左右=Y、上下=Z；右摇杆上下=X |
  | **R1 按住** + 摇杆 | 末端**旋转**：右摇杆左右=偏航 Z（首批仅绑 yaw；roll/pitch 待实操后按需加绑） |
  | **D-pad 上 / 下** | 平移速度档位 **+ / −** 步进（仅改线速度比例） |
  | **D-pad 左 / 右** | 旋转速度档位 **− / +** 步进（仅改角速度比例） |

  实现要点：
  1. 删除 `held.r1`（set_speed_scale 全速绑定）；删除 `deadman` 单开关块，轴改逐轴 `gates: [按钮…]` 列表——轴任一 gate 按住才输出，平移三轴 gates=[l1]、偏航轴 gates=[r1]。
  2. ActionExecutor 单一 `speed_scale` 拆为 `linear_scale` / `angular_scale`（默认均 0.35），tick_end 线速度通道乘 linear_scale、角速度通道乘 angular_scale；新增 `step_linear_scale` / `step_angular_scale`（kwargs delta，clamp 0.1..1.0，步进 0.15）。
  3. mapper 新增 `dpad` 配置块，对 dpad_x/dpad_y hat 轴（−1/0/+1）做 0→±1 边沿检测（持续按住不连发，回中后才能再步），映射到上述 step 动作。
  4. R2 夹爪、命名位姿/示教/电源键不受 L1/R1 门控影响（维持 F60 语义）。
  5. 松开 L1/R1：被门控轴立即归零，servo_mode_bridge 0.5 s 超时兜底停。
- **验收标准：**
  1. 不按 L1/R1 推任何摇杆：1.2 s 内关节 Δ<0.03 rad
  2. 按住 L1：平移三轴动、偏航轴不动；按住 R1：偏航动、平移三轴不动（合成场景逐断言）
  3. D-pad 上/下：linear_scale 步进且仅改变平移速度；D-pad 右/左：angular_scale 步进且仅改变旋转速度；持续按住不连发
  4. F62 合成脚本扩展场景在 domain 45 全绿；真机保持断电，用户先以真手柄在仿真栈实操验收
- **关联：** 修订 F60 的 L1/R1 语义（键位表其余部分不变）；F61（灯效仍按 READY 绿，jog 中不换色）、F62（合成验证扩展）；[shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `completed`（2026-09-21 仿真：全新栈 F62 合成 46/0；含 L1/R1 通道隔离、松手 Δ<0.03、D-pad 步进 0.15/clamp 0.1、持续按住不连发；真手柄真机验收待办）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
