# F55 — PS4 全功能映射：示教/执行/使能/初始化一键化（短按+长按双义）


- **说明：** 手柄遥控是示教/回放的主交互入口，但编排层服务（init/enable/disable/start_teach/stop_teach/playback）此前一个键都没接。本需求把 arm_controller 服务全部映射到 PS4 空闲键，并为映射引入短按边沿——一个键「短按=功能 A、长按=功能 B」的双义（Options：短按停示教 / 长按 3s 调零）。示教主流程定键：**Share=开始、Options=结束、Circle=执行**。
- **映射（`src/a3_teleop_ps4/config/mappings/default.yaml`，完整表见该包 `README.md`）：**

  | 键 | 边沿 | 动作 |
  |----|------|------|
  | Share | 短按 | `start_teach`（进示教，0/mode 拖动 + 记录） |
  | Options | 短按 | `stop_teach`（停记录 + F54 自动保存） |
  | Options | 长按 3 s | `power_set_zero`（调零；原 2 s 上调，降误触） |
  | Circle | 短按 | `playback` 空名（回放定义最新） |
  | Touchpad | 短按 | `arm_init`（0/mode，F51 越限只 WARN） |
  | L3 | 短按 | `arm_enable`（使能安全门禁：重锚 + 软起步） |
  | R3 | 短按 | `arm_disable`（F40：离 idle 先 safe park） |
  | Triangle | 长按 1 s | `power_shutdown`（**唯一**手柄急停） |
  | Square / Cross / D-pad / L1 / R1 / R2 | 不变 | 上电 / 立即停 / 命名位姿 / deadman / boost / 力控夹爪 |

- **「三键组合急停」废弃：** 原 SAFETY 契约的「L1+R1+Share = 关机」需要三键同时操作，误用风险高且真按三键反而不如单键可靠。急停收敛为 **Triangle 长按 1 s**（单键、防误触、无组合需求）；`L1+R1+Share` 从 SAFETY.md 移除。
- **验收标准：**
  1. 映射含上述 7 个新绑定；`buttons.options` 为**两条绑定的列表**（短按/长按各自独立边沿跟踪）；mapper 启动时 `validate_mapping` 对列表逐条校验、零报错
  2. 短按语义：按下起计时、**释放时**持续时长 `< hold_s`（3 s）才触发一次；按住超 `hold_s` 再释放该次不触发、也不能连带触发同一键的另一条绑定
  3. sim（domain-55 隔离栈 + fake `/joy`）：Share → `arm_status.state=TEACH`；Options 短按 → `stop_teach success + auto-saved latest.yaml`；Circle → 日志 `playback latest`；Options 按住 ≥3 s 再放 → `/power_sequence/command` 收到 `set_zero`（且**不**触发 teach_stop）；L3/R3 → enable/disable 成功
  4. 既有按键零回归：D-pad 4 位姿、Cross 立即停、Square/Triangle 电源、L1 deadman、R1 boost、R2 力控保持原绑定
  5. 文档三处可见：包 `README.md` 完整表 + `default.yaml` 文件名即可改（约零代码） + `QUICKSTART.md` 指向
- **关联：** F54（自动保存/空名=latest）、F38（示教回放）、F51（使能安全）、F40（失能保护）、F36（力控扳机）；[shared/SAFETY.md](../shared/SAFETY.md)、`src/a3_teleop_ps4/README.md`
- **状态：** `implemented`（2026-09-17，代码 + 文档；仿真验收通过；真机需用户在场按手柄）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
