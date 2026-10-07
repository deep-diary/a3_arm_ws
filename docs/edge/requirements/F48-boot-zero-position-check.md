# F48 — 开机零位校验（读数限位硬检查 + 期望位姿软检查 + 使能门禁）


- **说明：** F47 之后正常断电（断电间不超 ±180° 转动）读数跨上电保持连续，但环绕（+2π 多圈推算）仍可能发生（L2/L3 行程超 π；zero_sta 丢失、超范围转动等异常）。上电后校验通过前**不使能**。本需求两层：
  1. **编排层使能门禁**（arm_controller）：`/a3/arm/enable` 发 enable 前检查 7 关节读数全部落在 URDF 限位内**且 /joint_states 新鲜**（stamp 距今 ≤ `js_max_stale_s: 1.0`，参数 `enable_position_check: true` 默认开启；未收到 /joint_states 也拒绝）——环绕读数超限时 kp×误差会瞬间猛拉（LL-019），越限拒绝并点名关节与限位，恢复零位走 `/a3/arm/init`。`init` 是恢复路径（set_zero 重建零位帧后再使能），越限只 WARN 不阻断——WARN 指明「当前位姿将被定义为零位，仅在已知位姿（工装摆 URDF 零位）执行」。
  2. **独立开机校验脚本** `scripts/a3_check_zero_frame.py`：读 /joint_states 对照 URDF 限位（硬检查，任一越限、消息陈旧（>1 s）或超时无消息 exit 1——判断是否环绕的唯一标准）；再对照期望位姿模板软检查（L2/L3/L5/L6/L7 ≈ 0、L4 ≈ 0（URDF 零位）或 ≈ 0.34（折叠自然下垂）、L1 自由——水平转动 ±178° 由机械限位约束；默认容差 0.35 rad ≈ ±20°，仅提示性、不改变退出码）。
  3. **桥保活播种（执行层配套，三态语义，LL-020/LL-022）**：refresh 流在无指令历史（last_commanded 为 NaN）时按电机模式播种：①模式未知/失能 → 零增益保活帧（p=反馈位或 0，kp=kd=tau=0）；②已知使能且反馈新鲜 → 目标一次性锚定到最新反馈位，以 runtime kp/kd/tau 真实保位（服务直驱使能后无轨迹流的掉臂修复，LL-022 真机实测）；③已知使能但反馈陈旧 → 不盲发。保证开机/使能后总线有帧流、反馈持续上送——否则 js 冻结旧值（LL-020），或已使能电机静默无力矩（LL-022）。
- **验收标准：**
  1. 正常上电（读数在限位内）→ `/a3/arm/enable` 放行进入电机使能流程
  2. 人为构造越限读数（仿真注入 L6=5.9）→ enable 拒绝，message 点名关节与限位；init 放行但 WARN
  3. 未收到 /joint_states（桥未起）→ enable 拒绝并提示
  4. 脚本：真机正常位 → 硬检查 PASS exit 0（软检查打印位姿匹配结论）；越限注入 → FAIL exit 1
  5. 桥重启后无任何指令下发：5 s 内 /joint_states 持续新鲜发布且读数跟踪手动拖动（播种生效）；停桥后脚本报 stale FAIL
  6. 服务直驱使能（/a3/motor/enable）后无轨迹下发：已使能关节被锚定保位（runtime 增益），/a3/motor/states 全关节 fresh=true、tx_stats.tx_hz≈refresh 频率×7；02 反馈帧扩展 ID bit22-23 解析 mode=2（LL-022 真机验证）
- **关联：** F47（zero_sta 窗口，本需求是其开机侧兜底）；[lessons_learned/](../lessons_learned/)（环绕机理与「使能前必须 probe 校验」红线）；F45（状态机使能路径）
- **状态：** `implemented`（2026-09-13 真机验证：零位帧恢复后 7/7≈0，脚本硬检查 PASS exit 0 且软检查匹配「URDF 零位」；隔离域仿真验证 enable 门禁三场景——无 /joint_states 拒绝、L6=5.9 越限拒绝并点名、限内放行；init 越限 WARN 放行。限位比较需裕量（编码器量化噪声 ±0.0002，L2/L3/L7 下界为 0），参数 `position_check_margin_rad: 0.001`。补充：真机发现桥无指令历史时 js 冻结旧值（LL-020）——新增 refresh 零增益播种 + 脚本/门禁 stamp 新鲜度检查，软检查容差按用户反馈放宽至 ±20°）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
