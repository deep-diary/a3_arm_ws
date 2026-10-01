# PS4 Options 长按触发 F116 随机点位巡游 — 实施计划

## 背景结论（仓库调研）

1. **轨迹形态**：F116 `/a3/arm/random_pose_tour` 不是一条合并轨迹，而是 **N 段独立 MoveIt 规划+执行的串行序列**（默认 5 段；相邻点不重复；默认排除 `zero`；任一腿失败即中止回 READY；L7 不动）。无需改动，现状即"连续多段巡游"。
2. **手柄现状**：teleop 全仓无 random/tour 绑定；Options 仅绑短按 `teach_stop`。Options **长按 3s 手势空闲**（F91 已退役长按 set_zero，仅文件头注释残留旧文案）。
3. **双义手势支持**：`ButtonEdgeTracker` 对同一物理键的多条绑定生成 `btn#i` 独立 key，longpress 按住到阈值触发一次，shortpress 超时作废，二者并存互不干扰（mapping.py 注释明确为 F55 模式）。
4. **安全门已具备**：服务端 `_can_move` 仅在 READY 态放行（DISABLED/IDLE/TEACH/TRAJ/FAULT 等全部拒绝）。故 TEACH 中误长按、巡游中再按，服务端均安全拒绝；巡游中仍可 R3 软失能 / Cross 硬急停中断。
5. **已选方案（用户确认）**：Options 短按=结束示教并保存（语义不变）；**Options 长按 3s=随机巡游**。点位池保持现状（除 zero 外全部命名点，含 snap_*/home_*）。

## 变更文件

- `src/a3_teleop_ps4/a3_teleop_ps4/actions.py`
  - import 增加 `RandomPoseTour`（a3_msgs.srv）
  - `ActionExecutor.__init__` 新建 client `/a3/arm/random_pose_tour`（RandomPoseTour）
  - 新增方法 `random_pose_tour(self, count: int = 0, seed: int = 0)`：service 未就绪 warn 并返回；否则 `call_async`（count=0/seed=0 → 服务端默认 5 点真随机），日志一行。经 `apply_discrete` 的 kwargs 机制，yaml 可覆盖 count/seed。
- `src/a3_teleop_ps4/config/action_registry.yaml`
  - 新增 discrete action：`random_pose_tour`，description「Call /a3/arm/random_pose_tour（F116：Options 长按；READY 态抽点巡游）」。
- `src/a3_teleop_ps4/config/mappings/default.yaml`
  - `options` 列表追加第二条：`fn: random_pose_tour, edge: longpress, hold_s: 3.0`（短按 teach_stop 项保留不动）。
  - 更新头部速查注释：`示教: Share 短按=开始示教 / Options 短按=结束并自动保存 / Options 长按 3s=随机点位巡游(F116) / Square 短按=回放 latest`；删除/修正「Options 长按 3s=set_zero」过时文案（第 10 行）。
- 文档（docs-update 规则）
  - `docs/edge/PS4_OPERATOR_GUIDE.md`：Options 行补「长按 3s=F116 随机巡游（仅 READY 态生效）」。
  - `src/a3_teleop_ps4/README.md` 按键表同步。
  - `docs/edge/REQUIREMENTS.md` F116：在状态行补「PS4 入口：Options 长按 3s」（服务本体早已实现，仅补遥测入口说明，不新开需求）。

## 实施步骤（依赖序）

1. actions.py 加 RandomPoseTour client + `random_pose_tour()` 方法。
2. action_registry.yaml 注册 discrete action（否则 mapper 启动 validate_mapping 直接 RuntimeError）。
3. default.yaml 加 options longpress 绑定 + 注释修订。
4. 三处文档同步。
5. colcon build --packages-select a3_teleop_ps4（ament_python；顺带 flake8/test_pep257 视现有门禁）。

## 验证

- **静态**：mapper 启动不抛 invalid mapping；`ros2 run a3_teleop_ps4 joy_dump` 不需要（按键索引不变）。
- **mock 栈闭环**（隔离域，硬件不动）：
  1. 起 mock 全栈，L3 使能到 READY；用脚本向 `/joy` 注入 Options 按住 ≥3.2s 再松开（ds4_linux.yaml 的 options 键位），断言 arm_controller 日志出现 `random tour (5 legs, ...)` 且状态经历 TRAJ 后回 READY、sequence 返回 5 点。
  2. 注入 Options 短按（<3s 松开）：在非 TEACH 态应**无任何服务调用**（仅 teach_stop；stop_teach 非示教态被服务端拒绝，行为与现状一致）；先 Share 进入 TEACH 再短按 Options → 正常结束示教并保存 latest。
  3. TEACH 态注入长按 3s：巡游服务被 `state=TEACH` 拒绝，臂不动、仍处 TEACH（安全门回归）。
- 真机回归留给用户后续通电时顺手验证（与 mock 同手势）。

## 风险与处理

- **TEACH/TRAJ 中误长按**：服务端 `_can_move` 拒绝（已核实），不新增客户端状态判断，保持 mapper 无状态。
- **长按 3s 期间短按误触**：shortpress overshoot 机制保证超时释放不触发 teach_stop（F55 现成语义）。
- **巡游时间较长（多段十几~几十秒）**：async 调用不阻塞 mapper；中断靠 R3/Cross，既有链路不变。
- **set_zero 文案误导**：顺手清理，实际功能早已迁移 motor_maintenance 节点（F91）。
