# F189 PS4 模式化按键映射（固定层 + 7 模式）

## 背景

单层映射把全部功能压在同一批按键上（短按/长按双义），按键已经用尽，且新增的
跟踪/弹琴/演示/舞蹈无法再分配。改为「固定层 + 模式层」：

- **固定层**（所有模式生效）：急停、上电使能、软失能、init、L1/R1 摇杆、R2 夹爪
- **模式层**：按住 `Share` + 方向/面键**直接点名**模式（不循环），模式内按键各自复用
- 统一退出：`Options` 短按 → 回 NORMAL（示教类先保存）

## 模式与组合键

| 组合 | 模式 | 进入动作 |
|---|---|---|
| Share + ↑ | TEACH | 自动准备拖动（未使能先上电+使能，READY 后自动开录） |
| Share + ↓ | WAYPOINT | 同上，但开的是路点示教（**F198 起改绑 Share+□**） |
| Share + ← | TRACK | `/a3/face_track/enable true`（required，失败回退 NORMAL） |
| Share + → | PIANO | goto `melodion_white1`（optional） |
| Share + △ | DEMO | goto `home`（改：先进工作位再演示） |
| Share + ○ | DANCE | goto `ready`（optional） |
| Share + □ | NORMAL | —（**F198 删除，改绑 WAYPOINT**；退出统一用 Options） |

切模式前一律先调 `/a3/arm/stop`（F190），等响应或 1 s 超时后再提交新模式，
避免上一个模式的轨迹/示教/servo 与新模式的输入串扰。

## 各模式按键

| 模式 | 按键 → 动作 |
|---|---|
| NORMAL | ↑=home / ↓=idle / ←=normal_slot_left(前) / →=normal_slot_right(后)；△短=ready、△长=latest 回放；□短=存点、□长=路点 PTP；○短=停止、○长=路点 LIN；L2=存点 |
| TEACH | 进入即开录；△=开/停录制（停止即保存）；L2=存点 |
| WAYPOINT | 进入即开路点录制；△=开始/打点；L2=打点 |
| TRACK | △短=跟踪开/关、△长(2s)=重载雅可比；**○短=网格点/网格线/人脸框开关**、**○长=水平+前后目标标定**；□短=状态查询、**□长=前后（深度）标定**；L2=存点；4 根摇杆轴被屏蔽（R2 夹爪保留） |
| PIANO | D-pad 四向=口风琴四个定位点；△=回放 stream `latest`；○/□=首/末键；L2=存点 |
| DEMO | 进入先回 `home`；△短=**只平移**（X→Y→Z 各一轮）、△长=**只平移**随机连续（30s/越界自停）；○短=**只旋转**（X→Y→Z 各一轮）、○长=**只旋转**随机轴；□=随机命名点巡游；L2=存点；D-pad 不绑定 |
| DANCE | △=回放 latest；□=路点 PTP；○=停止；L2=存点 |

## 关键修复（2026-10-25）

1. **D-pad 方向反了**：蓝牙 hid-sony 的 hat 轴轴向与直觉相反（物理"上/左"读到 +1），
   导致上→idle、下→home、左→后退。`ds4_linux.yaml` 对 `dpad_x` / `dpad_y` 取反后，
   `dpad_up/left` 虚拟键才与物理方向一致。
2. **进示教拖不动**：旧实现只切了"按键语义"，臂仍处于位置闭环（甚至 DISABLED），
   所以拖不动。现在 TEACH/WAYPOINT 的 enter hook 调 `teach_enter`：READY 直接开录；
   未使能则先 `power start + /a3/arm/enable`，由 50 Hz tick 轮询到 READY 后自动开录。
3. **Share+方向重复触发**：`share` 按住期间用"level + 单次锁存"触发，而不是 rising 边沿
   （20 Hz joy 下快速按键会漏），提交模式后保留锁存，松手才清。
4. **切模式瞬间泄漏按键**：提交新模式时仍按住的键/方向键会先锁存，**松开后**才允许
   在新模式里生效。否则 `Share+△` 进 DEMO 后，△ 一松就又从新模式触发一次动作
   （实测被误认为"正常模式的 △ 也被执行了"）。

## 验收标准

1. 依次 `Share+↑/↓/←/→/△/○/□`，`/a3/teleop/mode` 分别为
   `teach/waypoint/track/piano/demo/dance/normal`，每次切换前日志出现 `arm/stop`
2. 灯带按模式着色（绿/蓝/青/紫/黄/白/品红），切模式有对应次数震动
3. `Share+↑` 在臂 DISABLED 时：日志 `示教需要使能…` → L3 流程自动上电使能 →
   `READY` 后自动 `start_teach`，手可以拖动，灯带蓝色呼吸
4. `Options` 短按从任意模式回 NORMAL；TEACH/WAYPOINT 下先保存再退出
5. 方向键：↑→home、↓→idle、←→前进、→→后退

## 关联文档

- [F190](F190-arm-stop-service.md)
- [F194](F194-unified-vision-display.md)
- `src/a3_teleop_ps4/config/mappings/default.yaml`
- `docs/edge/PS4_OPERATOR_GUIDE.md`
