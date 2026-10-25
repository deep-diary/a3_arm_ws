# a3_teleop_ps4 — PS4 全功能映射（F60/F64：双死人开关 / 示教 / 回放 + 灯带震动）

> **查键位看这里。改键位只编 `config/mappings/default.yaml`，零代码。**
> 操作员手册（流程/灯/震动）：[docs/edge/PS4_OPERATOR_GUIDE.md](../../docs/edge/PS4_OPERATOR_GUIDE.md)
> 需求与验收：[docs/edge/REQUIREMENTS.md](../../docs/edge/REQUIREMENTS.md) F60–F64、F119/F120/F135；
> 安全合同：[docs/shared/SAFETY.md](../../docs/shared/SAFETY.md)。

## 固定层（所有模式生效）

| 键 | 边沿 | 动作 | 底层调用 |
|----|------|------|----------|
| **L3** | 按下 | 一键开门禁 + 使能 | `power start` → `/a3/arm/enable`（幂等） |
| **R3** | 按下 | 任意模式软失能 | `/a3/arm/disable`（F119：先退出 SERVO/示教/重力补偿 → F40 safe-park 回 idle → 保电失能） |
| **Cross（✕）** | 长按 1 s | **硬急停**（断电、关闸；恢复需 L3） | `/power_sequence/command shutdown` |
| **L1+R1 同按** | 长按 2 s | init（设零；**不带使能**） | `/a3/arm/init` |
| **L1** | 按住 | **平移死人开关**：仅放行平移轴 | servo_lin_x/y/z（gates l1） |
| **R1** | 按住 | **旋转死人开关**：偏航/俯仰/滚转 | servo_ang_x/y/z（gates r1） |
| **R2** | 模拟 | 夹爪力控（不需要 L1/R1），松开全开 | gripper_force |
| **Share 按住** | 修饰 | 进入"选模式"窗口，屏蔽摇杆/其它键 | — |
| **Options** | 短按 | 退出当前模式 → NORMAL（示教类先保存） | `/a3/arm/stop` |

## 选模式（按住 Share + 方向/面键，直接点名不循环）

| 组合 | 模式 | 说明 |
|---|---|---|
| Share + ↑ | TEACH | 进模式即准备拖动（未使能会自动上电+使能，READY 后自动开录） |
| Share + ↓ | WAYPOINT | 路点示教（L2/△ 打点） |
| Share + ← | TRACK | 人脸跟踪（自动 enable；4 根摇杆轴被屏蔽，R2 保留） |
| Share + → | PIANO | 口风琴定位点 |
| Share + △ | DEMO | home 五向巡游 / 随机巡游 |
| Share + ○ | DANCE | 回放最新动作 |
| Share + □ | NORMAL | 手动/通用回放 |

## 各模式按键

| 模式 | 按键 → 动作 |
|---|---|
| **NORMAL** | ↑=home、↓=idle、←=前（normal_slot_left）、→=后（normal_slot_right）；△短=ready、△长=回放 latest；□短=存点、□长=路点 PTP；○短=停止、○长=路点 LIN；L2=存点 |
| **TEACH** | 进入即开录；△=开/停录制（停即保存）；L2=存点 |
| **WAYPOINT** | 进入即开路点录制；△=开始/打点；L2=打点 |
| **TRACK** | △短=跟踪开/关、△长(2s)=重载雅可比；**○短=网格点/网格线/人脸框开关**、**○长=水平+前后目标标定**；□短=状态查询、**□长=前后（深度）标定**；L2=存点 |
| **PIANO** | D-pad 四向=口风琴四点；△=回放 stream `latest`；○/□=首/末键；L2=存点 |
| **DEMO** | 进入先回 home；△短=固定三轴角度巡航（X→Y→Z）、△长=连续随机平移（30 s/越界自停）；○短=旋转巡游（偏航往复）、○长=随机角度巡游；□=随机命名点巡游；Options=停止退出；D-pad 不绑定 |
| **DANCE** | △=回放 latest；□=路点 PTP；○=停止；L2=存点 |

> 方向键方向已按物理方向修正（`ds4_linux.yaml` 对 `dpad_x`/`dpad_y` 取反，F189）。
> 口诀：**Share+方向选模式，Options 退出；L3 开工、R3 收工、✕ 长按急停；L1 平移、R1 旋转、R2 夹爪。**

## DS4 灯带 / 震动（F61，`ds4_feedback_node`）

派生态优先级：FAULT（红双闪 + 双震）→ 失电/硬急停（红闪 + 强震 600 ms）→
INIT（橙，完成边沿白闪一次）→ TEACH（蓝呼吸）→ TRAJ/safe-park（紫）→
READY/SERVO（绿，进入弱震 120 ms）→ DISABLED（橙 + 弱震 120 ms）。

- HID 报告自动按总线区分：USB 0x05/32 B；蓝牙 0x11/78 B + HWCTL + CRC32（seed 0xA2）。
- 无手柄时节点存活，逻辑帧以 JSON 发到 **`/a3/ds4/feedback`**（字段：state/power_state/gate/fault/class/color/pattern/rumble_weak/rumble_strong/reason/device；仅内容变化时发），供无设备断言。
- hidraw 权限：用户需对 `/dev/hidraw*` 有读写（plugdev/udev 规则），否则反馈节点只走诊断话题。

## 映射在哪里改 / 看

| 文件 | 作用 |
|------|------|
| `config/mappings/default.yaml` | **真机生产映射（本表驱动的 YAML）** |
| `config/mappings/simple.yaml` | 无 L1 直测映射（防真机误用，保留） |
| `config/action_registry.yaml` | 动作函数目录（加新动作才需要代码） |
| `config/ds4_linux.yaml` / `ds4_linux_usb.yaml` | 按键/轴索引（重校准索引时才碰） |
| `a3_teleop_ps4/actions.py` | ActionExecutor（函数实现 + gate/power 缓存） |
| `a3_teleop_ps4/ps4_mapper.py` + `mapping.py` | 边沿引擎（rising / shortpress / longpress） |
| `a3_teleop_ps4/ds4_feedback_node.py` | 灯带/震动反馈节点 |

## 边沿语义

- `rising`：按下瞬间触发一次。
- `shortpress`（短按）：**释放时**判定，按住 < `hold_s` 触发一次；超过 `hold_s` 再释放作废（不误触、不连带同键其他绑定）。
- `longpress`（长按）：按住 ≥ `hold_s` 触发一次。
- 一个键可用 **YAML list** 绑多条，独立跟踪，互不干扰（Options 短按=结束示教 / 长按 3 s=F116 随机点位巡游；F91 退役的长按调零走独立节点 `motor_maintenance`）。

```yaml
buttons:
  options:
    - {fn: teach_stop, edge: shortpress, hold_s: 3.0}
```

## 启动

```bash
# 全仿真（无手柄无 CAN，域 45）：双模型 RViz + 合成 /joy 验收
ros2 launch a3_bringup edge_teleop_full_sim.launch.py
python3 scripts/a3_test/ps4_sim_test.py          # 12 场景 46 项，逐项 PASS/FAIL

# 仿真接真手柄实操（合成全绿后）：
ros2 launch a3_bringup edge_teleop_full_sim.launch.py use_joy_node:=true

# 真机：统一入口默认已含 mapper + ds4_feedback_node（mapping:=default），servo 常驻
sudo systemctl start can-up.service
ros2 launch a3_bringup a3_bringup.launch.py hardware:=can

# 只起 teleop（底栈已在跑）：
ros2 launch a3_teleop_ps4 ps4_teleop.launch.py mapping:=default
```
