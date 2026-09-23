# a3_teleop_ps4 — PS4 全功能映射（F60/F64：双死人开关 / 示教 / 回放 + 灯带震动）

> **查键位看这里。改键位只编 `config/mappings/default.yaml`，零代码。**
> 操作员手册（流程/灯/震动）：[docs/edge/PS4_OPERATOR_GUIDE.md](../../docs/edge/PS4_OPERATOR_GUIDE.md)
> 需求与验收：[docs/edge/REQUIREMENTS.md](../../docs/edge/REQUIREMENTS.md) F60–F64；
> 安全合同：[docs/shared/SAFETY.md](../../docs/shared/SAFETY.md)。

## 完整映射表（`mapping:=default`，F60 + F64）

| 键 | 边沿 | 动作 | 底层调用 |
|----|------|------|----------|
| **L3** | 短按 | 一键开门禁 + 使能 | `power start` → `/a3/arm/enable`（幂等） |
| **R3** | 短按 | safe-park 后失能 | `/a3/arm/disable`（F40：离 home 先 park） |
| **Cross（✕）** | 长按 1 s | **硬急停**（断电、关闸；恢复需 L3） | `/power_sequence/command shutdown` |
| **Triangle（▲）** | 按下即触发 | goto ready | `/a3/arm/goto_named_pose {name: ready}` |
| **Circle（●）** | 按下即触发 | goto home（非机械零位） | `/a3/arm/goto_named_pose {name: home}` |
| **Share** | 短按 | 开始示教 | `/a3/arm/start_teach`（零力矩拖动 + 记录） |
| **Options** | 短按 | 结束示教（自动保存 latest） | `/a3/arm/stop_teach` |
| **Square（■）** | 短按 | 回放最新轨迹 | `/a3/arm/playback {name: ""}` ≡ latest |
| **PS** | 短按 | init（设零 + 到位确认 + 自动使能） | `/a3/arm/init` |
| **L1** | 按住 | **平移死人开关**：仅放行平移轴（F64 gates） | 松开约 0.5 s servo 超时停 |
| **R1** | 按住 | **旋转死人开关**：仅放行偏航轴 | 同上 |
| **R2** | 模拟 | 夹爪力控（**不需要 L1/R1**），松开全开 | gripper_force |
| 左摇杆 X / Y | 模拟（L1） | 平移 Y（左右）/ Z（上下） | servo_lin_y / servo_lin_z |
| 右摇杆 Y / X | 模拟 | 平移 X（前后，gate l1）/ 偏航（gate r1） | servo_lin_x / servo_ang_z |
| **D-pad 上 / 下** | 点按 | 平移速度 ±0.15（0.10–1.0，按住不连发） | step_linear_scale |
| **D-pad 右 / 左** | 点按 | 旋转速度 ±0.15（独立于平移档） | step_angular_scale |
| touchpad / L2 | — | **预留不绑**（蓝牙触摸板无键事件，LL-052） | — |

口诀：**L3 开工、R3 收工、✕ 长按急停；▲ ready、● home；Share-Options-■ = 示教-保存-回放；PS 初始化；L1 平移、R1 旋转、十字键调两速。**

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
- 一个键可用 **YAML list** 绑多条，独立跟踪，互不干扰（F91 前 Options 的长按调零已退役：产品栈零点维护走独立节点 `motor_maintenance`）。

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
