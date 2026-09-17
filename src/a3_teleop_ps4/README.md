# a3_teleop_ps4 — PS4 全功能映射（遥控 / 示教 / 回放）

> **PS4 映射写在这里就能查：本文件**。改键位只编 `config/mappings/default.yaml`，零代码。
> 需求与验收见 [docs/edge/REQUIREMENTS.md](../../docs/edge/REQUIREMENTS.md) F55；安全合同见 [docs/shared/SAFETY.md](../../docs/shared/SAFETY.md)。

## 完整映射表（`mapping:=default`，F55）

示教主流程三键：**Share 短按 = 开始示教，Options 短按 = 结束示教（自动保存），Circle 短按 = 执行回放（默认最新）**

| 键 | 边沿 | 动作 | 说明 |
|----|------|------|------|
| **L1** | 按住 | deadman 死人开关 | 松开即停；只有它按住时摇杆/R2 才有效（F36） |
| **R1** | 按住 | 加速 | 速度 0.35 → 1.0 |
| **Share** | 短按 | 开始示教 | `/a3/arm/start_teach`：切零力矩拖动 + 开始记录 |
| **Options** | 短按 | 结束示教 | `/a3/arm/stop_teach`：停记录 → F54 自动保存 `latest.yaml` |
| **Options** | 长按 3 s | 调零 | `/power_sequence/command set_zero`（互斥：长按不会连带触发短按停止示教） |
| **Circle** | 短按 | 执行回放 | `/a3/arm/playback {name:""}` 空名 ≡ 回放 `latest.yaml` |
| **Touchpad** | 短按 | 初始化 | `/a3/arm/init`（设零 + 到位确认；F51 越限只 WARN 不阻断） |
| **L3** | 短按 | 使能 | `/a3/arm/enable`（安全门禁：目标重锚反馈位 + 软起步 0.8 s） |
| **R3** | 短按 | 失能 | `/a3/arm/disable`（F40：离 home 先 safe park 再失能） |
| **Triangle** | 长按 1 s | **关机（唯一手柄急停）** | `/power_sequence/command shutdown` |
| **Square** | 长按 1 s | 上电 | `/power_sequence/command start` |
| **Cross** | 短按 | 立即停 | 零 Twist + 中止命名姿态 |
| **D-pad ↑ / ↓ / ← / →** | 短按 | 命名位姿 | ready / zero / home / ready |
| 右摇杆 X / Y | 模拟 | 平移 Y / Z | L1 按住时有效 |
| 左摇杆 Y / X | 模拟 | 平移 X / 偏航 | L1 按住时有效 |
| **R2** | 模拟 | 夹爪力控 | F36：按得越深抓得越紧（0.1..1.0 Nm）；松开全开 |

速记口诀：**「Share-Options-Circle」= 示教三步（开始-结束-执行）；L3 使能、R3 失能、触摸板初始化；Triangle 关机、Square 开机（都要长按）。**

## 映射在哪里改 / 看

| 文件 | 作用 |
|------|------|
| `config/mappings/default.yaml` | **真机全功能映射（本表驱动的 YAML）** |
| `config/mappings/simple.yaml` | 仿真直测映射（无 L1 deadman；防真机误用） |
| `config/action_registry.yaml` | 动作函数目录（加新动作才需要代码） |
| `config/ds4_linux.yaml` / `ds4_linux_usb.yaml` | 按键/轴索引（重校准索引时才碰） |
| `a3_teleop_ps4/actions.py` | ActionExecutor（函数实现） |
| `a3_teleop_ps4/ps4_mapper.py` + `mapping.py` | 边沿引擎（rising / shortpress / longpress） |

## 边沿语义（F55）

- `rising`：按下瞬间触发一次。
- `shortpress`（短按）：**释放时**判定，按住持续 < `hold_s`（默认 3 s）触发一次；按住超过 `hold_s` 再释放 = 本次作废（不误触、不连带同键其他绑定）。
- `longpress`（长按）：按住 ≥ `hold_s` 触发一次。
- 双义键（如 Options）用 **YAML list** 绑两条，每条独立跟踪 →「短按=结束示教 / 长按 3 s=调零」互不干扰。

```yaml
buttons:
  options:
    - fn: teach_stop
      edge: shortpress
      hold_s: 3.0
    - fn: power_set_zero
      edge: longpress
      hold_s: 3.0
```

## 启动

```bash
# 仿真（默认 simple 直测映射）：
ros2 launch a3_bringup edge_teleop_sim.launch.py mapping:=default use_rviz:=true

# 真机（域 0，arm_controller 必备）：
ros2 launch a3_teleop_ps4 ps4_teleop.launch.py mapping:=default
# 或带底栈一起：ros2 launch a3_bringup a3_bringup.launch.py use_teleop:=true mapping:=default
```