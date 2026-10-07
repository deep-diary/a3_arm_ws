# F60 — PS4 键位重设计（一键使能 / 单键硬急停 / 示教三键）


- **说明：** F55 键位在真机联调中暴露三个操作问题：(1) 上电与使能分属 Square 长按 + L3 两键两层，操作员分不清「已开机但未 READY」；(2) 急停是 Triangle 长按，与 Triangle 命名位姿的通用助记冲突；(3) 示教/回放分散在 Share/Options/Circle。重新设计 `config/mappings/default.yaml`（`simple.yaml` 保留作无 deadman 调试档）：

  | 按键 | 手势 | 动作 |
  |---|---|---|
  | L3 | 短按 | `arm_power_enable`：执行层 `power start` → 等 gate_open+Running → 编排层 `/a3/arm/enable`，一键到 READY（非阻塞轮询，5 s 超时） |
  | R3 | 短按 | `arm_disable`（F40：离 idle 先 SAFE_PARK 再失能） |
  | Cross(X) | **长按 1 s** | `power_shutdown` 硬急停：门禁关、电机失能；恢复需重新 L3 |
  | Triangle | 短按（rising） | goto 命名位姿 `ready` |
  | Circle | 短按（rising） | goto 命名位姿 `idle`（非 zero——机械零位 servo IK 奇异，LL-007） |
  | Share | 短按 | `teach_start` |
  | Options | 短按 / 长按 3 s | `teach_stop`（自动保存 latest，F54）/ `power_set_zero` |
  | Square | 短按 | `playback_latest`（空名 ≡ latest 槽位） |
  | PS | 短按 | `arm_init`（set_zero + 到位校验 + 自动 enable）（**F135 起改绑 L1+R1 同按长按 2 s，PS 解绑**） |
  | L1 | 按住 | deadman，**仅门控摇杆平移/偏航轴**（applies_to=analog_n11） |
  | R2 | 模拟量 | 夹爪力控，**不经 L1 门控**（kind=analog_01，F36 迟滞/力矩映射不变） |
  | R1 | 按住 | 速度档 0.35 ↔ 1.0 |
  | 左摇杆 | 模拟量 | servo 平移 Y（left_x）/ Z（left_y），需 READY + L1 |
  | 右摇杆 | 模拟量 | servo 平移 X（right_y）/ 偏航 Z 预留（right_x），需 READY + L1 |

  预留不绑定：D-pad 四键、触摸板键（蓝牙 js0 无此键事件，LL-052）、L2。命名位姿改走编排层服务 `/a3/arm/goto_named_pose`（a3_msgs/GotoNamedPose），不再由 teleop 本地插值直发 JointTrajectory——状态机进入 TRAJ（紫灯可见）并获得 F53 拒绝语义；删除 teleop 内 `/a3/goto_named_pose`（String）订阅（无其他发布者）。真机 launch 默认映射由 `simple` 改为 `default`。
- **验收标准：**
  1. mapper 启动 `validate_mapping` 零报错；options 双键列表各自独立边沿跟踪（短按释放判定不连带长按绑定）
  2. 合成 /joy 仿真（F62，domain 45）：12 场景全部 PASS——PS init→READY；Triangle/Circle 收敛容差 0.08 rad 且过程 state=TRAJ；L3 在已 READY 时幂等；R2 不按 L1 可开合 L7；摇杆不按 L1 不动、按住 L1 才动；Share/Options/Square 示教回放链保存 latest.yaml；R3 SAFE_PARK→DISABLED；X 长按 1 s gate 关闭；L3 可从关机/失能两态恢复 READY；Options 长按 3 s 发 set_zero
  3. teleop `/joint_states` 订阅为 BEST_EFFORT（真机 SensorDataQoS 兼容，LL-059）
  4. 文档：包 README 全键表 + SAFETY.md 急停/失能/deadman 段更新 + 操作员手册 PS4_OPERATOR_GUIDE.md
- **关联：** 取代 F55 键位表；F40（R3 失能保护）、F48（enable 越限拒绝）、F51（使能重锚）、F54（示教自动保存）、F36（R2 力控）、F61（灯/震反馈）、F62（合成验证）；[shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented-pending-sim`（2026-09-20 代码 + 配置；待 F62 合成验收 + 用户实操）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
