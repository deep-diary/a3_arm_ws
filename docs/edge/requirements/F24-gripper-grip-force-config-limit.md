# F24 — 夹爪握力配置与安全限幅


- **说明：** 第 7 电机（L7 夹爪）新增独立握力参数配置，控制「手抓不能抓太紧」。新增 `a3_gripper_controller/config/gripper_config.yaml`：最大握力硬上限 `max_grasp_torque_nm`（出厂不可越界）、默认目标力与弱/中/强档位 `torque_presets_nm`、夹爪力矩方向符号 `gripper_torque_sign`（真机标定，把「夹紧阻力」校正为正）、PI 参数（`force_kp`/`force_ki`/积分限幅/位置增量速率限幅）、位置/速率限幅、接触判定阈值、抓取/看门狗超时。节点启动（电机使能）时经现有 `/a3/motor/set_param` 服务（`motor_id=7`，`param_id=0x700B` 力矩限制）把固件级力矩上限写入电机，形成「固件硬限 + 节点软件 clamp」双保险。参数支持 YAML 默认值与运行时 web/服务下发：下发值一律校验 `≤ max_grasp_torque_nm` 且在 MIT 力矩量程（±6 Nm）内，越界拒绝；通过后落盘 `data/gripper_overrides.yaml`，重启自动加载。
- **验收标准：**
  1. `gripper_config.yaml` 含上述全部参数且注释标明单位与典型取值；缺 key 时节点用内置安全默认值启动并告警
  2. 调用配置服务把最大握力设为合法值：返回成功，落盘文件更新，重启节点后值保留
  3. 下发超过 `max_grasp_torque_nm` 或超过 ±6 Nm 的值：服务返回失败且不改变当前配置
  4. 电机使能流程中 `/a3/motor/set_param`（ID7, 0x700B）被调用且值与配置一致；服务失败时节点不上报就绪并报错
  5. 弱/中/强三档目标力均 ≤ 硬上限；不修改 L1–L6 的任何增益/限位配置
- **关联：** [shared/SAFETY.md](../shared/SAFETY.md)（夹爪力控安全段）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；`a3_gripper_controller/config/gripper_config.yaml`；F17（`/a3/motor/set_param`）；F26（配置服务/MQTT）
- **状态：** `implemented`（仿真闭环验证；真机 0x700B 写入与力矩方向标定板测中。2026-09-07：出厂硬上限 2.0→1.0 Nm（1.5 持续出力几分钟过热，见 LL-014），PI 减半为 kp=0.25/ki=0.3，运行时持久化上限同步 1.0 Nm；sim 植物接触方向随 2026-09-06 标定反转。同日起超硬限 FAULT 瞬态带放宽为 `overtorque_ratio` 1.0→1.5：硬物体 PI 过冲会顶穿 1.0 误报 `error_code=4`，软件 FAULT 只做失控兜底，固件 0x700B 仍硬钳 1.0 Nm）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
