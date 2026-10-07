# F16 — PS4 映射遥操作（笛卡尔 + 夹爪）


- **说明：** YAML 将手柄轴映射到带归一化参数的函数（`analog_01` / `analog_n11`），按键映射到无参动作。默认 **`mapping:=simple`**：D-pad 命名姿态、双摇杆 MoveIt Servo（`base_link` 系）、L2/R2 直控 L6/L7、Square/Circle 夹爪开/合；**无 L1 死人开关**（开发调试用）。生产/安全映射用 **`mapping:=default`**（L1 按住才发 Servo Twist 与 R2 夹爪）。
- **验收标准：**
  1. `joy_dump` 能打印当前手柄全部 `axes[]` / `buttons[]`；轴序写入 `ds4_linux.yaml` 后映射生效
  2. 改 `config/mappings/*.yaml` 即可换绑，不必改 Python
  3. 仿真 `edge_teleop_sim.launch.py`（`simple`）：右摇杆基座系左右/上下；左摇杆 Y 前后；松杆停止
  4. `simple`：L2→L6、R2→L7 模拟量；Square/Circle 夹爪开/合；`default`：L1+R2 夹爪
  5. D-pad 上/下/左/右分别到 `ready` / `zero` / `idle` / `ready`（上键暂与右键同）
  6. Cross 立即停；Square/Triangle/Options 长按电源语义与 F3 一致（`default` 映射）
  7. 无 `/joy` 或 1 s 无更新时 mapper 不发任何轨迹/Servo 令
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；[shared/SAFETY.md](../shared/SAFETY.md)；`a3_teleop_ps4`
- **状态：** `implemented`（仿真路径；无手柄门控与 Servo/命名姿态仲裁已自动化验；**手柄轴向与真机 CAN 板测待办**）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
