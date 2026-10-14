# F169 — 标定子命令集成（a3_stack.sh calibrate）+ F49 脚本 6 关节适配


- **说明：** 硬件拆掉夹爪后整臂质量/质心变化，重力前馈标定参数（`inertia_params.yaml` F49 模型 / `gravity_scales.yaml` F89 比例）失配，零力矩拖动「变重」。标定脚本（`scripts/gravity_calibration.py`、`scripts/gravity_scale_calibration.py`）可用但**入口零散、参数多、难记**；且 F49 脚本**写死 7 关节**（`/joint_states` 需含 L7、轨迹发 7 个关节名、`tau[:7]`、温度 `==7`、`q[:7]`），在 6 关节产品栈上直接跑会因缺 L7 / JTC 拒绝 7 名而失败。本需求：把两套标定收进 `a3_stack.sh calibrate {model|scale}` 子命令（含前置检查与提示），并让 F49 脚本支持 N∈{6,7} 关节。**7 关节默认行为不得改变。**
- **验收标准：**
  1. `./scripts/a3_stack.sh calibrate` 打印标定帮助与现状（栈/臂状态/档位）
  2. `./scripts/a3_stack.sh calibrate model [额外参数…]` → 以正确档位（`--joints 6|7`）运行 `gravity_calibration.py`；`calibrate model --dry-run` 在 6 关节栈上只打印网格（不动臂）
  3. `./scripts/a3_stack.sh calibrate scale [额外参数…]` → 运行 `gravity_scale_calibration.py`
  4. 前置不满足时明确报错：栈未运行 / 臂未到 READY（不做任何运动）
  5. F49 脚本 `--joints 7`（默认）行为逐字不变；`--joints 6` 时 `JOINT_NAMES`/符号/网格/`tau` 长度/温度判据全按 6 关节
  6. 标定完提示生效方式（F49 需重启栈/重力节点，F89 下次起栈自动加载）
- **关联：** F49（重力模型标定）、F89（逐关节重力比例）、F168（产品栈 6 关节档，本需求的前置）、[LL-158](../../lessons_learned/LL-158-calibration-entry-and-6j-f49-adaptation.md)
- **实现：**
  - `scripts/gravity_calibration.py`：新增 `--joints {6,7}`（默认 7）；据此设置 `JOINT_NAMES`/`JOINT_SIGNS` 及 `home/rest` 位姿长度；放开写死的 7（`tau[:len(q7)]`、温度 `== len(JOINT_NAMES)`、`q[:len(cfg)]`、anchor/符号长度校验、`run_optimize` 的 `ones/range`）
  - `scripts/a3_stack.sh`：新增动作 `calibrate [model|scale]`；`_cal_common_checks()` 前置校验（真机栈在跑 + `/a3/arm_status`=READY + 探测 `/joint_states` 档位）；`do_calibrate()` 按档位透传 `--joints` 并转发额外参数
  - 文档：QUICKSTART 标定节补 `a3_stack.sh calibrate` 用法；`scripts/README.md`
- **状态：** `implemented`（2026-10-13）
  - 代码：F49 加 `--joints {6,7}` 全套按 `len()` 自适应（默认 7 不变）；`calibrate` 前置校验与档位透传就绪
  - 真机（6J，拆掉夹爪）：`calibrate model --joints 6 --no-rest-anchor --anchors <home>` 采集 **68/68 点完成**（SSH 断连未影响已落盘数据）；离线 `--optimize-only` 拟合 **RMSE 0.0619 Nm / R² 0.9938（PASS）**，对照含夹爪旧模型同批数据 RMSE **0.6919** → 确认「拆夹爪后补偿失效」，重拟合修正；`inertia_params.yaml` 已写（旧文件自动备份）
  - **收尾补漏**：`residual_report` 的 `np.zeros(7)` 是第二处 7 写死（漏改致拟合崩、yaml 未写），已按 `len(records[0][0])` 修正；数据已落盘 → 修复后 `--optimize-only` 离线重拟合即可，**无需重采**
- **未覆盖项（有意）：** 未做「一键采集+生效」全自动（真机运动须有人监护，脚本只提供记忆入口 + 前置校验）；F89 未加 `--joints`（其 `ARM_JOINTS` 本就是 L1–L6，6/7 关节栈通用）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
