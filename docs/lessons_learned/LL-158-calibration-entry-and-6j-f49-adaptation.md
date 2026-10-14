# LL-158 — 标定脚本的 7 关节写死 + 标定入口收口

> **日期：** 2026-10-13
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble

## 现象

拆掉 L7 夹爪电机、切到 6 关节产品档（F168）后，零力矩拖动明显变重（重力前馈模型仍按含夹爪的
质量/质心算），需重标。但 `scripts/gravity_calibration.py`（F49）**写死 7 关节**，直接在 6 关节栈上跑会失败：
`on_js` 要求 `/joint_states` 含 `L7_joint`（缺 → `cur_pos` 永不更新 → `wait_js` 超时），
`move_to_position` 发 7 个关节名给 `/arm_controller`（6 关节 JTC 拒绝）；`predict_gravity` 返回 `tau[:7]`、
温度判据 `==7`、`q[:7]=cfg`、符号/anchor 长度校验 `!=7` 也都按 7 写死。

## 根因

F49 脚本源自 7 关节真机（官方 7J），把「7」散落在关节名表、符号表、位姿长度、张量切片与判据里，
没有随关节档位参数化；F89 脚本则本就以 `ARM_JOINTS=L1–L6` 工作，不受影响。

**收尾时又漏了一处（实测踩到）**：`residual_report` 里 `last_dir/prev_q = np.zeros(7)` 也是写死 7，
6 关节下残差诊断 `np.where(moved, …, last_dir)` 形状 (6,)/(6,)/(7,) 广播失败 → **采集跑完 68 点、
但拟合在残差诊断处崩、yaml 没写**。教训：**「7」要一次 grep 干净**（`zeros(7)`/`ones(7)`/`range(7)`/`[:7]`/`== 7`），
别只改主路径；漏一处就前功尽弃（但数据已落盘 → 修完 `--optimize-only` 离线重拟合即可，不必重采）。

## 正确做法 / 规避

- **标定/建模类脚本一律按关节档位参数化**：本仓加 `--joints {6,7}`，据此设 `JOINT_NAMES`/`JOINT_SIGNS`
  与 `home/rest` 位姿长度，所有长度按 `len(...)` 自适应（`tau[: len(q7)]`、`== len(JOINT_NAMES)`、
  `q[: len(cfg)]`、符号/anchor 校验按 N）。**默认 7 保持行为不变**。
- **入口收口到 `a3_stack.sh calibrate {model|scale}`**：前置自检（真机栈在跑 + 臂 READY + 探测
  `/joint_states` 关节数），自动透传 `--joints`，额外参数原样转发；避免每次手敲长命令与踩参数坑。
- 拆装末端负载（夹爪/工具）后**必须重标重力**：改质量/质心用 F49（模型重拟合，最彻底），
  只调整体幅度可用 F89（逐关节比例）。生效方式不同：F49 需重启栈/重力节点，F89 下次起栈自动加载。
- 6 关节臂标定 `--anchors` 给 **6 个**值（用 ready 自由位姿；折叠 rest 位顶限位 τ≈0，非重力样本，LL-035）。

## 相关路径

- `scripts/gravity_calibration.py`（`--joints`、`JOINT_NAMES`/`JOINT_SIGNS` 自适应）
- `scripts/gravity_scale_calibration.py`（本就 L1–L6）
- `scripts/a3_stack.sh`（`calibrate` 子命令）
- 需求：[F169](../edge/requirements/F169-calibration-entry-and-6j-adaptation.md)、[F168](../edge/requirements/F168-product-6j-profile.md)
- 相关：[LL-157](LL-157-product-6j-profile-include-gripper.md)、[LL-035](LL-035-gravity-rest-anchor-non-sample.md)、[LL-037](LL-037-joint-states-effort-motor-domain.md)
