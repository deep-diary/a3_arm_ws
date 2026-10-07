# F135 — init 键改绑 L1+R1 长按 2 s，PS 解绑防开机误触


- **说明：** 当前 `default.yaml` 将 `arm_init` 绑在 **PS 键短按**。PS 键是手柄开机/唤醒键，开机过程中极易误触导致重复 init（状态机虽会拒绝，但操作员感受是「按了没反应/报错」）。改为 **同时按住 L1+R1 再长按 2 s** 触发 init，大幅抬升误触门槛；PS 键不再绑定任何操作。

- **实现方式：** 新增虚拟按钮 `l1_r1`（组合键），以 `and` 逻辑对 L1/R1 两个物理按钮做采样，注册到 `ds4_linux.yaml` / `ds4_linux_usb.yaml`。mapping 引擎当前已有 `virtual_buttons`（轴阈值方式），新增 `type: combo` 语义，支持 `buttons: [l1, r1]` 列表。`default.yaml` 取消 `ps` 绑定，新增 `l1_r1` 绑定 `arm_init`（longpress，2 s）。

- **验证与追溯：**
  - `validate_mapping` 零报错；sim 脚本场景 S1 改发 L1+R1 同时按下 2.5 s（含 0.5 s 裕量）并断言 READY+白闪
  - PS 键单独短按不再触发任何动作（回归断言）
  - 手册/SAFETY 键位总览同步替换 PS→L1+R1

- **关联：** F60（键位总览）、F62（仿真验收脚本）、F134（上一条需求，后续编号连续）；[PS4_OPERATOR_GUIDE.md](PS4_OPERATOR_GUIDE.md)、[src/a3_teleop_ps4/README.md](../../src/a3_teleop_ps4/README.md)
- **状态：** `implemented`（2026-10-03：mapping.py 组合键 + 双 layout 注册 l1_r1 + default.yaml 改绑 + sim S1 全绿——PS 解绑回归断言通过、L1+R1 长按 2 s init → READY + 白闪；S4/S10 4 项 FAIL 为仿真已知差异，与本改动无关；真机实操待复测）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
