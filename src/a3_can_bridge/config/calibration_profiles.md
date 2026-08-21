# A3Arm CAN 标定配置档案（calibration profiles）

本文档用于管理 `control_gains.yaml` 中与零位标定相关的两套场景参数，避免现场切换时混淆。

---

## 1. 适用文件

- `a3_can_bridge/config/control_gains.yaml`

关键参数：
- `expand_joint_offsets_from_mirror`
- `joint_offset_hip_rad`
- `joint_offset_thigh_rad`
- `joint_offset_calf_rad`
- `joint_offsets_rad`（用于人工核对，运行时以 mirror 三参数为准）

---

## 2. 场景定义

## 场景A（当前调试场景）

- 机械状态：未装外壳，底部垫高，机身保持水平。
- 零位口径：在该姿态执行 `set_zero`。
- 结论：URDF 髋关节零位与电机零位重合，`joint_offset_hip_rad = 0.0`。

推荐参数：
- `joint_offset_hip_rad: 0.0`
- `joint_offset_thigh_rad: 1.0031`
- `joint_offset_calf_rad: 2.5469`

对应展开（核对用）：
- `[0.0, 1.0031, -2.5469, -0.0, -1.0031, 2.5469, -0.0, 1.0031, -2.5469, 0.0, -1.0031, 2.5469]`

## 场景B（装壳后场景）

- 机械状态：装外壳，外壳触地，4 个髋关节相对水平零位上翘。
- 零位口径：沿用趴姿 `set_zero`，但髋关节需补偿。

推荐参数（参考）：
- `joint_offset_hip_rad: 0.17444`
- `joint_offset_thigh_rad: 1.0031`
- `joint_offset_calf_rad: 2.5469`

对应展开（核对用）：
- `[0.17444, 1.0031, -2.5469, -0.17444, -1.0031, 2.5469, -0.17444, 1.0031, -2.5469, 0.17444, -1.0031, 2.5469]`

---

## 3. 场景切换 SOP

1. 打开 `control_gains.yaml`。
2. 确认 `expand_joint_offsets_from_mirror: true`（建议保持 true）。
3. 按目标场景修改：
   - 场景A：`joint_offset_hip_rad: 0.0`
   - 场景B：`joint_offset_hip_rad: 0.17444`
4. `joint_offset_thigh_rad` 与 `joint_offset_calf_rad` 保持当前标定值（若未重新标定）。
5. 同步更新（或核对）`joint_offsets_rad`，确保与三参数一致。
6. 重启 bridge 节点/launch，使参数生效。

---

## 4. 生效后快速检查

建议顺序：

1. 零位检查  
   - 执行 `set_zero` 后，观察 12 路反馈角是否在零点容差内（如 `±0.08 rad`）。
2. 单关节方向检查  
   - 小幅正负命令，确认方向符合 `joint_signs`。
3. 越限检查  
   - 验证限幅是否生效（不得触碰机械硬限位）。
4. 姿态检查  
   - RViz 与实机姿态趋势一致（重点看 4 个髋关节）。

---

## 5. 回滚策略

如果装壳后参数不稳定，回滚到场景A：

- `joint_offset_hip_rad` 恢复 `0.0`
- 保留 `joint_offset_thigh_rad: 1.0031`
- 保留 `joint_offset_calf_rad: 2.5469`
- 重启节点并重新执行零位检查

---

## 6. 变更记录模板（建议）

- 日期：
- 操作人：
- 机械状态（A/B）：
- 修改参数：
- 零位检查结果：
- 单关节方向检查结果：
- 备注：

