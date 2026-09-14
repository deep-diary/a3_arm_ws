# LL-041 关节静置在 URDF 限位外 → 编排层 F48 拒绝使能（执行层能过，别放宽限位）

> **日期：** 2026-09-14  
> **产品线：** Edge  
> **环境：** RK3588 (lubancat) + ROS 2 Humble，can1 真机（事故后 L6/L7 缺失）

## 现象

F51 真机验收时 `/a3/arm/enable` 被拒：

```
position check failed: L4_joint=-1.0981 limit=-1.0472..1.5708
(possible multi-turn wrap after power cycle — restore URDF zero pose then /a3/arm/init)
```

同时 `/a3/motor/enable`（执行层广播）**成功**——同一姿态一个拒一个过。

- L4（肘）URDF 限位下界 -1.0472 rad（-60°，与官方 EDULITE_A3 URDF **完全一致**），
  事故后（腕部 L6/L7 及线束断裂、质量分布改变）肘部**静置平衡位 ≈ -1.099 rad**，
  即比额定下界低 ~0.052 rad；关节靠摩擦维持一阵后会缓慢滑到这个平衡位。
- 后果：**只要臂处于静止自然位，编排层就永远拒绝使能**（`enable_position_check: true`，
  `position_check_margin_rad: 0.001` 几乎无容差）→ move_to / FJT / 遥操 / web 全部不可用；
  执行层（含 F51 保护）却可以，容易误判成「F51 把编排层弄坏了」。

## 根因

- F48 使能门禁判的是**读数 vs URDF 限位**（防 LL-019 断电多圈环绕：kp×巨大误差瞬间猛拉），
  它**不看**目标会不会被重锚——F51 重锚后目标=反馈位、误差为 0，其实没有猛拉风险。
  两者语义都对，只是 F48 保守地在「姿态越限」这一条上直接拒。
- 本例不是环绕（偏差 0.052 rad，不是 ±2π），是**物理姿态真的越了额定范围**：
  官方 URDF 同值 → 不能改限位来迁就。

## 正确做法 / 规避

- **不要**为通过校验去放宽 URDF/限位，也**不要**把 `enable_position_check` 关掉；
  恢复路径是把关节挪回限位内：
  1. 失能态下直接**手动**把肘部抬回限位内（失能=无力矩，安全），或
  2. 用验收脚本的修复动作（执行层先使能，F51 保护下走一段小幅插值轨迹）：
     ```bash
     source scripts/a3_shell_env.sh
     A3_REAL_ARM_ACCEPT=1 python3 scripts/a3_test/f51_real_arm_acceptance.py \
         --nudge-delta 0.15          # P4b：L4 +0.15 rad、2.5 s 插值、≤0.30 rad 限幅
     ```
     随后**在仍使能的状态下**调 `/a3/arm/enable`（关节已在限位内 → F48 放行）。
- 排障顺序：编排层使能失败先看 message 里 `position check failed` 是**连续小偏移**
  （物理姿态越限，按上面挪回）还是 **±2π 跳变**（环绕，走 `/a3/arm/init` 恢复零位）。
- 长期：新结构（含腕部）装回后重新确认各关节静置位与额定范围；若物理范围确实需要
  超过官方限位，须走 SAFETY.md 评审改 URDF，而不是在门禁上开口子。

## 相关路径

- `src/a3_description/urdf/el_a3.urdf.xacro`（L4 `lower="-1.0472"`，与官方一致）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_check_positions_in_limits`）
- `scripts/a3_test/f51_real_arm_acceptance.py`（`--nudge-delta`，P4b 修复动作）
- [LL-019](LL-019-multiturn-wrap-zero-sta-firmware-split.md)（F48 门禁的由来：环绕读数）
