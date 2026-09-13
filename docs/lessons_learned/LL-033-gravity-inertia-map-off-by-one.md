# LL-033 重力标定杆映射错位一位：L2→l1_link 是 L2 上方杆，改写只影响 τ₁（等效未生效）

- **日期**：2026-09-13
- **产品线**：Edge（gravity_torque_node 标定惯量改写 + F49 复刻 dynamics_calibration）
- **严重度**：高（重力补偿的 5 根杆标定值全部落在错杆上，τ_g 与姿态关系错误；标定拟合的 L2 参数梯度恒零）

## 现象

F49 复刻官方 dynamics_calibration 时，先用合成数据离线冒烟拟合链路：以已知「真值」参数生成 67 位姿的 τ，L-BFGS-B 拟合后 **rmse 已到噪声地板（0.019 ≈ 噪声 σ）但 L2 质量参数停在初值纹丝不动**（p00 恒等于 FIT_INIT[0]），其余参数互相补偿也凑到噪声地板——拟合「成功」但参数全错。

## 根因

Pinocchio 里 **body i = 关节 i 的子杆**（`l1_link_urdf_asm` 是 L1_joint 的子杆，body 1；l2_l3 是 L2_joint 的子杆，body 2；根固定关节 base_to_l1 被解析器内联）。τ_i 只依赖第 i 关节**下方**的杆。

官方脚本按 `model.inertias[2..6]` 下标改 = L2→l2_l3、L3→l3_lnik、L4→l4_l5、L5→part_9、L6→l5_l6（各关节下方杆，物理正确）。而 gravity_torque_node 的 joint_to_link 映射**整体上移一杆**（L2→l1_link_urdf_asm 等）——l1_link 是 L2 上方杆，改写它只影响 τ₁；τ₁ 又不在拟合目标（官方目标只算关节 1..5 = L2..L6）里 → L2 参数梯度恒零。其余参数因 τ₂ 里 m₂·c₂x 与 m₃·L₂ 同随 cos(θ₂) 变化（线性相关平坦方向）互相补偿凑到噪声地板，掩盖了映射错误。

## 修复

两处映射统一改为「各关节下方杆」（与官方 inertias[2..6] 同下标）：

```python
joint_to_link = {
    "L2": "l2_l3_urdf_asm",   # 旧：l1_link_urdf_asm（L2 上方杆）
    "L3": "l3_lnik_urdf_asm",
    "L4": "l4_l5_urdf_asm",
    "L5": "part_9",
    "L6": "l5_l6_urdf_asm",   # 旧：part_9
}
```

gravity_torque_node.py 与 scripts/gravity_calibration.py（FIT_LINK_MAP）同步修。

## 验证与判据教训

修复后离线冒烟：拟合 rmse 0.0193（噪声地板）、网格外姿态（折叠 home/ready/伸展）τ 外推误差 ≤0.014 Nm。**判据教训**：重力拟合有固有平坦方向（m₂·c₂x 与 m₃·L₂ 共线、τ₁ 缺失），「参数找回真值」不是有效验收——**重力补偿只用预测 τ，验收必须判 τ 预测精度**（F49 验收 RMSE ≤0.15 Nm、home/ready |Δτ|≤0.2 Nm 即为此）。参数错但 τ 预测对的等价参数集完全可用；拟合初值接近 URDF 值 + 有界约束是平坦方向的隐含正则。

## 关联

- [[LL-025]] 重力节点 QoS/初始化顺序事故——当时「Applied 5 links」只验证了数量没验证落杆，映射错位逃过。
- F49 标定（scripts/gravity_calibration.py）拟合目标关节 1..5 同官方；锚点位姿（rest/--anchors）追加网格外姿态防外推漂移。
