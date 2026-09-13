# LL-031 IK 求解器限位钳位索引错位：lowerPositionLimit 按 idx_qs 索引，按 joint id 索引每关节被钳到下一关节的界

- **日期**：2026-09-13
- **产品线**：Edge（MoveToPose IK，Pinocchio damped-LS）
- **严重度**：高（求解永失败只是表象；收敛校验同错位，理论上可放过真越界解）

## 现象

阶段 2-2 真机 IK 测试：伸展位（良态、远离奇异）目标 = 当前 EE +3cm x，`/a3/move_to_pose_ik` 连续返回 `ik did not converge`；离线无钳位复现同种子同目标 9 步收敛（err 6e-5）。

## 根因

Pinocchio 的 `model.lowerPositionLimit` / `upperPositionLimit` 是 **nq 维数组，按 idx_qs（构型向量下标）索引**。节点钳位/校验用 `model.getJointId(name)` 得到的 **joint id** 直接索引。本模型 7 个单自由度转动关节下 jid = idx_qs + 1，于是每个关节被钳到**下一个关节**的限位区间：

| 关节 | 应钳区间 | 实际被钳区间（错位） | 后果 |
|---|---|---|---|
| L1 | ±2.79 | [0.05, 3.62]（L2 的界） | 种子被强制 ≥0.05 |
| L2 | [0.05, 3.62] | [-3.96, -0.05]（L3 的界） | 0.6 直接被砸到 -0.05，越物理界 5cm |
| L3 | [-3.96, -0.05] | [-1.0, 1.52]（L4 的界） | 允许跑到 +1.5 |
| L6 | ±1.52 | [0.05, 1.74]（L7 的界） | 种子被强制 ≥0.05 |

种子每步被错位钳位踢出有效区，求解器死锁在边界上 200 步不收敛。收敛分支的「解在限位内」校验用**同样的错位索引**——L2=-0.33 的解用 L3 的界 [-4.01, 0] 校验反而通过，理论上会放行真越界解（此前靠无钳位版的漂移才暴露）。

## 修复

两处统一改为按 idx_qs 下标（iq）索引：

```python
# 钳位（每步）与收敛校验（解输出时）共用：
lo = float(self._model.lowerPositionLimit[iq]) + margin
hi = float(self._model.upperPositionLimit[iq]) - margin
```

修复后真机全链路通过：求解 9 步收敛（Δ≤0.19）、执行到位（EE +2.82cm/目标 3cm，残余 1.2cm 为 L3 重力 P 控制稳态误差 τg/kp≈0.036 rad，非 IK 误差）。

## 顺带确认（避免再次踩）

- **雅可比帧用 LOCAL 是对的**（与该 err 定义配套）：离线对比 WORLD 帧 lstsq 发散（q 爆炸至数百 rad），LOCAL 帧 9 步收敛到真解——保持 `computeFrameJacobian(..., pin.ReferenceFrame.LOCAL)`。
- **Quaternion 构造序是 (w,x,y,z)**：`pin.Quaternion(1,0,0,0).coeffs()=[0,0,0,1]`，节点 `pin.Quaternion(w,x,y,z)` 写法正确。
- **已知边界限制**：折叠 home 位 L3 顶着 URDF 上界 0（margin 0.05 把 L3 钉在 -0.05），home 附近 IK 目标可能死锁不收敛——安全失败（不输出解、不动臂）。home 区域运动用 move_to，不要用 IK。
- **IK action 语义是「轨迹发布即 succeed」**：客户端 FK 验证必须等轨迹完成+收敛（轮询 js ≤0.05），否则读到运动起点、误差恒等于偏移量（本次实测误差恰好 0.0300 = 偏移量的迷惑现象）。
