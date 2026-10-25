# LL-193 — `pin.Quaternion(x,y,z,w)` 顺序写反，6D IK 静默回落 position-only

> **日期：** 2026-10-25
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble + Pinocchio/Eigen

## 现象

口风琴按键 stream 带 `hold_orientation: true`，但每个点日志都刷：

```
6D IK 未收敛（保持示教姿态不可达），该点回落 position-only
```

播放姿态与示教点明显不一致（腕部姿态漂），键位 xyz 正确，故很容易误判为“示教姿态不可达”。

## 根因

`_pose_ref_rotation` 用 `pin.Quaternion(x, y, z, w)` 构造参考姿态；Eigen/pinocchio 的
**4 标量构造顺序是 `(w, x, y, z)`**。四元数被重排后参考旋转整体偏差约 90°，
6D IK 的旋转残差常驻 ~1.5 rad，`|res| < 1e-3` 永远不满足 → 静默回落 position-only。
（`RandomPoseTour` 的 `ref_quat` 也用了 `pin.Quaternion(*ref_quat)`，同样问题。）

## 正确做法 / 规避

- 用 coeffs 数组最稳：`pin.Quaternion(np.array([x, y, z, w]))`；或显式 `pin.Quaternion(w, x, y, z)`；
- 离线核对：`scipy.spatial.transform.Rotation.from_quat([x,y,z,w])` 与 FK 旋转矩阵比
  `log3(R_ref.T @ R_fk)`，应为 0；
- 修后录制点 6D IK 残差 0，日志不再出现“6D IK 未收敛”。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_pose_ref_rotation`、`RandomPoseTour.ref_quat`）
- `src/a3_arm_controller/a3_arm_controller/melodion_stream.py`（stream 各点姿态来源）
