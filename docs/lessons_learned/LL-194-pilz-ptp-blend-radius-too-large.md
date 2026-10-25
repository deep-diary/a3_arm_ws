# LL-194 — pilz PTP blend 半径过大：小段+大姿态跳变导致整条 stream 停在中途

> **日期：** 2026-10-25
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble + MoveIt pilz

## 现象

口风琴 `st_extremes` 播放第 1、2 段正常，第 3 段（白19 → 黑13）连续三次失败：

```
Intersection point of first trajectory not found.
Blend radius too large.
Unable to find IK solution for sampled Cartesian pose.
stream st_extremes 3/5 attempt N failed: sequence error_code=99999
playback st_extremes: seg 3 failed
```

重试是原样重跑，三次结果一致 → 停在第 2 个键。

## 根因

pilz PTP 的 blend 走**笛卡尔过渡窗**（对采样位姿求 IK）。`_adaptive_blend_radii` 按
`r = min(0.02, 0.45 × 最短相邻段)` 算出 10.6mm；而白19→黑13 水平只差 ~35mm、示教姿态
跳变大，24mm 的短腿配 10.6mm 圆角让过渡窗超出 IK 可行域 → “Blend radius too large”。

## 正确做法 / 规避

- stream 的 `ptp_blend` 半径再收紧到 **≤6mm**（普通路点回放仍用 `waypoint_blend_radius_m`）；
- 序列失败时自动用 **全部 exact（blend=0）重试一次**，保证一条 stream 走完（日志见
  `blend→exact fallback`，该段多几次停车）；
- 仍失败时：减小 `--clear-max/--beta` 缩短弧线，或该段改 `jnt_g64_quintic`（纯关节圆角，
  不走笛卡尔混合）。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_goto_via_ptp_blend`、
  `_adaptive_blend_radii`）
- `src/a3_arm_controller/a3_arm_controller/melodion_stream.py`
- `trajectories/melodion/streams/*.yaml`
