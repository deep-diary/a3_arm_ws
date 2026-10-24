# LL-183 — servo 角速度积分漏乘 dt_step：实测超转 2~4 倍且与指令时长无关

> **日期：** 2026-10-24
> **产品线：** Edge
> **环境：** LubanCat RK3588 + ROS 2 Humble，真机 6 关节档（L1–L6）

## 现象

R1 三轴旋转验收（`scripts/a3_test/f183_servo_rotation_check.py`）6/6 fail：

- 指令 0.100 rad @ 0.15 rad/s（0.67 s），实测 `0.400 / 0.131 / 0.197 / 0.154 / 0.155 rad`（X/Y±/Z±）；
- **把指令角减半到 0.050 rad，实测几乎不变**（X 0.353、Y 0.136/0.193、Z 0.155/0.132）→ 与时长不成比例；
- 轴向 dot 初判 ≈ 0 / −0.2（其实是测试脚本 bug，见下）；位置漂移只有 0.4–3 mm；
- 伺服日志每段开头都刷 `IK 解远离实测 0.20~0.25 rad > 0.200 → 判为另一分支` 与 `IK 限步跟进 dmax=0.22`。

## 根因

`a3_servo_target` 的绝对目标积分里，**线速度乘了 `dt_step`、角速度没乘**：

```cpp
const double th = w.norm();                  // rad/s，却被当成"本拍转角 rad"
if (th > 1e-9) d.linear() = AngleAxisd(th, w / th)...
d.translation() = v.head<3>() * dt_step;      // 线速度有 dt
```

于是目标姿态每 10 ms 周期前进 `|w|` 弧度（0.15 rad/s 档 ≈ 100 倍速），CLIK 追不上 →
每拍判"远离实测/换支"回退 → 实测停在 0.4 rad 这类**与指令大小无关**的固定位置。

定位关键：**用"指令减半、实测是否减半"区分"时间/增益错"与"目标跑飞"**；再用
`/tmp` 临时脚本按 100 ms 打印实测 FK 姿态角 + 关节角，看到"只有 L6 一路走到 0.40"，
立刻能判断是关节被拖到某处而不是速度标定偏了。

附带两条测试脚本自身缺陷（先修脚本再判伺服，否则指标不可信）：

1. `rot_angle_axis(ra, rb)` 返回的轴表达在 **ra 的工具系**，直接与 base 系的 X/Y/Z 做 dot
   恒接近 0 → 必须 `axis_base = ra @ axis_local`；
2. 随机相位把多次旋转的角度**标量相加**当期望值 → 多轴/正负必须按旋转矩阵复合
   （`R_cmd = R_seg · R_cmd`），期望终态 `R_cmd · R_ref`，残差 = `angle((R_cmd·R_ref)ᵀ · R_end)`。

## 正确做法 / 规避

- 修法：`const double wnorm = w.norm(); const double th = wnorm * dt_step;`
  （`rotation_about_tip_` 两个分支都要改），改完必须 `colcon build` + `a3_stack.sh restart`。
- 修后实测：6 轴探针 `0.099–0.110 rad`（指令 0.100）、dot `1.00`、位置漂移 0.3–3.7 mm，随机相位逐段 13.0–13.2°（指令 12.9°）。
- **home 位形（全伸、工具水平）做绕末端点旋转会先撞 L4 限位**：纯姿态旋转要保持末端点不动，
  必须整臂重构，L4（−1.05…+1.57 rad）在累计约 26–31° 倾斜后到限位、`ik_fail` 上涨、后续段走不动。
  这不是伺服 bug；脚本已把"关节触限位"识别为提前结束条件而不是失败项。做大幅旋转验收请从半折位形起（如 L5≈−0.9）。
- 建议判据：单次定向探针用"角度幅值 ±25% + 轴 dot>0.8 + 位置漂移 <5 mm + 反向回位残差"；
  随机相位用**逐段**核对，别用累计终态（限位/耦合会让累计值天然对不上）。

## 相关路径

- `src/a3_trajectory_processing/src/servo_target_node.cpp`（step() → 绝对目标积分）
- `scripts/a3_test/f183_servo_rotation_check.py`
- `docs/edge/requirements/F183-face-tracking-servo.md`（§4.8 servo 旋转语义）
