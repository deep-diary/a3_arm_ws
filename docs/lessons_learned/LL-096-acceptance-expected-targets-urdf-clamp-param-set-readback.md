# LL-096 — 验收脚本期望目标必须按 URDF 限位 clamp 后再比对（L3 上限=0）；ros2 param set 成功与否要读回确认

> **日期：** 2026-09-23  
> **产品线：** Edge  
> **环境：** RK3588 lubancat + ROS 2 Humble（el_a3.urdf 关节限位）

## 现象

F88 验收连续出现「jog 已成功执行、到位判据却永远 FAIL」的假红：从当前位姿给所有关节统一加正向偏移（+0.06/+0.20），实际轨迹已正常走完，但残留误差始终 ≈0.20。另有「已把 goto_use_moveit 设为 false，goto 却仍在等 move_group」的偶发失败。

## 根因

1. **URDF 限位不对称**：`el_a3.urdf` 中 **L3_joint 区间是 [−4.014, 0]**（上限为 0），L2 是 [0, 3.665]。状态机下发前对每个关节按 URDF 限位 clamp，所以「当前位姿 + 0.20」对 L3 实际只到达 0；验收拿未 clamp 的原始请求值做期望，残留自然恒为正偏移量。这是验收期望值错误，不是运动没执行。
2. **`ros2 param set` 的 CLI 输出不稳定**：用 stdout 里是否出现 "successful" 判断设置生效不可靠（守护进程陈旧、输出措辞变化都会误判）；参数没真正写成 false 时，goto 仍按 moveit 路径走。

## 正确做法 / 规避

1. 验收里对「请求目标」和「期望到位目标」分开：请求可故意越界以覆盖 clamp 逻辑，比对必须用按 URDF 限位 clamp 后的值。限位直接从已安装的 URDF 解析（`a3_description/urdf/el_a3.urdf` 的 `<limit>`），不要在脚本里复制常量表。
2. 记牢非对称限位：L2 只接受正、L3 只接受非正；写「所有关节 ±同向偏移」类测试用例时必须逐关节过限位。
3. 参数设置采用 set → `ros2 param get` 读回轮询确认（如读回 "boolean value is: false"），不要只看 set 的退出码/措辞。
4. CLI 行为异常（空 node list、set 无响应）先 `ros2 daemon stop` 再查，避免对着陈旧缓存调试。

## 相关路径

- `scripts/a3_test/f88_two_point_trajectory_acceptance.py`（`urdf_limits` / `clamp_targets` / `force_fallback`）
- `src/a3_description/urdf/el_a3.urdf`（L2/L3 非对称限位）
