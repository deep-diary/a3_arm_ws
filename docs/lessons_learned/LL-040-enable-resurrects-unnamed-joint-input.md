# LL-040 使能后「未点名关节」的历史输入会复活：F51 重锚漏了 `latest_input_champ_rad_`

> **日期：** 2026-09-14  
> **产品线：** Edge  
> **环境：** RK3588 (lubancat) + ROS 2 Humble，can1 真机（F52 5J 档）

## 现象

F51（LL-039）真机验收前的代码排查里发现：使能（`/a3/motor/enable` / `/a3/arm/enable`）会把
**已命名的逐关节目标**重锚到反馈位，但**没有清**「未点名关节的回退目标」`latest_input_champ_rad_`。

于是这条路径成立：

1. 使能前播放过一条满关节轨迹 → `latest_input_champ_rad_` 里留下使能前的旧值（比如 L2=0.6）；
2. stop / 复位 / 失能（目标重锚到反馈位，臂被人工搬走）；
3. 重新使能 → F51 把「命名目标」重锚到当前位，**回退目标仍是第 1 步的旧值**；
4. 此后任何**只点名部分关节**的轨迹（典型：只点名 L7 夹爪的轨迹；在 F52 5J 档下它连
   一个档位内关节都没点名）走 `ApplyPositionTargets` 的 else 分支 → 未点名关节被
   **回退到使能前的旧值**驱动。等于「使能执行历史目标」，与 LL-039 事故同一类。

仿真回归里 T5 用旧代码复现：满关节轨迹把 L2 带到 0.6 → stop/reset/重锚 → 只点名 L1 的
轨迹 → **L2 仍被驱向 0.6**（新代码下 L2 保持在重锚位）。

## 根因

- F51 重锚只覆盖了「轨迹命名过的关节」（`last_commanded_mit_rad_` / 插值目标），漏了
  Named 轨迹的**回退表** `latest_input_champ_rad_` + `has_latest_input_`。
- 回退表是「上一条输入的历史」，跨使能边界存活；它在语义上属于**陈旧目标**，必须与
  命名目标一起在使能时清掉。
- 单独看不危险（每个关节的目标 = 反馈位），但和「部分点名轨迹」组合起来就是一次
  不加限制的历史目标回放——和 LL-039 的成因是同一根：**目标在意图边界没有全部失效**。

## 正确做法 / 规避

- 使能处理里，除了重锚命名目标，**逐关节把 `has_latest_input_[i] = false`**（清回退表）；
  之后未点名关节由 refresh 保持在重锚位，不再被回退驱动。
  ```cpp
  if (const auto route = GetRouteByMotorId(mid); route.has_value() &&
      route->trajectory_index < has_latest_input_.size()) {
    has_latest_input_[route->trajectory_index] = false;
  }
  ```
- **判据（比修一行更重要）：**任何「状态量的失效/重锚」都要问一句**「还有哪些同源副本没清」**。
  LL-039 清了命名目标，漏了回退表；下次新增任何「末次输入缓存」都要在使能/新意图边界一起清。
- 回归写法：`incident_regression_test.py` T5（部分点名轨迹不得复活使能前历史输入），
  并**验牙齿**——`git show HEAD:<file>` 还原旧代码跑一遍必须失败（实测 L2→0.6）。
- 附带修法：F52 档位日志曾写死「由 motor_map.yaml 配置」→ 换成「由 motor_map_file 配置」，
  避免选 5J 档时日志误导（配置文件名由 launch 参数决定，节点内拿不到文件名）。

## 相关路径

- `src/a3_can_bridge/src/motor_protocol_node.cpp`（使能处理里的重锚 + 回退表清零）
- `scripts/a3_test/incident_regression_test.py`（T5）
- [LL-039](LL-039-teach-exit-reanchor-false-trip-enable-snap.md)（同一根因家族：意图边界的目标失效）
