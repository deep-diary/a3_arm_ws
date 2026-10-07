# F92 — 全仓质量门禁接线（lint 真跑 + 一键 CI 门脚本）


- **说明：** 现状审计：5 个包在 `package.xml` 声明了 `ament_lint_auto`/`ament_lint_common`，但除 `a3_can_bridge` 外 C++ 包 CMakeLists 无 `BUILD_TESTING → ament_lint_auto_find_test_dependencies()` 块、Python 包 `setup.py` 无 `tests_require` 且无 `test/test_*.py`——lint 声明全是死的，`colcon test` 什么都不查。工业基线是「每次构建后一条命令跑全部静态检查与集成验收，非零即不合格」。
  - **门禁 linter 集合（本任务选定的 ament_lint 子集）**：C++ 包 `ament_cmake_lint_cmake` + `ament_cmake_xmllint` + `ament_cmake_cppcheck`；Python 包 `ament_flake8` + `ament_pep257`（经标准 `test/test_copyright` 同款 pytest 入口接线）；外加已有 launch_testing 产品验收（`a3_acceptance_tests` F75/F76/F77）。
  - **明确不纳入本任务（存量专项，见 LL-103）**：`cpplint`/`uncrustify`（`a3_can_bridge` 实测 cpplint 3454 项、uncrustify 5 文件——全量修复是独立格式化专项，产生巨型 blame churn，且多个相关文件正被队友在工作区修改）；`copyright`（全部存量文件缺版权头，机械添加同样需要避开队友在改文件）。门禁必须真的过——把 3500 项失败留在门上等于没有门。版权头/格式化作为后续任务，新文件起执行规范。
  - 声明了 `ament_lint_common` 的 5 个包改为上述精选 `test_depend`（不再由 common 间接拉入 cpplint/uncrustify/copyright），使门禁诚实。
  - 新增 `scripts/a3_test/a3_ci_gate.sh`：`colcon build --symlink-install` → `colcon test`（全仓 lint + launch_testing，顺序执行器）→ 结果 XML 强执行（JUnit errors/failures + ctest `<Test Status="failed">` 双格式），任一失败非零退出；脚本内 source 规则、`PYTHONNOUSERSITE=1`、离线 xmllint catalog 固化。本机 `colcon test-result` 无 `--enforce`，强执行内置于脚本（LL-103）。
  - F76 门禁化暴露的起栈竞态一并修复：高负载下 rmw 丢失 `load_controller` 响应，spawner 重试撞上 "already loaded" FATAL 退出。`a3_bringup.launch.py` 改 JTC→zero_torque→JSB 严格顺序链 + spawner 非零退出自动重启（spawner 启动先查 is_controller_loaded，重启幂等），`--service-call-timeout 5.0`（内部固定 3 次重试，恢复 ≤15 s；30 s 时最长 90 s 超出验收窗口）。
  - 门禁固定 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`：Cyclone 严格按 DDS 规范判 QoS，借此发现真实缺陷——PS4/F77 的 JointJog 发布端是 BEST_EFFORT，而 Humble moveit_servo 订阅实测 RELIABLE（Fast-DDS 宽松放行一直掩盖），已统一改 RELIABLE（LL-104）。
- **改动：**
  1. C++ 包（`a3_msgs`、`a3_description`、`a3_hardware_interface`、`a3_trajectory_processing`、`a3_moveit_config`、`a3_acceptance_tests`、`a3_cloud_edge`）：`package.xml` 加精选 test_depend；CMakeLists 加 `BUILD_TESTING` 块（`a3_cloud_edge` 仅补 test_depend）
  2. Python 包（`a3_arm_controller`、`a3_gripper_controller`、`a3_mqtt_bridge`、`a3_teleop_ps4`、`a3_bringup`）：`package.xml` 加 `test_depend ament_lint_auto/ament_flake8/ament_pep257`；`setup.py` 加 `tests_require`；新增标准 `test/test_flake8.py`、`test/test_pep257.py`
  3. `a3_can_bridge`：`ament_lint_common` 换精选 test_depend（接线已存在）
  4. 修复门禁报出的全部真实问题（cppcheck/flake8/pep257/lint_cmake/xmllint）
  5. 新增 `scripts/a3_test/a3_ci_gate.sh`、`scripts/a3_test/catalog.xml`、`scripts/a3_test/schema/package_format3.xsd`（离线 xmllint）
  6. `a3_bringup.launch.py`：spawner 顺序链 + 非零自动重启 + service-call-timeout 5.0 s（修 F76 起栈竞态）
  - 不动：`a3_lerobot_config`、`lerobot_robot_a3`（待批准删除，不接门禁）、`third_party`
- **验收标准（仿真；脚本可在无 vcan 条件下运行；mock hardware 栈不要求真机）：**
  1. `./scripts/a3_test/a3_ci_gate.sh` 退出码 0；输出中每包 lint 测试 0 failures
  2. `colcon test --packages-select a3_acceptance_tests`：F75/F76/F77 launch_testing 全绿
  3. 故意在任一 Python 文件引入一行 PEP8 违规 → 该包 flake8 FAIL、脚本非零（验收后还原）
- **关联：** F79（colcon test 产品验收，本任务把它串进统一门）；对标工业 CI 静态门；[LL-103](../lessons_learned/LL-103-lint-gate-curated-subset-and-honest-wiring.md)、[LL-104](../lessons_learned/LL-104-cyclone-qos-strictness-servo-reliable-and-setup-set-u.md)
- **状态：** 已完成（2026-09-23，仿真；验收 1：`./scripts/a3_test/a3_ci_gate.sh` 退出码 0——5 个 JUnit 包 10 tests、ctest 23 tests 全 0 failures，含每包 lint；验收 2：F75 15/15、F76 13/13、F77 8/8 全绿；验收 3：注入 W291 → a3_gripper_controller flake8 FAIL、门禁非零，已还原复绿。新增 apt 依赖 `ros-humble-rmw-cyclonedds-cpp`）。真机验收待通电。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
