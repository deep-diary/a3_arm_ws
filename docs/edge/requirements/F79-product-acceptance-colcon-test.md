# F79 — 产品验收接入 colcon test / launch_testing（标准 CI 测试门）


- **说明：** 审计任务 #10 发现：F75/F76/F77 验收脚本只能人工「先起栈、再开一个终端跑脚本、肉眼看 PASS」，无标准退出码收集、无 JUnit XML、不能进 CI/门禁——工业工程要求验收可重复、自动化、可归档。F79 新增独立测试包 `a3_acceptance_tests`（ament_cmake），用 ROS 官方 `launch_testing` 框架把每个验收封装为一个 launch test：测试描述内拉起统一产品入口（`a3_bringup.launch.py hardware:=mock`，固定 ROS_DOMAIN_ID、PYTHONNOUSERSITE=1）并执行既有验收脚本（单一事实源，脚本经 CMake install 进包内 share/harness，测试不复制检查逻辑）；断言脚本进程退出码 0；测试结束 launch_testing 自动 SIGINT 拆栈。结果经 `add_launch_test` 自动产出 JUnit XML（`colcon test-result` 可读），与 ament 生态标准测试完全同构。F72 vcan 验收因需 sudo 建 vcan，保留人工/脚本入口，不纳入本测试包。
- **接线：** 新包 `src/a3_acceptance_tests/`（package.xml + CMakeLists.txt + `test/test_f75_full_mock.py`、`test/test_f76_pilz.py`、`test/test_f77_joint_jog.py`）。
- **验收标准：**
  1. `colcon build --packages-select a3_acceptance_tests` 后 `colcon test --packages-select a3_acceptance_tests` 三个测试全部通过，退出码 0
  2. `colcon test-result --all` 可见三个 launch test 的 JUnit 结果；任一验收项失败时测试失败（失败可注入验证）
  3. 测试结束无残留 ROS 进程；F75 测试栈配置与人工验收一致（含 MQTT，broker 不可达时该测试按失败计），F76/F77 栈 `use_mqtt:=false`
- **关联：** F75–F78（被封装的验收与统一入口）；审计任务 #10
- **状态：** 已完成（2026-09-22，仿真验收）。证据：`colcon test --packages-select a3_acceptance_tests` 三个 launch test 全绿（合计 ~85 s），`colcon test-result` Summary 6 tests / 0 errors / 0 failures（3 个 ctest 包装 + 3 个 launch 内用例）；F75 15/15、F76 12/12、F77 8/8 均经 colcon 门通过。失败传播已实证：首跑 F77 因使能早于 controller spawner（6/8 FAIL，退出码 1），JUnit 记录 failure、colcon test 非零退出；修复方式为 F77 脚本内等待 `joint_state_broadcaster` active（spawn 完成标志）再 enable（见 LL-081）。JUnit XML 位于 `build/a3_acceptance_tests/test_results/a3_acceptance_tests/test_test_f7*.xunit.xml`。测试结束 domain 61/62/63 无残留进程。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
