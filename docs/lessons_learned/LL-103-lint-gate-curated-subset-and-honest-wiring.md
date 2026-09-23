# LL-103 — lint 门禁要「精选子集 + 诚实接线」；批量 codemod 必须保 AST 与闭合引号

> **日期：** 2026-09-23  
> **产品线：** Edge  
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，cppcheck 2.7

## 现象

F92 前审计发现：5 个包在 `package.xml` 声明了 `ament_lint_auto`/`ament_lint_common`，但除 `a3_can_bridge` 外 C++ 包 CMakeLists 没有 `BUILD_TESTING → ament_lint_auto_find_test_dependencies()` 块，Python 包没有 `tests_require`、没有 `test/test_*.py`——lint 声明全是死的，`colcon test` 什么都不查。接线让 lint 真跑后又连续踩了：linter 集合选择、colcon 参数名、远程 schema 卡顿、cppcheck 版本门，以及两轮批量 codemod 把文件改坏（第二轮丢掉单行 docstring 的闭合 `"""`，造成 22 个文件 44 处 `SyntaxError`，下游解析器报出的非法字符/缩进错误全部是同一根因的恢复噪声）。

## 根因

1. **死声明**：`test_depend` 只是「装了工具」，没有测试入口就不会执行。Python 包必须在 `setup.py` 放**字面量** `tests_require=['pytest']`（colcon 匹配的是字符串 `'pytest'`，拼出来的不算）+ 标准 `test/test_flake8.py`；C++ 包必须有 ament_lint_auto 块；package.xml 还要有 `<test_depend>python3-pytest</test_depend>`。
2. **门必须真的过**：`ament_lint_common` 一把拉入 cpplint（a3_can_bridge 实测 3454 项）/uncrustify/copyright，全量修复是独立格式化专项且会产生巨型 blame churn（多文件正被队友修改）。把几千项失败留在门上等于没有门。
3. **xmllint 慢/挂**：package.xml 第 2 行的 `<?xml-model href="http://download.ros.org/...xsd"?>` PI 使 ament_xmllint 对每个文件调 `xmllint --schema <URL>`，libxml2 现场走 HTTP 拉 schema，负载高时整轮卡死。
4. **cppcheck 2.7 被 ament 硬跳过**（21 个测试 skip），只有 `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1` 才真跑。
5. **codemod 事故**：手写 AST 行列号去替换源码（`lines[0][:len(...) - ...]`），闭合引号恢复还加了 `len(lines) > 1` 守卫，导致单行 docstring 的 `"""` 被吞。行列算术不可靠；AST 变换后必须回读验证。

## 正确做法 / 规避

1. 门禁用**精选子集**：C++ = lint_cmake + xmllint + cppcheck；Python = flake8（行长 99）+ pep257（ament 约定）。cpplint/uncrustify/copyright 作为后续独立专项，**新文件**起遵守规范。
2. 离线 schema：vendor `scripts/a3_test/schema/package_format3.xsd` + `scripts/a3_test/catalog.xml`（OASIS XML Catalog 把 URL 映射到本地），门禁 `export XML_CATALOG_FILES=<绝对路径 catalog>`。xmllint 全仓从「卡死」降到 0.34 s。
3. 门禁固化环境：`AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1`、`PYTHONNOUSERSITE=1`、顺序执行器（`--executor sequential`，schema 结果稳定）。
4. `colcon test` 永远退出码 0，必须自己强执行：结果写**全新目录**（`--test-result-base`，杜绝旧 JUnit 残留假通过），再遍历 `*.xml` 累加 errors/failures，非零即失败。注意 `colcon test` 与 `colcon test-result` 的参数名都是 `--test-result-base`（没有 `--test-result-base-dir`，也没有 `--enforce`，拼错只在参数解析期静默失败）。
5. F76 经验：move_group 当前状态有滞后，断言前用 `PlanningSceneComponents.ROBOT_STATE` 轮询 `/get_planning_scene`（常量是 ROBOT_STATE=2，别误用 SCENE_STATE）。
6. **批量 codemod 铁律**：
   - 永远不要手算 AST 源码行列跨度做替换；用 `ast.get_source_segment` 取段、只做物理行级变换。
   - 变换前后都跑 `ast.parse`；docstring 等节点序列用 HEAD 做真值比对（归一化时中英标点都要剥掉：`。，,.!?！？`，否则验证器误报）。
   - 单行 docstring 的闭合 `"""` 必须原样保留——替换行首时把行尾也当成可截断部分是最常见的吞引号来源。
7. 门禁化让 F76 的起栈竞态现形：rmw_fastrtps 在高负载下丢 service 响应（日志出自 `librmw_fastrtps_shared_cpp.so` 的 `rmw_response.cpp`："failed to send response ... client will not receive response"），spawner 内部重试撞上服务端 "already loaded" 直接 FATAL。修法是 launch 级标准手段：spawner 用 `OnProcessExit` 严格串链消除并发突发 + 非零退出自动重启（spawner 先跑 `is_controller_loaded`，重启天然幂等）+ `--service-call-timeout 5.0`（spawner 内部固定重试 3 次，30 s 时单次恢复最长 90 s，远超验收窗口）。不要把多个 spawner 放在同一启动窗口并行跑。
   - 事件处理器里退出码属性是 **`event.returncode`**（`ProcessExited` 没有 `exit_code`）；在回调里抛 AttributeError 会被 launch 主循环捕获并对全栈发起 SIGINT——表象是「起栈 4~5 秒后所有节点被整齐 SIGINT、harness 退出码 -2」，真正的一行 `[ERROR] [launch]: Caught exception ...` 藏在进程输出中段，必须往信号序列之前翻。

## 相关路径

- `scripts/a3_test/a3_ci_gate.sh`（一键门禁）
- `scripts/a3_test/catalog.xml`、`scripts/a3_test/schema/package_format3.xsd`（离线 catalog）
- 各 Python 包 `test/test_flake8.py`、`test/test_pep257.py` 与 `setup.py` 的 `tests_require`
- 各 C++ 包 `CMakeLists.txt` 的 `BUILD_TESTING` ament_lint_auto 块
- `/tmp/repair_oneliner_docstrings.py`（以 HEAD 为真值的确定性修复工具）
