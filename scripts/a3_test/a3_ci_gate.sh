#!/usr/bin/env bash
# F92 A3 工业级质量门禁（一键 CI gate）。
# 流程: 固定环境 → colcon build --symlink-install → colcon test（顺序执行）
#       → 结果 XML 强执行（JUnit errors/failures + ctest Status=failed，非零即失败）。
# 覆盖: lint（lint_cmake / xmllint / cppcheck / flake8 / pep257）
#       + F75/F76/F77 launch 验收 + 全部包单元测试。
# 边界: src/third_party、src/lerobot_robot_a3 是外部代码，不进门禁。
# 用法: ./scripts/a3_test/a3_ci_gate.sh
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$DIR/../.." && pwd)"

# ---- 固定环境（不依赖调用者已 source 的状态）----
# ROS setup 脚本引用未绑定变量（AMENT_TRACE_SETUP_FILES），source 期间放开 -u。
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/local_setup.bash"
set -u
export PYTHONNOUSERSITE=1
export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
export XML_CATALOG_FILES="$DIR/catalog.xml"
# 固定 RMW：Fast-DDS 在起栈突发下丢 service 响应（F76 曾失败），且对
# best-effort 发布→reliable 订阅宽松放行，会掩盖 QoS 真问题；Cyclone 行为
# 确定、ARM 上 CPU 占用更低（LL-104）。依赖：ros-humble-rmw-cyclonedds-cpp。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RESULT_BASE="$WS/log/ci_results"

echo "==> [1/3] colcon build --symlink-install"
cd "$WS"
colcon build --symlink-install

# 全新结果目录，杜绝旧 JUnit 残留造成的假通过/假失败。
rm -rf "$RESULT_BASE"
mkdir -p "$RESULT_BASE"

echo "==> [2/3] colcon test (sequential)"
colcon test --executor sequential --test-result-base "$RESULT_BASE"

echo "==> [3/3] enforce JUnit results"
python3 - "$RESULT_BASE" <<'PY'
import pathlib
import sys
import xml.etree.ElementTree as ET

base = pathlib.Path(sys.argv[1])

# 两种结果格式：
#  1) JUnit pytest.xml：<testsuite tests= errors= failures= skipped=>
#  2) ctest Test.xml：<Site><Testing><Test Status="passed|failed|...">
#     JUnit 属性解析看不到 ctest 的 <Test Status="failed">，必须单独处理。
files = 0
total = errors = failures = skipped = 0
ctest_total = ctest_failed = 0
bad = []

for path in base.rglob("*.xml"):
    root = ET.parse(path).getroot()

    if root.tag == "Site":
        for test in root.iter("Test"):
            # <TestStatus> 块里也有无属性的 <Test>（仅名字文本），跳过。
            status = test.get("Status")
            if status is None:
                continue
            ctest_total += 1
            if status not in ("passed", "notrun"):
                ctest_failed += 1
                name = test.findtext("Name", default=path.name)
                bad.append(f"ctest failed: {name} ({path})")
        continue

    suites = root.iter("testsuite") if root.tag == "testsuites" else [root]
    for suite in suites:
        files += 1
        total += int(suite.get("tests", 0))
        errors += int(suite.get("errors", 0))
        failures += int(suite.get("failures", 0))
        skipped += int(suite.get("skipped", 0))

print(
    f"Summary: {files} JUnit files, {total} tests, "
    f"{errors} errors, {failures} failures, {skipped} skipped; "
    f"ctest {ctest_total} tests, {ctest_failed} failed"
)
for line in bad:
    print(line)
if errors or failures or ctest_failed:
    sys.exit(1)
PY

echo "==> CI GATE PASSED"
