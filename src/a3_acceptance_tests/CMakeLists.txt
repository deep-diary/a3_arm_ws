cmake_minimum_required(VERSION 3.8)
project(a3_acceptance_tests)

find_package(ament_cmake REQUIRED)

# 验收脚本保持单一事实源（scripts/a3_test），安装进本包 share 供 launch test 执行
install(FILES
  ${PROJECT_SOURCE_DIR}/../../scripts/a3_test/f75_full_mock_acceptance.py
  ${PROJECT_SOURCE_DIR}/../../scripts/a3_test/f76_pilz_acceptance.py
  ${PROJECT_SOURCE_DIR}/../../scripts/a3_test/f77_joint_jog_acceptance.py
  DESTINATION share/${PROJECT_NAME}/harness
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_f75_full_mock.py TIMEOUT 300)
  add_launch_test(test/test_f76_pilz.py TIMEOUT 300)
  add_launch_test(test/test_f77_joint_jog.py TIMEOUT 300)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
