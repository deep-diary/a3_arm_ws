cmake_minimum_required(VERSION 3.8)
project(a3_self_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(self_test REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_executable(a3_self_test src/a3_self_test_node.cpp)
ament_target_dependencies(
  a3_self_test
  rclcpp
  self_test
  diagnostic_msgs
  diagnostic_updater
  sensor_msgs
  controller_manager_msgs
)

install(TARGETS a3_self_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
