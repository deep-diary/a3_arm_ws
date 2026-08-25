cmake_minimum_required(VERSION 3.8)
project(a3_cloud_edge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

option(BUILD_MICROROS_MOCK "Build microros_mock_client in this package (use a3_microros_mock from host overlay instead)" OFF)

if(BUILD_MICROROS_MOCK)
  add_executable(microros_mock_client src/microros_mock_client.cpp)
  target_include_directories(microros_mock_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(microros_mock_client
    rclcpp trajectory_msgs sensor_msgs builtin_interfaces)
  install(TARGETS microros_mock_client
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(PROGRAMS
  scripts/trajectory_test_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/run_micro_ros_agent.sh
  scripts/setup_microros_host.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
