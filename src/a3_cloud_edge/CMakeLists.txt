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
find_package(ament_index_cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(geometry_msgs REQUIRED)

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

add_executable(gravity_compensation_node src/gravity_compensation_node.cpp)
target_compile_features(gravity_compensation_node PUBLIC cxx_std_17)
ament_target_dependencies(gravity_compensation_node
  rclcpp trajectory_msgs ament_index_cpp)
target_link_libraries(gravity_compensation_node pinocchio::pinocchio)

install(TARGETS gravity_compensation_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/trajectory_test_publisher.py
  scripts/moveit_plan_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/run_micro_ros_agent.sh
  scripts/setup_microros_host.sh
  scripts/setup_moveit_debs.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
