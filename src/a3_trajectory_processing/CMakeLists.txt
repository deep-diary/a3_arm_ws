cmake_minimum_required(VERSION 3.8)
project(a3_trajectory_processing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(a3_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(ruckig CONFIG REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(retime_trajectory_node src/retime_trajectory_node.cpp)
ament_target_dependencies(
  retime_trajectory_node
  rclcpp
  a3_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
)
target_link_libraries(retime_trajectory_node ruckig::ruckig)

# F167.3: 实时 OTG 演示（纯计算，复用 ruckig；仿真-only，不接生产链路）
add_executable(otg_demo src/otg_demo.cpp)
target_link_libraries(otg_demo ruckig::ruckig)

# F180/F181: servo 绝对目标（外部积分 + CLIK + 有状态速度积分 + twist 平滑）
add_executable(servo_target_node src/servo_target_node.cpp)
ament_target_dependencies(
  servo_target_node
  rclcpp
  a3_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
)

install(TARGETS retime_trajectory_node otg_demo servo_target_node
  DESTINATION lib/${PROJECT_NAME}
)

# 负载优化：/joint_states 200 Hz → /joint_states_lite 50 Hz 中转
add_executable(joint_state_relay src/joint_state_relay_node.cpp)
ament_target_dependencies(
  joint_state_relay
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

install(TARGETS joint_state_relay
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
