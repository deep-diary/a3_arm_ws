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

# F180: servo 绝对目标（外部积分 + Ruckig 在线 OTG）
add_executable(servo_target_node src/servo_target_node.cpp)
ament_target_dependencies(
  servo_target_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
)
target_link_libraries(servo_target_node ruckig::ruckig)

install(TARGETS retime_trajectory_node otg_demo servo_target_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
