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

install(TARGETS retime_trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
