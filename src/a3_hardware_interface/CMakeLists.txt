cmake_minimum_required(VERSION 3.8)
project(a3_hardware_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(a3_can_bridge REQUIRED)
find_package(pinocchio REQUIRED)

add_library(
  a3_hardware_interface
  SHARED
  src/a3_mit_hardware_interface.cpp
  src/gravity_compensation_controller.cpp
  src/socketcan_transport.cpp
)
target_include_directories(
  a3_hardware_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  a3_hardware_interface
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  diagnostic_msgs
  std_msgs
  a3_can_bridge
)
target_link_libraries(a3_hardware_interface pinocchio::pinocchio)

pluginlib_export_plugin_description_file(
  hardware_interface a3_hardware_interface.xml)
pluginlib_export_plugin_description_file(
  controller_interface a3_controller_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS a3_hardware_interface
  EXPORT export_a3_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_a3_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  diagnostic_msgs
  std_msgs
  a3_can_bridge
  pinocchio
)

ament_package()
