cmake_minimum_required(VERSION 3.8)
project(a3_can_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

set(srv_files
  "srv/MotorCommand.srv"
  "srv/SetCanId.srv"
  "srv/SetMotorParam.srv"
  "srv/MotorScan.srv"
  "srv/MotorMitCommand.srv"
  "srv/MotorStop.srv"
  "srv/MotorSetMode.srv"
  "srv/MotorScanCollect.srv"
)

set(msg_files
  "msg/MotorState.msg"
  "msg/MotorStates.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES std_msgs
)

add_library(socketcan_driver
  src/socketcan_driver.cpp
)
ament_target_dependencies(socketcan_driver
  rclcpp
  std_msgs
)

add_executable(can_transport_node
  src/can_transport_node.cpp
)
target_link_libraries(can_transport_node socketcan_driver)
ament_target_dependencies(can_transport_node
  rclcpp
  std_msgs
)

add_executable(motor_protocol_node
  src/motor_protocol_node.cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motor_protocol_node "${cpp_typesupport_target}")
ament_target_dependencies(motor_protocol_node
  rclcpp
  diagnostic_msgs
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)

add_executable(power_sequence_node
  src/power_sequence_node.cpp
)
ament_target_dependencies(power_sequence_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  socketcan_driver
  can_transport_node
  motor_protocol_node
  power_sequence_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/el05_motor_cansend.sh
  scripts/a3_motor_cansend.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
