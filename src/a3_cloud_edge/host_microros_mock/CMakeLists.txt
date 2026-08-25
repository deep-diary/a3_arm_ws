cmake_minimum_required(VERSION 3.8)
project(a3_microros_mock)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(micro_ros_utilities REQUIRED)
find_package(rmw_microxrcedds REQUIRED)

add_executable(microros_mock_client microros_mock_client.c)

ament_target_dependencies(microros_mock_client
  rcl rclc trajectory_msgs sensor_msgs micro_ros_utilities rmw_microxrcedds)

install(TARGETS microros_mock_client
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
