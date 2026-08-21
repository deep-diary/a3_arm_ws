cmake_minimum_required(VERSION 3.8)
project(a3_lerobot_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config docs
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)
ament_package()
