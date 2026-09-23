cmake_minimum_required(VERSION 3.8)
project(a3_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

