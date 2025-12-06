cmake_minimum_required(VERSION 3.8)
project(r2d2_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Create symlink from bin to lib for ROS 2 launch compatibility
# This allows ros2 launch to find executables in lib/${PROJECT_NAME}/
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib_target")

install(
  DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/../../install/r2d2_perception/bin/"
  DESTINATION
    "lib/${PROJECT_NAME}"
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
