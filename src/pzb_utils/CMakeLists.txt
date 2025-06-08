# UBICACIÓN: ~/pzb_ws/src/pzb_utils/CMakeLists.txt
# REEMPLAZAR completamente el archivo generado por ros2 pkg create

cmake_minimum_required(VERSION 3.8)
project(pzb_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pzb_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(fpga_node src/fpga_node.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(fpga_node
  rclcpp
  pzb_msgs
  std_msgs
)

# Install targets
install(TARGETS
  fpga_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()