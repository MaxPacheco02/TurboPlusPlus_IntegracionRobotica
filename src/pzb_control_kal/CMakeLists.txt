cmake_minimum_required(VERSION 3.8)
project(pzb_control_kal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pzb_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(aru_interfaces REQUIRED)

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME})

set(DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
  std_srvs
  rclcpp
  sensor_msgs
  tf2
  nav_msgs
  tf2_ros
  Eigen3
  tf2_geometry_msgs
  pzb_msgs
  aru_interfaces
)

add_executable(state_estimate src/state_estimate.cpp)
ament_target_dependencies(state_estimate ${DEPENDENCIES})

add_executable(guidance_node src/guidance_node.cpp)
ament_target_dependencies(guidance_node ${DEPENDENCIES})

# add_executable(master_node src/master_node.cpp)
# ament_target_dependencies(master_node ${DEPENDENCIES})

add_executable(line_follower_path_publisher_node src/line_follower_path_publisher_node.cpp)
ament_target_dependencies(line_follower_path_publisher_node ${DEPENDENCIES})

# Install Python executables
install(PROGRAMS
scripts/path_publisher_node.py
scripts/pzb_teleop_node.py
scripts/pzb_teleop_low_level_node.py
DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  state_estimate
  guidance_node
  # master_node
  line_follower_path_publisher_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
