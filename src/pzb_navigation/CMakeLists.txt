cmake_minimum_required(VERSION 3.8)
project(pzb_navigation LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pzb_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(DEPENDENCIES
  ament_cmake
  urdf
  robot_state_publisher
  xacro
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
  OpenCV
  cv_bridge
)

add_executable(reactive_navigation_node src/reactive_navigation_node.cpp)
ament_target_dependencies(reactive_navigation_node ${DEPENDENCIES})
target_compile_features(reactive_navigation_node PUBLIC cxx_std_17)

# Install targets
install(TARGETS
  reactive_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()