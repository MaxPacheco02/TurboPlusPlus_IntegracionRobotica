cmake_minimum_required(VERSION 3.8)
project(pzb_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(cvplot_DIR "src/cvplot/build/")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
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
# find_package(cvplot REQUIRED)

set(DEPENDENCIES
  rclpy  
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

# message("cvplot_FOUND: ${cvplot_FOUND}")
# include_directories("${PROJECT_SOURCE_DIR}/cvplot/include/cvplot/")

add_executable(mcl_node src/mcl_node.cpp)
ament_target_dependencies(mcl_node ${DEPENDENCIES})

foreach(dependency ${DEPENDENCIES})
  list(APPEND DEPENDENCY_INCLUDE_DIRS ${${dependency}_INCLUDE_DIRS})
endforeach()

# Set include directories for all code
include_directories(
  ${PROJECT_SOURCE_DIR}/src/include
  ${DEPENDENCY_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /opt/ros/humble/include  # Adjust this if using a different ROS distro
)

# Compile all sources directly into the executable
add_executable(mcl_node_test
  src/mcl_node_test.cpp
  src/include/robot.cpp
  src/include/map_utils.cpp
  # src/include/robot_utils.cpp
  src/include/utility.cpp
  # src/include/MCL.cpp
  src/include/MCL_test.cpp

)

ament_target_dependencies(mcl_node_test ${DEPENDENCIES})

# Install Python executables
install(PROGRAMS
  scripts/display_lcd_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mcl_node
  mcl_node_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)
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
