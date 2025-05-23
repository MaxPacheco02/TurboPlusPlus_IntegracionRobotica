cmake_minimum_required(VERSION 3.8)
project(aru_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aru_interfaces REQUIRED)

set(DEPENDENCIES
  sensor_msgs 
  geometry_msgs
  std_msgs
  rclcpp
  tf2
  nav_msgs
  tf2_ros
  Eigen3
  tf2_geometry_msgs
  aru_interfaces
)

install(DIRECTORY 
  launch/
  DESTINATION share/${PROJECT_NAME}
)

# CPP
# INCLUDE CPP 'include' DIR
include_directories(include)

# CREATE CPP EXECS
add_executable(kalman_exe src/kalman_localization.cpp)
ament_target_dependencies(kalman_exe ${DEPENDENCIES})

# INSTALL CPP EXECS
install(TARGETS kalman_exe DESTINATION lib/${PROJECT_NAME})


#---------------------------------------------------------------------------

# PYTHON
# INSTALL MODULES
ament_python_install_package(${PROJECT_NAME})

# INSTALL EXECS
install(PROGRAMS
  scripts/ARU_node.py 
  scripts/ARU_WAY_node.py
  scripts/localization_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# PAQUETES
install(DIRECTORY 
  urdf/ 
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY 
  resources/
  DESTINATION share/${PROJECT_NAME}/resources
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
