cmake_minimum_required(VERSION 3.8)
project(service)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME})

# CPP
# INCLUDE CPP 'include' DIR
include_directories(include)
# CREATE CPP EXECS
add_executable(cpp_exe src/cpp_node.cpp)
ament_target_dependencies(cpp_exe rclcpp)
# INSTALL CPP EXECS
install(TARGETS
  cpp_exe
  DESTINATION lib/${PROJECT_NAME}
)

# PYTHON
# INSTALL MODULES
ament_python_install_package(${PROJECT_NAME})
# INSTALL EXECS
install(PROGRAMS
  scripts/ObjPos_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# INSTALAR DIRECTORIO DE PNG's (HACERLAS ACCESIBLES EN RUNTIME)
install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

# # INSTALAR DIRECTORIO DE CONFIG 
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )



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
