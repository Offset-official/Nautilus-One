cmake_minimum_required(VERSION 3.8)
project(auv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

set(dependencies
    geometry_msgs
    rclcpp
    mavros
    mavros_msgs
)

add_executable(cmd_vel_publisher src/cmd_vel_publisher.cpp)
ament_target_dependencies(cmd_vel_publisher ${dependencies})

add_executable(base_controller src/base_controller.cpp)
ament_target_dependencies(base_controller ${dependencies})

target_include_directories(cmd_vel_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(base_controller PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)


target_compile_features(cmd_vel_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(base_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  cmd_vel_publisher
  base_controller
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
  # Install Python executables
install(PROGRAMS
  scripts/velocity_plotter.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
