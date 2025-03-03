cmake_minimum_required(VERSION 3.8)
project(auv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
    geometry_msgs
    rclcpp
    mavros
    mavros_msgs
    auv_interfaces
    rclcpp_action
    std_srvs
)

# Make sure to include the proper include directory
include_directories(include)

add_executable(dumb_controller src/dumb_controller.cpp src/controller_actions.cpp src/controller_services.cpp)
ament_target_dependencies(dumb_controller ${dependencies})

add_executable(control_cmd_cli src/control_cmd_cli.cpp)
ament_target_dependencies(control_cmd_cli ${dependencies})

target_include_directories(dumb_controller PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_include_directories(control_cmd_cli PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_compile_features(dumb_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(control_cmd_cli PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install targets
install(TARGETS 
  dumb_controller
  control_cmd_cli
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
