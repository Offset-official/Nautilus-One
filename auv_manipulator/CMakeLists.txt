cmake_minimum_required(VERSION 3.8)
project(auv_manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(auv_interfaces REQUIRED)

# Include directories
include_directories(include)

# Add the thruster controller node
add_executable(dropper src/dropper.cpp)
ament_target_dependencies(dropper
  rclcpp
  std_msgs
  std_srvs
  mavros_msgs
  auv_interfaces
)

# Install the node
install(TARGETS
  dropper
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_package()