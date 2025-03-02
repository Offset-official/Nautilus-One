cmake_minimum_required(VERSION 3.8)
project(auv_hud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosbridge_server REQUIRED)

# Add executable
add_executable(bag_recorder src/bag_recorder.cpp)
target_include_directories(bag_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link libraries
target_link_libraries(bag_recorder
  yaml-cpp)

ament_target_dependencies(bag_recorder
  rclcpp
  std_srvs
  rosbag2_cpp
  rosbag2_storage
  rcutils
  ament_index_cpp)

# Install targets
install(TARGETS bag_recorder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
