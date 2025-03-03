cmake_minimum_required(VERSION 3.8)
project(auv_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(auv_interfaces REQUIRED)
set(dependencies
    rclcpp
    mavros
    mavros_msgs
    apriltag_msgs
    geometry_msgs
    auv_interfaces
)

add_executable(forward src/forward.cpp)
add_executable(april_follow src/april_follow.cpp)

ament_target_dependencies(forward ${dependencies})
ament_target_dependencies(april_follow ${dependencies})

install(TARGETS
    forward
    april_follow
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
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

ament_export_dependencies(${dependencies})
ament_package()
