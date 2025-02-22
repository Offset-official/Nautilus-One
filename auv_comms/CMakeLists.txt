cmake_minimum_required(VERSION 3.8)
project(auv_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(auv_interfaces REQUIRED)
set(dependencies
    rclcpp
    OpenCV 
    cv_bridge
    image_transport
    auv_interfaces
)

add_executable(color_detector src/color_detector.cpp)
add_executable(color_detector_debug src/color_detector_debug.cpp)

ament_target_dependencies(color_detector ${dependencies})
ament_target_dependencies(color_detector_debug ${dependencies})

install(TARGETS
    color_detector
    color_detector_debug
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
