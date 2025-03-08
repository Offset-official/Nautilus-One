cmake_minimum_required(VERSION 3.8)
project(auv_sensing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

set(dependencies
    rclcpp
    OpenCV 
    cv_bridge
    auv_interfaces
    image_transport
)

add_executable(show_camera src/show_camera.cpp)
add_executable(infer_camera_gate src/infer_camera_gate.cpp)
add_executable(show_camera_uncomp src/show_camera_uncomp.cpp)

ament_target_dependencies(show_camera ${dependencies})
ament_target_dependencies(infer_camera_gate ${dependencies})
ament_target_dependencies(show_camera_uncomp ${dependencies})

# Link additional libraries for infer_camera_gate that uses libcurl and nlohmann_json
target_link_libraries(infer_camera_gate CURL::libcurl nlohmann_json::nlohmann_json)

install(TARGETS
    show_camera
    infer_camera_gate    
    show_camera_uncomp
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
