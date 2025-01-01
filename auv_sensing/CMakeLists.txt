cmake_minimum_required(VERSION 3.8)
project(auv_sensing)

# Enable warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Append custom CMake modules path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")


# Find build dependencies first
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find other dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(onnxruntime REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate service and message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/YoloInference.srv"
  "msg/Detection.msg"
  DEPENDENCIES sensor_msgs
)

# Declare dependencies for your C++ target
set(dependencies
    rclcpp
    mavros
    mavros_msgs
    OpenCV 
    cv_bridge
    sensor_msgs
    onnxruntime
    Eigen3
)

# Add the executable
add_executable(yolo_inference_server src/yolo_inference_server.cpp)

# Link dependencies
ament_target_dependencies(yolo_inference_server ${dependencies})

# Link the generated service and message interfaces
rosidl_target_interfaces(yolo_inference_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install the executable
install(
  TARGETS yolo_inference_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  
)
install (
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)
# Handle testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip specific linters if necessary
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(${dependencies})

# Finalize the package
ament_package()
