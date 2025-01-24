cmake_minimum_required(VERSION 3.8)
project(auv_interfaces)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/Manipulator.msg"
  "srv/YoloInference.srv"
  DEPENDENCIES sensor_msgs std_msgs
)

# Ensure that message and service files are found during runtime
ament_export_dependencies(rosidl_default_runtime)

ament_package()
