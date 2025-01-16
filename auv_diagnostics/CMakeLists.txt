cmake_minimum_required(VERSION 3.8)
project(auv_diagnostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
# find_package(nlohmann_json REQUIRED)

set(dependencies
    rclcpp
    mavros
    mavros_msgs
    # nlohmann_json
)

add_executable(diagnostics src/diagnostics_node.cpp)
ament_target_dependencies(diagnostics ${dependencies})

target_include_directories(diagnostics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(diagnostics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  diagnostics
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
