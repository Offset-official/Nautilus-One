cmake_minimum_required(VERSION 3.8)
project(auv_diagonostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)

set(dependencies
    rclcpp
    mavros
    mavros_msgs
)

add_executable(diagonostics src/diagonostics_node.cpp)
ament_target_dependencies(diagonostics ${dependencies})

target_include_directories(diagonostics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(diagonostics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  diagonostics
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
