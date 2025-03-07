cmake_minimum_required(VERSION 3.8)
project(auv_sensing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(image_transport REQUIRED)


set(dependencies
    rclcpp
    OpenCV 
    cv_bridge
    auv_interfaces
    image_transport
)

add_executable(show_camera src/show_camera.cpp)
add_executable(infer_camera src/infer_camera.cpp)
add_executable(show_camera_url src/show_camera_url.cpp)
add_executable(show_camera_compressed src/show_camera_compressed.cpp)
add_executable(show_camera_sdp src/show_camera_sdp.cpp)
add_executable(republish_sdp src/republish_sdp.cpp)
add_executable(republish_sdp_camera_info src/republish_sdp_camera_info.cpp)
add_executable(publish_webcam src/publish_webcam.cpp)
add_executable(show_camera_uncomp src/show_camera_uncomp.cpp)





ament_target_dependencies(show_camera ${dependencies})
ament_target_dependencies(infer_camera ${dependencies})
ament_target_dependencies(show_camera_url ${dependencies})
ament_target_dependencies(show_camera_compressed ${dependencies})
ament_target_dependencies(show_camera_sdp ${dependencies})
ament_target_dependencies(republish_sdp ${dependencies})
ament_target_dependencies(republish_sdp_camera_info ${dependencies})
ament_target_dependencies(publish_webcam ${dependencies})
ament_target_dependencies(show_camera_uncomp ${dependencies})




install(TARGETS
    show_camera
    infer_camera
    show_camera_url
    show_camera_compressed
    show_camera_sdp
    show_camera_uncomp
    republish_sdp
    publish_webcam
    republish_sdp_camera_info
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
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
