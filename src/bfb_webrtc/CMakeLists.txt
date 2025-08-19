cmake_minimum_required(VERSION 3.8)
project(bfb_webrtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(X11 REQUIRED)

# find gstreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED 
  gstreamer-1.0 
  gstreamer-webrtc-1.0 
  gstreamer-sdp-1.0 
  gstreamer-video-1.0
)


# Add your executable
add_executable(signaling_node src/signaling.cpp)
add_executable(webrtc_send_node src/send.cpp)

# Link ROS 2 and other dependencies
ament_target_dependencies(signaling_node rclcpp)
ament_target_dependencies(webrtc_send_node rclcpp)

target_link_libraries(signaling_node jsoncpp pthread)
target_link_libraries(webrtc_send_node jsoncpp pthread ${GSTREAMER_LIBRARIES})
target_link_libraries(webrtc_send_node
  jsoncpp
  pthread
  ${GSTREAMER_LIBRARIES}
  ${X11_LIBRARIES}
)

target_include_directories(webrtc_send_node PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_include_directories(webrtc_send_node PRIVATE ${GSTREAMER_INCLUDE_DIRS})

# Install the executable
install(TARGETS signaling_node webrtc_send_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
