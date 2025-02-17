cmake_minimum_required(VERSION 3.10)
project(WebRTCExample)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)

# Find Boost
find_package(Boost REQUIRED)

# Add nlohmann JSON header
include_directories("/usr/include/nlohmann")

# Add executable (just use send.cpp here)
add_executable(webrtc_send src/send.cpp)

# Link libraries
target_link_libraries(webrtc_send ${GST_LIBRARIES} ${GST_WEBRTC_LIBRARIES} Boost::Boost pthread)
