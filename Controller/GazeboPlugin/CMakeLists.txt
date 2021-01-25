cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(roboarm_plugin SHARED roboarm_plugin.cpp)
target_link_libraries(roboarm_plugin ${GAZEBO_LIBRARIES})
target_include_directories(roboarm_plugin PUBLIC ${PROJECT_SOURCE_DIR})