project(RoboArmGazeboPlugin)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if (PRINT_VERBOSE)
  add_definitions(-DPRINT_VERBOSE)
endif()

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(roboarm_plugin SHARED roboarm_plugin.cpp)
target_link_libraries(roboarm_plugin ${GAZEBO_LIBRARIES})
target_include_directories(roboarm_plugin PUBLIC ${PROJECT_SOURCE_DIR})

# Build the stand-alone test program
add_executable(pos pos.cpp)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(pos ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(pos ${GAZEBO_LIBRARIES})
endif()