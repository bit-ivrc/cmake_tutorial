cmake_minimum_required(VERSION 2.8.3)
project(simple_ros_cmake_example)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=c++11 ${CMAKE_CXX_FLAGS}")

set(catkin_dpes
  nav_msgs
  roscpp
  roslib
  grid_map_ros
  grid_map_cv
  )
find_package(catkin REQUIRED COMPONENTS
  ${catkin_dpes}
  )

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
    LIBRARIES point
    CATKIN_DEPENDS ${catkin_dpes}
    DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
)

add_library(point
  src/point.cpp)

add_executable(test_simple_ros_node
  src/main.cpp)
target_link_libraries(test_simple_ros_node
  point
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES})
