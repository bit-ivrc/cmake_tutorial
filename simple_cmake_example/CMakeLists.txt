cmake_minimum_required(VERSION 2.8)
project(simple_cmake_example)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=c++11 ${CMAKE_CXX_FLAGS}")

# find opencv library
find_package(OpenCV REQUIRED)
# find boost::filesystem library
find_package(Boost REQUIRED COMPONENTS filesystem)

# print the value of cmake variables in terminal:
message("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")
message("Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")
message("Boost_LIBRARIES = ${Boost_LIBRARIES}")

# set the path of library headers
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# generate a static library named point
add_library(point
  src/point.cpp)

# generate a executable program named simple_cmake_example
add_executable(simple_cmake_example
  src/main.cpp)
# add the link to necessary libraries
target_link_libraries(simple_cmake_example
  point
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES})