cmake_minimum_required(VERSION 2.8)
project(find_package_demo)

set(CMAKE_CXX_STANDARD 11)

message("\n********Find package in Module Mode!")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
message("CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")
message("CMAKE_LIBRARY_ARCHITECTURE = ${CMAKE_LIBRARY_ARCHITECTURE}")
find_package(Glog REQUIRED)

message("\n\n********Find package in Config Mode!")
#set(ENV{CMAKE_PREFIX_PATH} "")
#set(ENV{PATH} "")
find_package(OpenCV REQUIRED)
message("opencv config file path: " ${OpenCV_CONFIG})
message("1. cmake cache var 'CMAKE_PREFIX_PATH' = ${CMAKE_PREFIX_PATH}" )
message("2. env var 'CMAKE_PREFIX_PATH' = $ENV{CMAKE_PREFIX_PATH}" )
message("3. env var 'PATH' = $ENV{PATH}" )
message("4. cache var 'CMAKE_SYSTEM_PREFIX_PATH' = ${CMAKE_SYSTEM_PREFIX_PATH}" )


add_executable(find_package_demo
  main.cpp)
target_link_libraries(find_package_demo
  ${Glog_LIBRARIES})