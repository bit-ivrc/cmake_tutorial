cmake_minimum_required(VERSION 3.3)
project(install_package_demo)

set(CMAKE_CXX_STANDARD 11)

include_directories(
  include
)

aux_source_directory(src LIB_SOURCE_FILES)
file(GLOB_RECURSE LIB_HEADER_FILES include/*.h)

add_library(tinyspline
  ${LIB_SOURCE_FILES})

add_executable(${PROJECT_NAME}
  main.cpp)
target_link_libraries(${PROJECT_NAME}
  tinyspline)

# change install prefix
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install")
message("CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})
# install targets
install(TARGETS tinyspline ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
# install header files
install(FILES ${LIB_HEADER_FILES}
  DESTINATION include)

set(LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
set(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include")
set(CONFIG_INSTALL_DIR "${LIB_INSTALL_DIR}/cmake")

# generate config file and version file using CMakePackageConfigHelpers
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  tinysplineConfigVersion.cmake
  VERSION 0.0.1
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
  cmake/tinysplineConfig.cmake.in
  tinysplineConfig.cmake
  INSTALL_DESTINATION ${CONFIG_INSTALL_DIR}
  PATH_VARS LIB_INSTALL_DIR INCLUDE_INSTALL_DIR CONFIG_INSTALL_DIR)

#install .cmake files
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/tinysplineConfigVersion.cmake ${CMAKE_CURRENT_BINARY_DIR}/tinysplineConfig.cmake
  DESTINATION ${CONFIG_INSTALL_DIR})

