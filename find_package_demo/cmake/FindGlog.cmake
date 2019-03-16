find_package(PkgConfig)
pkg_check_modules(PC_GLOG QUIET libglog)
message(">>>> Found result using pkg-config:")
message("PKG_CONFIG_PATH = $ENV{PKG_CONFIG_PATH}")
message("<PREFIX>_FOUND = ${PC_GLOG_FOUND}")
message("<PREFIX>_PREFIX = ${PC_GLOG_PREFIX}")
message("<PREFIX>_LIBRARIES = ${PC_GLOG_LIBRARIES}")
message("<PREFIX>_INCLUDE_DIRS = ${PC_GLOG_INCLUDE_DIRS}")
message("<PREFIX>_VERSION = ${PC_GLOG_VERSION}")

if (${PC_GLOG_FOUND})
  set(Glog_FOUND ${PC_GLOG_FOUND})
  set(Glog_VERSION ${PC_GLOG_VERSION})
  set(Glog_INCLUDE_DIRS ${PC_GLOG_INCLUDE_DIRS})
  set(Glog_LIBRARIES ${PC_GLOG_LIBRARIES})
else ()
  find_path(Glog_INCLUDE_DIR
    NAMES glog
    PATHS ${PC_GLOG_INCLUDE_DIRS})
  find_library(Glog_LIBRARY_DIR
    NAMES glog
    PATHS ${PC_GLOG_LIBRARY_DIRS})

  include(FindPackageHandleStandardArgs)
  # if all listed variables are TRUE
  find_package_handle_standard_args(Glog
    FOUND_VAR Glog_FOUND
    REQUIRED_VARS Glog_INCLUDE_DIR Glog_LIBRARY_DIR
    VERSION_VAR PC_GLOG_VERSION)

  if (${Glog_FOUND})
    set(Glog_INCLUDE_DIRS ${Glog_INCLUDE_DIR})
    set(Glog_LIBRARIES ${Glog_LIBRARY_DIR})
    set(Glog_VERSION ${PC_GLOG_VERSION})
  endif ()
  mark_as_advanced(Glog_LIBRARY_DIR Glog_INCLUDE_DIR)
endif ()

if (${Glog_FOUND})
  message(STATUS "Found Glog include = ${Glog_INCLUDE_DIRS}")
  message(STATUS "Found Glog lib = ${Glog_LIBRARIES}")
  message(STATUS "Found Glog version: " ${Glog_VERSION})
else ()
  message(SEND_ERROR "Could not find Glog")
endif ()