# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_aruco_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED aruco_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(aruco_FOUND FALSE)
  elseif(NOT aruco_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(aruco_FOUND FALSE)
  endif()
  return()
endif()
set(_aruco_CONFIG_INCLUDED TRUE)

# output package information
if(NOT aruco_FIND_QUIETLY)
  message(STATUS "Found aruco: 0.0.0 (${aruco_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'aruco' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT aruco_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(aruco_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${aruco_DIR}/${_extra}")
endforeach()
