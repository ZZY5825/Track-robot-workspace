# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_track_robot_drivers_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED track_robot_drivers_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(track_robot_drivers_FOUND FALSE)
  elseif(NOT track_robot_drivers_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(track_robot_drivers_FOUND FALSE)
  endif()
  return()
endif()
set(_track_robot_drivers_CONFIG_INCLUDED TRUE)

# output package information
if(NOT track_robot_drivers_FIND_QUIETLY)
  message(STATUS "Found track_robot_drivers: 0.0.0 (${track_robot_drivers_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'track_robot_drivers' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${track_robot_drivers_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(track_robot_drivers_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${track_robot_drivers_DIR}/${_extra}")
endforeach()
