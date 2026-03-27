# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bunker_base_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bunker_base_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bunker_base_FOUND FALSE)
  elseif(NOT bunker_base_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bunker_base_FOUND FALSE)
  endif()
  return()
endif()
set(_bunker_base_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bunker_base_FIND_QUIETLY)
  message(STATUS "Found bunker_base: 0.1.0 (${bunker_base_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bunker_base' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${bunker_base_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bunker_base_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bunker_base_DIR}/${_extra}")
endforeach()
