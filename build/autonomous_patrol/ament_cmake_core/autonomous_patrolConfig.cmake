# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_autonomous_patrol_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED autonomous_patrol_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(autonomous_patrol_FOUND FALSE)
  elseif(NOT autonomous_patrol_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(autonomous_patrol_FOUND FALSE)
  endif()
  return()
endif()
set(_autonomous_patrol_CONFIG_INCLUDED TRUE)

# output package information
if(NOT autonomous_patrol_FIND_QUIETLY)
  message(STATUS "Found autonomous_patrol: 0.0.1 (${autonomous_patrol_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'autonomous_patrol' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${autonomous_patrol_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(autonomous_patrol_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${autonomous_patrol_DIR}/${_extra}")
endforeach()
