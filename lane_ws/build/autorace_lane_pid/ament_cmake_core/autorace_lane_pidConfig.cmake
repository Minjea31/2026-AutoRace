# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_autorace_lane_pid_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED autorace_lane_pid_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(autorace_lane_pid_FOUND FALSE)
  elseif(NOT autorace_lane_pid_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(autorace_lane_pid_FOUND FALSE)
  endif()
  return()
endif()
set(_autorace_lane_pid_CONFIG_INCLUDED TRUE)

# output package information
if(NOT autorace_lane_pid_FIND_QUIETLY)
  message(STATUS "Found autorace_lane_pid: 0.1.0 (${autorace_lane_pid_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'autorace_lane_pid' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${autorace_lane_pid_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(autorace_lane_pid_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${autorace_lane_pid_DIR}/${_extra}")
endforeach()
