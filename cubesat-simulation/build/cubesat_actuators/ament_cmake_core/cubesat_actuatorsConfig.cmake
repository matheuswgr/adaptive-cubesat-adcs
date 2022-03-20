# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cubesat_actuators_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cubesat_actuators_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cubesat_actuators_FOUND FALSE)
  elseif(NOT cubesat_actuators_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cubesat_actuators_FOUND FALSE)
  endif()
  return()
endif()
set(_cubesat_actuators_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cubesat_actuators_FIND_QUIETLY)
  message(STATUS "Found cubesat_actuators: 0.0.0 (${cubesat_actuators_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cubesat_actuators' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cubesat_actuators_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cubesat_actuators_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cubesat_actuators_DIR}/${_extra}")
endforeach()
