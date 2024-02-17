# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_light_sensor_plugin_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED light_sensor_plugin_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(light_sensor_plugin_FOUND FALSE)
  elseif(NOT light_sensor_plugin_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(light_sensor_plugin_FOUND FALSE)
  endif()
  return()
endif()
set(_light_sensor_plugin_CONFIG_INCLUDED TRUE)

# output package information
if(NOT light_sensor_plugin_FIND_QUIETLY)
  message(STATUS "Found light_sensor_plugin: 0.0.0 (${light_sensor_plugin_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'light_sensor_plugin' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${light_sensor_plugin_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(light_sensor_plugin_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${light_sensor_plugin_DIR}/${_extra}")
endforeach()
