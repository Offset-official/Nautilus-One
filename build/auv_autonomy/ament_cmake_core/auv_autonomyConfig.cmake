# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_auv_autonomy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED auv_autonomy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(auv_autonomy_FOUND FALSE)
  elseif(NOT auv_autonomy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(auv_autonomy_FOUND FALSE)
  endif()
  return()
endif()
set(_auv_autonomy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT auv_autonomy_FIND_QUIETLY)
  message(STATUS "Found auv_autonomy: 0.0.0 (${auv_autonomy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'auv_autonomy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${auv_autonomy_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(auv_autonomy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${auv_autonomy_DIR}/${_extra}")
endforeach()
