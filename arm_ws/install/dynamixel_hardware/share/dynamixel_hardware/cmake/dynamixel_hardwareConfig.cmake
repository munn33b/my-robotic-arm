# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dynamixel_hardware_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dynamixel_hardware_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dynamixel_hardware_FOUND FALSE)
  elseif(NOT dynamixel_hardware_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dynamixel_hardware_FOUND FALSE)
  endif()
  return()
endif()
set(_dynamixel_hardware_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dynamixel_hardware_FIND_QUIETLY)
  message(STATUS "Found dynamixel_hardware: 0.0.0 (${dynamixel_hardware_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dynamixel_hardware' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT dynamixel_hardware_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dynamixel_hardware_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${dynamixel_hardware_DIR}/${_extra}")
endforeach()
