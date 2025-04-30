# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_qcar2_nodes_copy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED qcar2_nodes_copy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(qcar2_nodes_copy_FOUND FALSE)
  elseif(NOT qcar2_nodes_copy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(qcar2_nodes_copy_FOUND FALSE)
  endif()
  return()
endif()
set(_qcar2_nodes_copy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT qcar2_nodes_copy_FIND_QUIETLY)
  message(STATUS "Found qcar2_nodes_copy: 0.0.0 (${qcar2_nodes_copy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'qcar2_nodes_copy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${qcar2_nodes_copy_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(qcar2_nodes_copy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${qcar2_nodes_copy_DIR}/${_extra}")
endforeach()
