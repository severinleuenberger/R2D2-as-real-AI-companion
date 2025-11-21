# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_r2d2_llm_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED r2d2_llm_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(r2d2_llm_FOUND FALSE)
  elseif(NOT r2d2_llm_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(r2d2_llm_FOUND FALSE)
  endif()
  return()
endif()
set(_r2d2_llm_CONFIG_INCLUDED TRUE)

# output package information
if(NOT r2d2_llm_FIND_QUIETLY)
  message(STATUS "Found r2d2_llm: 0.0.0 (${r2d2_llm_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'r2d2_llm' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${r2d2_llm_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(r2d2_llm_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${r2d2_llm_DIR}/${_extra}")
endforeach()
