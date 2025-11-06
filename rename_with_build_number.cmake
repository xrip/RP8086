# rename_with_build_number.cmake
# Usage (from CMake custom command):
#   ${CMAKE_COMMAND} -DOUTPUT_DIR=... -DPROJECT_NAME=... -DBUILD_NUMBER_FILE=... -P rename_with_build_number.cmake

if(NOT DEFINED OUTPUT_DIR)
  message(FATAL_ERROR "rename_with_build_number.cmake: OUTPUT_DIR is not defined")
endif()
if(NOT DEFINED PROJECT_NAME)
  message(FATAL_ERROR "rename_with_build_number.cmake: PROJECT_NAME is not defined")
endif()
if(NOT DEFINED BUILD_NUMBER_FILE)
  message(FATAL_ERROR "rename_with_build_number.cmake: BUILD_NUMBER_FILE is not defined")
endif()

# Ensure directory exists
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

# Read current build number (default to 0)
set(_build_num 0)
if(EXISTS "${BUILD_NUMBER_FILE}")
  file(READ "${BUILD_NUMBER_FILE}" _bn_raw)
  string(STRIP "${_bn_raw}" _bn_raw)
  if(_bn_raw MATCHES "^[0-9]+$")
    set(_build_num "${_bn_raw}")
  endif()
endif()

# Increment and persist
math(EXPR _build_num "${_build_num} + 1")
file(WRITE "${BUILD_NUMBER_FILE}" "${_build_num}\n")

# Format build number with zero padding (4 digits)
string(LENGTH "${_build_num}" _bn_len)
set(_build_num_padded "${_build_num}")
if(_bn_len LESS 4)
  math(EXPR _zeros "4 - ${_bn_len}")
  string(REPEAT "0" ${_zeros} _pad)
  set(_build_num_padded "${_pad}${_build_num}")
endif()

# Helper to copy/rename if source exists
function(_copy_with_build source_ext)
  set(src "${OUTPUT_DIR}/${PROJECT_NAME}.${source_ext}")
  if(EXISTS "${src}")
    set(dst "${OUTPUT_DIR}/${PROJECT_NAME}_build${_build_num_padded}.${source_ext}")
    # Use configure_file for a simple copy and rename
    configure_file("${src}" "${dst}" COPYONLY)
    message(STATUS "Renamed ${src} -> ${dst}")
  else()
    message(STATUS "File not found (skipped): ${src}")
  endif()
endfunction()

_copy_with_build("uf2")
