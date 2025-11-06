# rename_with_commit_and_build.cmake
# Usage (from CMake custom command):
#   ${CMAKE_COMMAND} -DOUTPUT_DIR=... -DPROJECT_NAME=... -DSTATE_FILE=... -DSOURCE_DIR=... -P rename_with_commit_and_build.cmake

# Validate inputs
if(NOT DEFINED OUTPUT_DIR)
  message(FATAL_ERROR "rename_with_commit_and_build.cmake: OUTPUT_DIR is not defined")
endif()
if(NOT DEFINED PROJECT_NAME)
  message(FATAL_ERROR "rename_with_commit_and_build.cmake: PROJECT_NAME is not defined")
endif()
if(NOT DEFINED STATE_FILE)
  message(FATAL_ERROR "rename_with_commit_and_build.cmake: STATE_FILE is not defined")
endif()
if(NOT DEFINED SOURCE_DIR)
  # Fallback: assume the script is launched from the build directory of the top-level project
  set(SOURCE_DIR "${CMAKE_SOURCE_DIR}")
endif()

# Ensure output directory exists
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

# Resolve current Git commit short hash (8 chars). Fallback to "nogit" if not available.
set(_git_hash "nogit")
set(_git_ok FALSE)
execute_process(
  COMMAND git rev-parse --short=8 HEAD
  WORKING_DIRECTORY "${SOURCE_DIR}"
  OUTPUT_VARIABLE _git_hash_out
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _git_rv
  ERROR_QUIET
)
if(_git_rv EQUAL 0 AND NOT "${_git_hash_out}" STREQUAL "")
  set(_git_hash "${_git_hash_out}")
  set(_git_ok TRUE)
endif()

# Detect dirty working tree (optional, best-effort). Do not affect counter reset logic.
set(_dirty_suffix "")
if(_git_ok)
  # git diff-index returns 0 when clean, 1 when dirty
  execute_process(
    COMMAND git diff-index --quiet HEAD --
    WORKING_DIRECTORY "${SOURCE_DIR}"
    RESULT_VARIABLE _git_dirty_rv
    ERROR_QUIET
  )
  if(_git_dirty_rv EQUAL 1)
    set(_dirty_suffix "-dirty")
  endif()
endif()

# Read state: last_commit (pure hash, without -dirty) and build_num
set(_last_commit "")
set(_build_num 0)
if(EXISTS "${STATE_FILE}")
  file(READ "${STATE_FILE}" _state_raw)
  foreach(_line IN LISTS _state_raw)
    # CMake doesn't split by lines by default; emulate simple parsing
  endforeach()
  # Simple manual parsing using regex for key=value pairs
  string(REGEX MATCH "last_commit=([^\n\r]+)" _m1 "${_state_raw}")
  if(_m1)
    string(REGEX REPLACE ".*last_commit=([^\n\r]+).*" "\\1" _last_commit "${_state_raw}")
  endif()
  string(REGEX MATCH "build_num=([0-9]+)" _m2 "${_state_raw}")
  if(_m2)
    string(REGEX REPLACE ".*build_num=([0-9]+).*" "\\1" _build_num "${_state_raw}")
  endif()
endif()

# Reset counter if commit changed
if(NOT "${_last_commit}" STREQUAL "${_git_hash}")
  set(_build_num 0)
endif()

# Increment and persist
math(EXPR _build_num "${_build_num} + 1")

# Zero-pad build number to 4 digits
string(LENGTH "${_build_num}" _bn_len)
set(_build_num_padded "${_build_num}")
if(_bn_len LESS 4)
  math(EXPR _zeros "4 - ${_bn_len}")
  string(REPEAT "0" ${_zeros} _pad)
  set(_build_num_padded "${_pad}${_build_num}")
endif()

# Write back state (pure hash, without dirty suffix)
file(WRITE "${STATE_FILE}" "last_commit=${_git_hash}\nbuild_num=${_build_num}\n")

# Helper to copy/rename if source exists
function(_copy_with_commit_and_build source_ext)
  set(src "${OUTPUT_DIR}/${PROJECT_NAME}.${source_ext}")
  if(EXISTS "${src}")
    set(dst "${OUTPUT_DIR}/${PROJECT_NAME}_${_git_hash}${_dirty_suffix}_b${_build_num_padded}.${source_ext}")
    configure_file("${src}" "${dst}" COPYONLY)
    message(STATUS "Renamed ${src} -> ${dst}")
  else()
    message(STATUS "File not found (skipped): ${src}")
  endif()
endfunction()

_copy_with_commit_and_build("uf2")
_copy_with_commit_and_build("bin")
_copy_with_commit_and_build("elf")
