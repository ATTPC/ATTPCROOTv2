# This file is meant to be included once in the main CMakelists.txt file
# It will download GoogleTest if needed and then add all registered tests
if(NOT BUILD_TESTS)
  message(STATUS "Tests are disabled ${BUILD_TESTS}")
  function(attpcroot_generate_tests TEST_NAME)
    cmake_parse_arguments(ARG "" "" "SRCS;DEPS" ${ARGN})
  endfunction()

  return()
endif()

FetchContent_Declare(
  googletest  
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL https://github.com/google/googletest/archive/a7f443b80b105f940225332ed3c31f2790092f47.zip
)
FetchContent_MakeAvailable(googletest)

include(CTest)
include(GoogleTest)

function(attpcroot_generate_tests TEST_NAME)
  cmake_parse_arguments(ARG "" "" "SRCS;DEPS" ${ARGN})

  # We will save tests to a different location than the main build
  set(ORIG_CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
  
  message(STATUS "Generating test ${TEST_NAME} with sources ${ARG_SRCS} and dependencies ${ARG_DEPS}")
  
  add_executable(${TEST_NAME} ${ARG_SRCS})
  target_link_libraries(${TEST_NAME} PRIVATE ${ARG_DEPS} GTest::gtest_main)
  gtest_discover_tests(${TEST_NAME} TEST_PREFIX "${TEST_NAME}.")

  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${ORIG_CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endfunction()
