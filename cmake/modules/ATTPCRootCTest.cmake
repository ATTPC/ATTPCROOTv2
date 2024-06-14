# This file is meant to be included once in the main CMakelists.txt file
# It will download GoogleTest if needed and then add all registered tests
FetchContent_Declare(
  googletest  
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL https://github.com/google/googletest/archive/a7f443b80b105f940225332ed3c31f2790092f47.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Our test outputs should not sit in the normal desination
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)

#Search for all subdirectories with tests
get_property(test_dirs GLOBAL PROPERTY ATTPCROOT_TEST_DIRS)
foreach(d ${test_dirs})
  add_subdirectory(${d})
endforeach()
