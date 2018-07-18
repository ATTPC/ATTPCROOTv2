cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(random_sample_consensus)

find_package(PCL 1.2 REQUIRED)
find_package(ROOT REQUIRED)

include(${ROOT_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS} ${ROOT_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${ROOT_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

add_executable (random_sample_consensus random_sample_consensus.cpp)
target_link_libraries (random_sample_consensus ${PCL_LIBRARIES} ${ROOT_LIBRARIES})