cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(random_sample_consensus)

#set(PCL_DIR "/mnt/data/fair_install/pcl_1.8.1/share/pcl-1.8/")
set(PCL_DIR "/mnt/simulations/attpcroot/fair_install_2020/pcl_1.9.1_inst/share/pcl-1.10/")

find_package(PCL 1.9 REQUIRED)
find_package(ROOT REQUIRED)

include(${ROOT_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS} ${ROOT_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${ROOT_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

add_executable (random_sample_consensus random_sample_consensus.cpp)
target_link_libraries (random_sample_consensus ${PCL_LIBRARIES} ${ROOT_LIBRARIES})
