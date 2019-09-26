cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCLFAIRROOT_Test)

set(PCL_DIR "/mnt/data/fair_install/pcl_1.8.1/share/pcl-1.8/")

# Check for needed environment variables
IF(NOT DEFINED ENV{FAIRROOTPATH})
  MESSAGE(FATAL_ERROR "You did not define the environment variable FAIRROOTPATH which is needed to find FairRoot. Please set this variable and execute cmake again.")
ENDIF(NOT DEFINED ENV{FAIRROOTPATH})

IF(NOT DEFINED ENV{SIMPATH})
   MESSAGE(FATAL_ERROR "You did not define the environment variable SIMPATH which is nedded to find the external packages. Please set this variable and execute cmake again.")
ENDIF(NOT DEFINED ENV{SIMPATH})

SET(SIMPATH $ENV{SIMPATH})
SET(FAIRROOTPATH $ENV{FAIRROOTPATH})
SET(ATTPCROOTPATH $ENV{VMCWORKDIR})

find_package(PCL 1.8 REQUIRED)
find_package(ROOT REQUIRED)

include(${ROOT_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS} ${ROOT_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${ROOT_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

add_executable (PCLFAIRROOT_Test PCLFAIRROOT_Test.cpp)
target_link_libraries (PCLFAIRROOT_Test ${PCL_LIBRARIES} ${ROOT_LIBRARIES})
