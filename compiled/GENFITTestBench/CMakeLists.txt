# CMakeLists.txt for event package. It creates a library with dictionary and a main program
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(GENFITTestBench)

#SET(BOOST_ROOT "/mnt/data/fair_install/boost_1_67_0/")
#SET(PCL_DIR "/mnt/data/fair_install/pcl_1.8.1/share/pcl-1.8/")

SET(GENFIT2_DIR "/mnt/simulations/attpcroot/fair_install_2020/GenFit_Inst/")

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

#set(CMAKE_MODULE_PATH "/mnt/simulations/attpcroot/fair_install_2020/yassid/ATTPCROOTv2/cmake/modules")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")

message(${CMAKE_MODULE_PATH})

# You need to tell CMake where to find the ROOT installation. This can be done in a number of ways:
#   - ROOT built with classic configure/make use the provided $ROOTSYS/etc/cmake/FindROOT.cmake
#   - ROOT built with CMake. Add in CMAKE_PREFIX_PATH the installation prefix for ROOT
list(APPEND CMAKE_PREFIX_PATH $ENV{ROOTSYS})

#---Locate the ROOT package and defines a number of variables (e.g. ROOT_INCLUDE_DIRS)
find_package(ROOT REQUIRED)

#---Define useful ROOT functions and macros (e.g. ROOT_GENERATE_DICTIONARY)
include(${ROOT_USE_FILE})

#---Locate other external packages
find_package(Boost 1.67 REQUIRED)
find_package(GENFIT2)
#find_package(PCL 1.8 REQUIRED)

include_directories( ${ROOT_INCLUDE_DIR}
                     ${Boost_INCLUDE_DIRS}
                     ${BASE_INCLUDE_DIRECTORIES}
                     ${ATTPCROOTPATH}/include
                     ${FAIRROOTPATH}/include
                     ${CMAKE_SOURCE_DIR}
		     ${GENFIT2_INCLUDE_DIR}
                     #${PCL_INCLUDE_DIRS}
		     )

link_directories( ${FAIRROOT_LIBRARY_DIR}
                  ${FAIRROOTPATH}/lib
                  ${ATTPCROOTPATH}/build/lib
                  ${GENFIT2_LIBRARY_DIR}
                  #${PCL_LIBRARY_DIRS}
                  )

#add_definitions(${PCL_DEFINITIONS})

message(STATUS ${ATTPCROOTPATH})

add_library(genfit_Test SHARED genfit_Test.cc)
#target_link_libraries(genfit_Test ${ROOT_LIBRARIES} ATTPCReco FairTools Base ${PCL_LIBRARIES})
target_link_libraries(genfit_Test ${ROOT_LIBRARIES} AtReconstruction FairTools Base ${GENFIT2_LIBRARY_DIR}/libgenfit2.so)

add_executable(genfit_Test_exe genfit_Test.cc)
target_link_libraries (genfit_Test_exe genfit_Test
  ${ROOT_LIBRARIES}
${GENFIT2_LIBRARY_DIR}/libgenfit2.so
EG
Core
Gpad
Graf
Graf3d
Gui
Hist
MathCore
Matrix
Minuit
Net
Physics
Postscript
Rint
RIO
Thread
Tree
TreePlayer
AtReconstruction
AtTpcMap
FairTools
Geom
Base)
