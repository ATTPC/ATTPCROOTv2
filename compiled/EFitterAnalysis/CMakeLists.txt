# CMakeLists.txt for event package. It creates a library with dictionary and a main program
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(EFitterAnalysis)

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
find_package(Boost 1.67 COMPONENTS system REQUIRED)
find_package(GENFIT2)
find_package(Curses)

include_directories( INCLUDE_DIRECTORIES
                     ${ROOT_INCLUDE_DIR}
                     ${Boost_INCLUDE_DIRS}
                     ${BASE_INCLUDE_DIRECTORIES}
                     ${ATTPCROOTPATH}/include
                     ${FAIRROOTPATH}/include
                     ${CMAKE_SOURCE_DIR}
		     ${GENFIT2_INCLUDE_DIR}
		     ${CURSES_INCLUDE_DIRS}
		     #${PCL_INCLUDE_DIRS}
		     )

link_directories(LINK_DIRECTORIES
		  ${FAIRROOT_LIBRARY_DIR}
                  ${FAIRROOTPATH}/lib
                  ${ATTPCROOTPATH}/build/lib
                  ${GENFIT2_LIBRARY_DIR}
		  ${CURSES_LIBRARIES}
		  ${Boost_LOG_LIBRARY}
		  ${Boost_LOG_SETUP_LIBRARY}
		  ${Boost_THREAD_LIBRARY}
		  ${Boost_DAtE_TIME_LIBRARY}
		  ${Boost_FILESYSTEM_LIBRARY}
		  ${Boost_SYSTEM_LIBRARY}
                  #${PCL_LIBRARY_DIRS}
                  )


#add_definitions(${PCL_DEFINITIONS})

message(STATUS ${ATTPCROOTPATH})

add_library(eFitterAnalysis SHARED eFitterAnalysis.cc)
target_link_libraries(eFitterAnalysis ${ROOT_LIBRARIES}   ${GENFIT2_LIBRARY_DIR}/libgenfit2.so)

add_executable(eFitterAnaExe eFitterAnalysis.cc)
		
target_link_libraries (eFitterAnaExe eFitterAnalysis
${ROOT_LIBRARIES}
${GENFIT2_LIBRARY_DIR}/libgenfit2.so
${CURSES_LIBRARIES}
${Boost_LOG_LIBRARY}
${Boost_LOG_SETUP_LIBRARY}
${Boost_THREAD_LIBRARY}
${Boost_DAtE_TIME_LIBRARY}
${Boost_FILESYSTEM_LIBRARY}
${Boost_SYSTEM_LIBRARY}
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
AtTools
AtTpcMap
FairTools
Geom
Spectrum
Base
)
