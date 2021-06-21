# CMakeLists.txt for event package. It creates a library with dictionary and a main program
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ROOT2HDF)

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

# You need to tell CMake where to find the ROOT installation. This can be done in a number of ways:
#   - ROOT built with classic configure/make use the provided $ROOTSYS/etc/cmake/FindROOT.cmake
#   - ROOT built with CMake. Add in CMAKE_PREFIX_PATH the installation prefix for ROOT
list(APPEND CMAKE_PREFIX_PATH $ENV{ROOTSYS})

#---Locate the ROOT package and defines a number of variables (e.g. ROOT_INCLUDE_DIRS)
find_package(ROOT REQUIRED COMPONENTS RIO)
find_package(HDF5 REQUIRED COMPONENTS CXX HL)

#---Define useful ROOT functions and macros (e.g. ROOT_GENERATE_DICTIONARY)
include(${ROOT_USE_FILE})

include_directories( ${ROOT_INCLUDE_DIR}               
                     ${BASE_INCLUDE_DIRECTORIES}
                     ${ATTPCROOTPATH}/include
                     ${FAIRROOTPATH}/include
                     ${CMAKE_SOURCE_DIR}
		     ${HDF5_INCLUDE_DIRS} 
		     ${HDF5_HL_INCLUDE_DIR})

link_directories( ${FAIRROOT_LIBRARY_DIR}
                  ${FAIRROOTPATH}/lib
                  ${ATTPCROOTPATH}/build/lib
                  )

add_library(R2H SHARED R2HMain.cc)
target_link_libraries(R2H ${ROOT_LIBRARIES} ${HDF5_CXX_LIBRARIES} ${HDF5_CXX_HL_LIBRARIES} ${HDF5_HL_LIBRARIES} AtReconstruction FairTools Base)

add_executable(R2HExe R2HMain.cc)
target_link_libraries (R2HExe R2H
  ${ROOT_LIBRARIES}
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
Base)
