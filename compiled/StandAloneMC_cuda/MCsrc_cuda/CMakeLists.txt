# CMakeLists.txt for event package. It creates a library with dictionary and a main program
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(MCsrc_cuda)

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

#---Define useful ROOT functions and macros (e.g. ROOT_GENERATE_DICTIONARY)
include(${ROOT_USE_FILE})

#---Locate other external packages
find_package(Boost REQUIRED)
find_package(CUDA REQUIRED)

include_directories( ${ROOT_INCLUDE_DIRS}
                     ${Boost_INCLUDE_DIRS}
                     ${BASE_INCLUDE_DIRECTORIES}
                     ${ATTPCROOTPATH}/include
                     ${FAIRROOTPATH}/include
                     ${CMAKE_SOURCE_DIR}/MCsrc_cuda)

link_directories( ${FAIRROOT_LIBRARY_DIR}
                  ${FAIRROOTPATH}/lib
                  ${ATTPCROOTPATH}/build/lib)

#---Generate the dictionary  (G__MCsrc_cuda.cxx G__MCsrc_cuda.pcm)
ROOT_GENERATE_DICTIONARY(G__MCsrc_cuda MCsrc_cuda.hh LINKDEF MCsrc_cudaLinkDef.h)

#---Create a shared library with geneated dictionary
add_library(MCsrc_cuda SHARED G__MCsrc_cuda.cxx)
target_link_libraries(MCsrc_cuda ${ROOT_LIBRARIES} ATTPCReco FairTools Base)

#---Create  a main program using the library
cuda_add_executable(MC_cudaExe MCsrc_cuda.cu)
target_link_libraries(MC_cudaExe MCsrc_cuda
MCsrc_cuda
Core
Gpad
Graf
Graf3d
Gui
Hist
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
ATTPCReco
AtTpcMap
FairTools
Base)
