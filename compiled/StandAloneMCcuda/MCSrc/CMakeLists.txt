# CMakeLists.txt for event package. It creates a library with dictionary and a main program
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(MCSrc)

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
find_package(OpenMP)
find_package(CUDA REQUIRED)

if (OPENMP_FOUND)
    MESSAGE("-- OpenMP Support Found")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

endif()

include_directories( ${ROOT_INCLUDE_DIR}
                     ${Boost_INCLUDE_DIRS}
                     ${BASE_INCLUDE_DIRECTORIES}
                     ${ATTPCROOTPATH}/include
                     ${FAIRROOTPATH}/include
                     ${CMAKE_SOURCE_DIR}/MCSrc)

link_directories( ${FAIRROOT_LIBRARY_DIR}
                  ${FAIRROOTPATH}/lib
                  ${ATTPCROOTPATH}/build/lib)

#---Generate the dictionary  (G__MCSrc.cxx G__MCSrc.pcm)
ROOT_GENERATE_DICTIONARY(G__MCSrc MCSrc.hh MCMinimization.cuh LINKDEF MCSrcLinkDef.h)

#---Create a shared library with geneated dictionary
add_library(MCSrc SHARED G__MCSrc.cxx)
target_link_libraries(MCSrc ${ROOT_LIBRARIES} ATTPCReco FairTools Base)

#---Create  a main program using the library
cuda_add_executable(MCExe MCSrc.cc MCMin.cu)

set( custom_cuda_flags "-arch=sm_20 -lcurand -L/opt/software/cudatoolkit/7.0/lib64//opt/software/cudatoolkit/7.0/lib64" CACHE STRING "Set the cuda flags like --arch=sm_20 --ptxas-options=-v --use_fast_math")
list(APPEND CUDA_NVCC_FLAGS ${custom_cuda_flags})
SEPARATE_ARGUMENTS(custom_cuda_flags)
MESSAGE( STATUS "custom_cuda_flags             = ${custom_cuda_flags}" )

target_link_libraries(MCExe MCSrc
MCSrc
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
