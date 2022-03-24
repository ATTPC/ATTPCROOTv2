# CMakeLists.txt for event package. It creates a library with dictionary and a main program
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(E20009Analysis)

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

set(CMAKE_MODULE_PATH "${ATTPCROOTPATH}/cmake/modules" ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")

list(APPEND CMAKE_PREFIX_PATH ${SIMPATH}/..)  #Need access to dependecies of ATTPCROOT
list(APPEND CMAKE_PREFIX_PATH ${ATTPCROOTPATH}/build/install) #Need access to ATTPCROOT


#---Locate the ROOT and VMC packages
find_package(ROOT 6.16 CONFIG)
if(NOT ROOT_FOUND)
  find_package(ROOT 6.0 REQUIRED)
  message("Looked for ROOT using modules and found ${ROOT_VERSION}")
endif()
if(ROOT_vmc_FOUND)
  set(VMCLIB ROOT::VMC)
else()
  message("NO VMC in ROOT install, looking for seperate package")
  find_package(VMC REQUIRED)
  set(VMCLIB VMCLibrary)
endif()

message("Looking in: ${CMAKE_PREFIX_PATH}")
#---Locate other external packages
find_package(Boost 1.67 REQUIRED)
find_package(GENFIT2 REQUIRED)
find_package(PCL) # temp until ATTPCROOT properly exports its dependencies
find_package(HiRAEVT) # temp until ATTPCROOT properly exports its dependencies
find_package(ATTPCROOT REQUIRED)

message(STATUS ${ATTPCROOTPATH})

add_library(e20009Ana SHARED e20009Ana.cc)
target_link_libraries(e20009Ana
  ROOT::Core
  ROOT::GenVector
  ${VMCLIB}
  AtReconstruction
  FairTools
  Base
  ${GENFIT2_LIBRARY_DIR}/libgenfit2.so)
target_include_directories(e20009Ana PRIVATE ${GENFIT2_INCLUDE_DIR} ${FAIRROOTPATH}/include)

add_executable(e20009AnaExe e20009Ana.cc)
target_link_libraries (e20009AnaExe e20009Ana
${GENFIT2_LIBRARY_DIR}/libgenfit2.so
ROOT::EG
ROOT::Core
ROOT::Gpad
ROOT::Graf
ROOT::Graf3d
ROOT::Gui
ROOT::Hist
ROOT::MathCore
ROOT::Matrix
ROOT::Minuit
ROOT::Net
ROOT::Physics
ROOT::Postscript
ROOT::Rint
ROOT::RIO
ROOT::Thread
ROOT::Tree
ROOT::TreePlayer

AtReconstruction
AtMap

FairTools
Geom
Spectrum
Base)
target_include_directories(e20009AnaExe PRIVATE ${GENFIT2_INCLUDE_DIR} ${FAIRROOTPATH}/include)
