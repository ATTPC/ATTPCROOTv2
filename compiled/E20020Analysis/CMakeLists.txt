cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
project(E20020Analysis)

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


set(CMAKE_MODULE_PATH "${ATTPCROOTPATH}/cmake/modules" ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")

message(${CMAKE_MODULE_PATH})

# You need to tell CMake where to find the ROOT installation. This can be done in a number of ways:
#   - ROOT built with classic configure/make use the provided $ROOTSYS/etc/cmake/FindROOT.cmake
#   - ROOT built with CMake. Add in CMAKE_PREFIX_PATH the installation prefix for ROOT
list(APPEND CMAKE_PREFIX_PATH $ENV{ROOTSYS})

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

#---Locate other external packages
find_package(Boost 1.67 REQUIRED)
find_package(GENFIT2)

include_directories( ${Boost_INCLUDE_DIRS}
                     ${BASE_INCLUDE_DIRECTORIES}
                     ${ATTPCROOTPATH}/build/include
		     ${ATTPCROOTPATH}/build/include/AtDecoder
		     ${ATTPCROOTPATH}/build/include/AtFitter
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

add_library(e20020Ana SHARED e20020Ana.cc)
target_link_libraries(e20020Ana
  ROOT::Core
  ${VMCLIB}
  AtReconstruction
  AtTpc
  FairTools
  Base
  ${GENFIT2_LIBRARY_DIR}/libgenfit2.so)

add_executable(e20020AnaExe e20020Ana.cc)
target_link_libraries (e20020AnaExe e20020Ana
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
ROOT::Geom

AtReconstruction
AtTpcMap
AtTpc
FairTools
Base)
