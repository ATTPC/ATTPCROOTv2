cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
project(E20020Analysis)

#SET(GENFIT2_DIR "/mnt/simulations/attpcroot/fair_install_2020/GenFit_Inst/")

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

#list(APPEND CMAKE_PREFIX_PATH $ENV{ROOTSYS})
list(APPEND CMAKE_PREFIX_PATH ${SIMPATH}/..)  #Need access to dependecies of ATTPCROOT
list(APPEND CMAKE_PREFIX_PATH ${ATTPCROOTPATH}/build/install) #Need access to ATTPCROOT

find_package(ATTPCROOT REQUIRED)

add_library(e20020Ana SHARED e20020Ana.cc)
target_link_libraries(e20020Ana
  ROOT::Core
  ROOT::GenVector
  ROOT::VMC
  AtReconstruction
  AtTpc
  AtSimulationData
  FairRoot::FairTools
  FairRoot::Base
  GENFIT2::genfit2
)

add_executable(e20020AnaExe e20020Ana.cc)
target_link_libraries (e20020AnaExe e20020Ana
  GENFIT2::genfit2
  
  ROOT::EG
  ROOT::Core
  ROOT::GenVector
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
  AtMap
  AtTpc
  AtSimulationData

  FairRoot::FairTools
  FairRoot::Base)
