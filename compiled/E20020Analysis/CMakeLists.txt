cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
project(E20020Analysis)

SET(ATTPCROOTPATH $ENV{VMCWORKDIR})

list(APPEND CMAKE_PREFIX_PATH ${ATTPCROOTPATH}/build/install) #Need access to ATTPCROOT

# Will also load all of its dependecies
find_package(ATTPCROOT 0.3 REQUIRED CONFIG)

add_library(e20020Ana SHARED e20020Ana.cc)
target_link_libraries(e20020Ana
  ROOT::Core
  ROOT::GenVector
  ROOT::VMC
  ATTPCROOT::AtReconstruction
  ATTPCROOT::AtTpc
  ATTPCROOT::AtSimulationData
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

  ATTPCROOT::AtReconstruction
  ATTPCROOT::AtMap
  ATTPCROOT::AtTpc
  ATTPCROOT::AtSimulationData

  FairRoot::FairTools
  FairRoot::Base)
