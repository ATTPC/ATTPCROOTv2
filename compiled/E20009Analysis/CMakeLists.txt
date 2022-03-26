# CMakeLists.txt for event package. It creates a library with dictionary and a main program
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(E20009Analysis)

SET(SIMPATH $ENV{SIMPATH})
SET(FAIRROOTPATH $ENV{FAIRROOTPATH})
SET(ATTPCROOTPATH $ENV{VMCWORKDIR})

list(APPEND CMAKE_PREFIX_PATH ${ATTPCROOTPATH}/build/install) #Need access to ATTPCROOT

find_package(ATTPCROOT REQUIRED 1.0)

add_library(e20009Ana SHARED e20009Ana.cc)
target_link_libraries(e20009Ana
  ROOT::Core
  ROOT::GenVector
  ROOT::VMC
  AtReconstruction
  FairRoot::FairTools
  FairRoot::Base
  GENFIT2::genfit2
)

add_executable(e20009AnaExe e20009Ana.cc)
target_link_libraries (e20009AnaExe e20009Ana
GENFIT2::genfit2
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

FairRoot::Base
FairRoot::FairTools
ROOT::Geom
ROOT::Spectrum
)
