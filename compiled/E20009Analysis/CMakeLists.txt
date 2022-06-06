# CMakeLists.txt for event package. It creates a library with dictionary and a main program
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(E20009Analysis)

SET(ATTPCROOTPATH $ENV{VMCWORKDIR})

list(APPEND CMAKE_PREFIX_PATH ${ATTPCROOTPATH}/build/install) #Need access to ATTPCROOT

# Will also load all of its dependecies
find_package(ATTPCROOT 0.3 REQUIRED)

add_library(e20009Ana SHARED e20009Ana.cc)
target_link_libraries(e20009Ana
  ROOT::Core
  ROOT::GenVector
  ROOT::VMC
  ATTPCROOT::AtReconstruction
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
ROOT::Geom
ROOT::Spectrum

ATTPCROOT::AtReconstruction
ATTPCROOT::AtMap

FairRoot::Base
FairRoot::FairTools
)
