# CMakeLists.txt for event package. It creates a library with dictionary and a main program
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(EFitterAnalysis)

SET(ATTPCROOTPATH $ENV{VMCWORKDIR})
list(APPEND CMAKE_PREFIX_PATH ${ATTPCROOTPATH}/build/install) #Need access to ATTPCROOT

# Will also load all of its dependecies
find_package(ATTPCROOT 0.3 REQUIRED)
find_package(Curses)

include_directories( INCLUDE_DIRECTORIES
		     ${CURSES_INCLUDE_DIRS}
		     )

link_directories(LINK_DIRECTORIES
  ${CURSES_LIBRARIES}  )


add_library(eFitterAnalysis SHARED eFitterAnalysis.cc)
target_link_libraries(eFitterAnalysis ${ROOT_LIBRARIES}
  GENFIT2::genfit2

  ATTPCROOT::AtData
  ATTPCROOT::AtTools
  ATTPCROOT::AtSimulationData
  )

add_executable(eFitterAnaExe eFitterAnalysis.cc)

target_link_libraries (eFitterAnaExe eFitterAnalysis
${ROOT_LIBRARIES}
${CURSES_LIBRARIES}
GENFIT2::genfit2
ATTPCROOT::AtReconstruction
ATTPCROOT::AtTools
ATTPCROOT::AtMap
FairRoot::Base
FairRoot::FairTools
)
