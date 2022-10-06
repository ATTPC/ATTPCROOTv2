# CMakeLists.txt for event package. It creates a library with dictionary and a main program
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(EFitterSpecMAT)

SET(ATTPCROOTPATH $ENV{VMCWORKDIR})
list(APPEND CMAKE_PREFIX_PATH ${ATTPCROOTPATH}/build/install) #Need access to ATTPCROOT

# Will also load all of its dependecies
find_package(ATTPCROOT 0.3 REQUIRED)

include_directories( INCLUDE_DIRECTORIES
		     
		     )

link_directories(LINK_DIRECTORIES
    )


add_library(eFitterSpecMAT SHARED eFitterSpecMAT.cc)
target_link_libraries(eFitterSpecMAT ${ROOT_LIBRARIES}
  GENFIT2::genfit2

  ATTPCROOT::AtData
  ATTPCROOT::AtTools
  ATTPCROOT::AtSimulationData
  )

add_executable(eFitterSM eFitterSpecMAT.cc)
		
target_link_libraries (eFitterSM eFitterSpecMAT
${ROOT_LIBRARIES}
${CURSES_LIBRARIES}
GENFIT2::genfit2
ATTPCROOT::AtReconstruction
ATTPCROOT::AtTools
ATTPCROOT::AtMap
FairRoot::Base
FairRoot::FairTools
)
