# Create a library called "libATDigi" which includes the source files given in the array .

set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
${CMAKE_SOURCE_DIR}/fairtools
${CMAKE_SOURCE_DIR}/parbase
${CMAKE_SOURCE_DIR}/geobase
${CMAKE_SOURCE_DIR}/base
# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/tpc
${CMAKE_SOURCE_DIR}/par
${CMAKE_SOURCE_DIR}/reco
${CMAKE_SOURCE_DIR}/digi
${CMAKE_SOURCE_DIR}/AtGenerators
${CMAKE_SOURCE_DIR}/AtMap
${CMAKE_SOURCE_DIR}/reco/ATDecoder
)

include_directories(${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}
${SIMPATH}/lib
)

link_directories(${LINK_DIRECTORIES})

set(SRCS
# Add all the source files below this line. Those must have cc for their extension.
ATAvalancheTask.cc
ATClusterizeTask.cc
ATPulseTask.cc
ATSimulatedPoint.cc
ATTrigger.cc
ATTriggerTask.cc

)

CHANGE_FILE_EXTENSION(*.cc *.hh HEADERS "${SRCS}")

Set(LINKDEF ATDigiLinkDef.h)
Set(LIBRARY_NAME ATDigi)
Set(DEPENDENCIES
  Base
  #MCStack
# Add all library names your code depending on.
# Basically those are the libraries in the header files' directory.
  AtTpc
  AtGen
  ATPar
  ATTPCReco
  AtTpcMap
)

GENERATE_LIBRARY()
