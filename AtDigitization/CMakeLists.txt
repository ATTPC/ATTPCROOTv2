# Create a library called "libAtDigitization" which includes the source files given in the array .

Set(LINKDEF AtDigiLinkDef.h)
Set(LIBRARY_NAME AtDigitization)
Set(DEPENDENCIES
  Base
  #MCStack
# Add all library names your code depending on.
# Basically those are the libraries in the header files' directory.
  AtTpc
  AtGen
  AtParameter
  AtTPCReco
  AtTpcMap
)

set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
${CMAKE_SOURCE_DIR}/fairtools
${CMAKE_SOURCE_DIR}/parbase
${CMAKE_SOURCE_DIR}/geobase
${CMAKE_SOURCE_DIR}/base

# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/AtTpc
${CMAKE_SOURCE_DIR}/AtParameter
${CMAKE_SOURCE_DIR}/reco
${CMAKE_SOURCE_DIR}/AtDigitization
${CMAKE_SOURCE_DIR}/AtGenerators
${CMAKE_SOURCE_DIR}/AtMap
${CMAKE_SOURCE_DIR}/reco/AtDecoder
)

include_directories(${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}
${SIMPAtH}/lib
)

link_directories(${LINK_DIRECTORIES})

set(SRCS
# Add all the source files below this line. Those must have cc for their extension.
AtAvalancheTask.cxx
AtClusterizeTask.cxx
AtPulseTask.cxx
AtSimulatedPoint.cxx
AtTrigger.cxx
AtTriggerTask.cxx
)

GENERAtE_LIBRARY()
