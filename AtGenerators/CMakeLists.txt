# Create a library called "libGen" which includes the source files given in
# the array .
# The extension is already found.  Any number of sources could be listed here.

set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${PYTHIA8_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${SIMPATH}/include
${BASE_INCLUDE_DIRECTORIES}
${CMAKE_SOURCE_DIR}/AtGenerators
${CMAKE_SOURCE_DIR}/generators
${CMAKE_SOURCE_DIR}/AtData
${CMAKE_SOURCE_DIR}/tpc

)

include_directories( ${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}
${SIMPATH}/lib
)

link_directories( ${LINK_DIRECTORIES})

set(SRCS
#Pythia6Generator.cxx
#Pythia8Generator.cxx
ATTPCIonGenerator.cxx
ATTPCIonPhaseSpace.cxx
ATVertexPropagator.cxx
ATTPC2Body.cxx
ATTPC_d2He.cxx
ATTPC_Background.cxx
ATEulerTransformation.cxx
ATTPCFissionGenerator.cxx
ATTPCFissionGeneratorV2.cxx
ATTPCFissionGeneratorV3.cxx
ATTPCXSReader.cxx
)

set(LINKDEF  GenLinkDef.h)
set(LIBRARY_NAME AtGen)

#set(DEPENDENCIES  Base AtData pythia8 Pythia6)
set(DEPENDENCIES  Base AtData FairLogger::FairLogger)

GENERATE_LIBRARY()
