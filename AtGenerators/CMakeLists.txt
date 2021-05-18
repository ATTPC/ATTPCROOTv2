# Create a library called "libGen" which includes the source files given in
# the array .
# The extension is already found.  Any number of sources could be listed here.

set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${PYTHIA8_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${SIMPAtH}/include
${BASE_INCLUDE_DIRECTORIES}
${CMAKE_SOURCE_DIR}/AtGenerators
${CMAKE_SOURCE_DIR}/AtData
${CMAKE_SOURCE_DIR}/AtTpc

)

include_directories( ${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}
${SIMPAtH}/lib
)

link_directories(${LINK_DIRECTORIES})

set(SRCS
#Pythia6Generator.cxx
#Pythia8Generator.cxx
AtTPCIonDecay.cxx
AtTPCIonGenerator.cxx
AtTPCIonPhaseSpace.cxx
AtVertexPropagator.cxx
AtTPC2Body.cxx
AtTPC_d2He.cxx
AtTPC_Background.cxx
AtEulerTransformation.cxx
AtTPCFissionGenerator.cxx
AtTPCFissionGeneratorV2.cxx
AtTPCFissionGeneratorV3.cxx
AtTPCXSReader.cxx
AtTPCXSManager.cxx
AtTPCGammaDummyGenerator.cxx
AtTPC20MgDecay.cxx
)

set(LINKDEF  GenLinkDef.h)
set(LIBRARY_NAME AtGenerators)

#set(DEPENDENCIES  Base AtData pythia8 Pythia6)
set(DEPENDENCIES  Base AtData FairLogger::FairLogger)

GENERAtE_LIBRARY()
