# Create a library called "libGen" which includes the source files given in
# the array .
# The extension is already found.  Any number of sources could be listed here.

set(LIBRARY_NAME AtGenerators)
set(LINKDEF  GenLinkDef.h)

set(SRCS
#Pythia6Generator.cxx
#Pythia8Generator.cxx
AtTPCIonDecay.cxx
AtTPCIonGenerator.cxx
AtTPCIonPhaseSpace.cxx

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

set(INCLUDE_DIRECTORIES
${PYTHIA8_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
#${CMAKE_SOURCE_DIR}/AtGenerators
#${CMAKE_SOURCE_DIR}/AtData
#${CMAKE_SOURCE_DIR}/AtTpc

)
set(DEPENDENCIES
  Base
  FairTools
  ROOT::Core
  AtData
  AtTpc
  )

set(LINK_DIRECTORIES
${FAIRROOT_LIBRARY_DIR}
)

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

