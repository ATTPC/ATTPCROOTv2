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
AtTPCIonGeneratorS800.cxx
AtTPCIonGeneratorGaussian.cxx
AtTPCIonPhaseSpace.cxx

AtTPC2Body.cxx
AtTPC_d2He.cxx
AtTPC_Background.cxx

AtTPCFissionGenerator.cxx
AtTPCFissionGeneratorV2.cxx
AtTPCFissionGeneratorV3.cxx
AtTPCXSReader.cxx
AtTPCXSManager.cxx
AtTPCGammaDummyGenerator.cxx
AtTPC20MgDecay.cxx
AtTPC20MgDecay_pag.cxx

)

set(DEPENDENCIES
  FairRoot::Base
  FairRoot::FairTools
  ROOT::Core
  ROOT::MathCore
  ROOT::GenVector

  ATTPCROOT::AtSimulationData
  ATTPCROOT::AtTools
  )

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

