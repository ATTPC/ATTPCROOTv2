#Create a library called "libAtDigitization" which includes the source files given in the array.

Set(LINKDEF AtDigiLinkDef.h)
Set(LIBRARY_NAME AtDigitization)
Set(DEPENDENCIES
  Base
  FairTools

  ROOT::Core
  ROOT::GenVector
  
  AtTpc
  AtGenerators
  AtParameter
  AtReconstruction
  AtTpcMap
  )

set(INCLUDE_DIRECTORIES
  ${BASE_INCLUDE_DIRECTORIES}
)
set(LINK_DIRECTORIES
${FAIRROOT_LIBRARY_DIR}
)

set(SRCS
# Add all the source files below this line.
AtAvalancheTask.cxx
AtClusterizeTask.cxx
#AtClusterizeFastTask.cxx
AtPulseTask.cxx
AtSimulatedPoint.cxx
AtSimulatedElectron.cxx
AtTrigger.cxx
AtTriggerTask.cxx
)

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
