# Create a library called "libAtDigitization" which includes the source files given in the array .

Set(LINKDEF AtDigiLinkDef.h)
Set(LIBRARY_NAME AtDigitization)
Set(DEPENDENCIES
  Base
  FairTools
  ROOT::Core

  AtData
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
# Add all the source files below this line. Those must have cc for their extension.
AtAvalancheTask.cxx
AtClusterizeTask.cxx
AtPulseTask.cxx
AtSimulatedPoint.cxx
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
