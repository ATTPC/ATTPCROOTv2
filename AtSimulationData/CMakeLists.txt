# Create a library called "libAtSimulationData"

Set(LIBRARY_NAME AtSimulationData)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)

set(INCLUDE_DIRECTORIES
${FAIRROOT_INCLUDE_DIR}
)

set(DEPENDENCIES
  Base # FairRunIdGenerator
  FairTools # FairLogger

  ROOT::Core
  #ROOT::VMC # TMCProcess
  )

set(LINK_DIRECTORIES
  ${FAIRROOT_LIBRARY_DIR}
  )

set(SRCS
  AtStack.cxx
  AtMCTrack.cxx
  AtVertexPropagator.cxx
  AtMCPoint.cxx
)

message("****** ${FAIRROOT_INCLUDE_DIR}")

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

