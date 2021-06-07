# Create a library called "libAtPar" which includes the source files given in the array .

Set(LINKDEF AtParLinkDef.h)
Set(LIBRARY_NAME AtParameter)

set(INCLUDE_DIRECTORIES
${BASE_INCLUDE_DIRECTORIES}
)
set(LINK_DIRECTORIES
${FAIRROOT_LIBRARY_DIR}
)
set(SRCS
# Add all the source files below this line. Those must have cc for their extension.
AtContFact.cxx
#STGeoPar.cc
AtDigiPar.cxx
AtGas.cxx
AtTriggerPar.cxx
)


Set(DEPENDENCIES
    Base
    FairTools
    ROOT::Core
)

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
