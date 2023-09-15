# Create a library called "libAtPar" which includes the source files given in the array .

Set(LINKDEF AtParLinkDef.h)
Set(LIBRARY_NAME AtParameter)

set(SRCS
# Add all the source files below this line. Those must have cc for their extension.
AtContFact.cxx
#STGeoPar.cc
AtDigiPar.cxx
AtGas.cxx
AtTriggerPar.cxx
)


Set(DEPENDENCIES
  ROOT::Core
  FairRoot::ParBase
  FairLogger::FairLogger
)

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
