Set(LIBRARY_NAME AtGeArray)
Set(LINKDEF AtGeArrayLinkDef.h)


set(SRCS
AtGeArrayContFact.cxx
AtGeArrayGeo.cxx
AtGeArrayGeoPar.cxx
AtGeArray.cxx
AtGePoint.cxx
)

Set(DEPENDENCIES
  FairRoot::Base
  ROOT::Core
  ATTPCROOT::AtSimulationData
)

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
