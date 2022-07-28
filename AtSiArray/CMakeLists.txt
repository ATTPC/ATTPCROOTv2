Set(LIBRARY_NAME AtSiArray)
Set(LINKDEF AtSiArrayLinkDef.h)


set(SRCS
AtSiArrayContFact.cxx
AtSiArrayGeo.cxx
AtSiArrayGeoPar.cxx
AtSiArray.cxx
AtSiPoint.cxx
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
