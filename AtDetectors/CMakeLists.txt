Set(LIBRARY_NAME AtDetectors)
Set(LINKDEF 
AtTpc/AtTpcLinkDef.h
AtPxCT/AtPxCTLinkDef.h
)

set(SRCS
#Put here your sourcefiles
AtTpc/AtTpc.cxx
AtTpc/AtTpcContFact.cxx
AtTpc/AtTpcGeo.cxx
AtTpc/AtTpcGeoPar.cxx

AtPxCT/AtPxCT.cxx
AtPxCT/AtPxCTContFact.cxx
AtPxCT/AtPxCTCrystalCalData.cxx
AtPxCT/AtPxCTDigitizer.cxx

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
