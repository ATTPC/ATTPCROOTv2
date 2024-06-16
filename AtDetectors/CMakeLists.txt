Set(LIBRARY_NAME AtDetectors)
Set(LINKDEF 
AtTpc/AtTpcLinkDef.h
AtPxCT/AtPxCTLinkDef.h
AtSiArray/AtSiArrayLinkDef.h
AtSeGA/AtSeGALinkDef.h
AtDeGAi/AtDeGAiLinkDef.h
AtApollo/AtApolloLinkDef.h
AtField/AtFieldLinkDef.h
AtPassive/AtPassiveLinkDef.h
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

AtSiArray/AtSiArrayContFact.cxx
AtSiArray/AtSiArrayGeo.cxx
AtSiArray/AtSiArrayGeoPar.cxx
AtSiArray/AtSiArray.cxx
AtSiArray/AtSiPoint.cxx

AtSeGA/AtSeGA.cxx
AtSeGA/AtSeGAContFact.cxx
AtSeGA/AtSeGACrystalCalData.cxx
AtSeGA/AtSeGADigitizer.cxx

AtDeGAi/AtDeGAi.cxx
AtDeGAi/AtDeGAiContFact.cxx
AtDeGAi/AtDeGAiCrystalCalData.cxx
AtDeGAi/AtDeGAiDigitizer.cxx

AtApollo/AtApollo.cxx
AtApollo/AtApolloContFact.cxx
AtApollo/AtApolloPoint.cxx
AtApollo/AtApolloCrystalCalData.cxx
AtApollo/AtApolloDigitizer.cxx

AtField/AtConstField.cxx   
AtField/AtFieldCreator.cxx 
AtField/AtFieldPar.cxx

AtPassive/AtCave.cxx           
AtPassive/AtPipe.cxx 
AtPassive/AtGeoCave.cxx        
AtPassive/AtMagnet.cxx         
AtPassive/AtPassiveContFact.cxx
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
