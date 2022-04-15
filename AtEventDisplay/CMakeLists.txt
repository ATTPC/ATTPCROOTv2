# Create a library called "libSTEventDisplay" which includes the source files given in the array .

Set(LINKDEF AtEventDisplayLinkDef.h) # Change the linkdef name and delete this comment
Set(LIBRARY_NAME AtEventDisplay) # Change the library name and delete this comment

set(SRCS
# Add all the source files below this line. Those must have cc for their extension.
AtEventManager.cxx
AtEventManagerProto.cxx
AtEventDrawTask.cxx
AtEventDrawTaskProto.cxx
AtEventManagerS800.cxx
AtEventDrawTaskS800.cxx

)

Set(DEPENDENCIES
  FairRoot::Base
  FairRoot::FairTools
  FairRoot::EventDisplay
  ROOT::Eve
  ROOT::Core
  ROOT::HistPainter
  
  ATTPCROOT::AtReconstruction
  ATTPCROOT::AtData
  ATTPCROOT::AtMap
  ATTPCROOT::AtS800
  ATTPCROOT::AtParameter
  )
  
generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

