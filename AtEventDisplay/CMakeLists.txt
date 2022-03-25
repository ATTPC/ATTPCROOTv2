# Create a library called "libSTEventDisplay" which includes the source files given in the array .

Set(LINKDEF AtEventDisplayLinkDef.h) # Change the linkdef name and delete this comment
Set(LIBRARY_NAME AtEventDisplay) # Change the library name and delete this comment

#set(LINK_DIRECTORIES
#${HIRAEVT_SOURCE_DIR}/build/lib
#)
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
  #EventDisplay
  ROOT::Eve

  AtReconstruction
  AtData
  AtMap
  AtS800
  AtParameter
  )
  
generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

