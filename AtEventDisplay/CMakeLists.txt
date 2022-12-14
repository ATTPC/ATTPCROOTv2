# Create a library called "libSTEventDisplay" which includes the source files given in the array .

Set(LINKDEF AtEventDisplayLinkDef.h) # Change the linkdef name and delete this comment
Set(LIBRARY_NAME AtEventDisplay) # Change the library name and delete this comment

set(INCLUDE_DIRECTORIES
# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/AtEventDisplay/AtEventTab
  )

set(SRCS
# Add all the source files below this line. Those must have cc for their extension.

AtEventTab/AtEventTab.cxx
AtEventTab/AtEventTabMain.cxx

AtEventManager.cxx
AtEventManagerNew.cxx
AtEventManagerProto.cxx
AtEventDrawTask.cxx
AtEventDrawTaskNew.cxx
AtEventTabTaskBase.cxx
AtEventTabTask.cxx
AtEventDrawTaskProto.cxx
AtEventManagerS800.cxx
AtEventDrawTaskS800.cxx

  )

Set(DEPENDENCIES
  FairRoot::Base
  FairRoot::FairTools

  ROOT::Eve
  ROOT::Core
  ROOT::HistPainter

  ATTPCROOT::AtData
  ATTPCROOT::AtMap
  ATTPCROOT::AtS800
  ATTPCROOT::AtAnalysis
  )

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

set_source_files_properties(AtEventDrawTask.cxx PROPERTIES COMPILE_FLAGS -Wno-shadow)
set_source_files_properties(AtEventDrawTaskNew.cxx PROPERTIES COMPILE_FLAGS -Wno-shadow)
set_source_files_properties(AtEventDrawTaskProto.cxx PROPERTIES COMPILE_FLAGS -Wno-shadow)
set_source_files_properties(AtEventDrawTaskS800.cxx PROPERTIES COMPILE_FLAGS -Wno-shadow)
