# Create a library called "libSTEventDisplay" which includes the source files given in the array .

Set(LINKDEF AtEventDisplayLinkDef.h) # Change the linkdef name and delete this comment
Set(LIBRARY_NAME AtEventDisplay) # Change the library name and delete this comment

set(INCLUDE_DIRECTORIES
# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/AtEventDisplay/Deprecated
${CMAKE_SOURCE_DIR}/AtEventDisplay/AtTabs
${CMAKE_SOURCE_DIR}/AtEventDisplay/AtSidebar
${CMAKE_SOURCE_DIR}/AtEventDisplay/AtSidebar/PSA
)

set(SRCS
# Add all the source files below this line. Those must have cc for their extension.
AtViewerManager.cxx
AtViewerManagerSubject.cxx

AtTabs/AtTabBase.cxx
AtTabs/AtTabCanvas.cxx
AtTabs/AtTabInfo.cxx
AtTabs/AtTabMacro.cxx
AtTabs/AtTabMain.cxx
AtTabs/AtTabPad.cxx
AtTabs/AtTabInfoTree.cxx

AtSidebar/AtEventSidebar.cxx
AtSidebar/AtSidebarFrames.cxx
AtSidebar/AtSidebarAddon.cxx
AtSidebar/AtSidebarInfoMacro.cxx
AtSidebar/PSA/AtSidebarPSA.cxx
AtSidebar/PSA/AtSidebarPSADeconv.cxx
AtSidebar/PSA/AtSidebarPSAIterDeconv.cxx

Deprecated/AtEventManager.cxx
Deprecated/AtEventManagerProto.cxx
Deprecated/AtEventDrawTask.cxx
Deprecated/AtEventDrawTaskProto.cxx
Deprecated/AtEventManagerS800.cxx
Deprecated/AtEventDrawTaskS800.cxx
)

Set(DEPENDENCIES
  FairRoot::Base
  FairRoot::FairTools

  ROOT::Eve
  ROOT::Core
  ROOT::HistPainter

  ATTPCROOT::AtData
  ATTPCROOT::AtMap
  ATTPCROOT::AtReconstruction
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
#set_source_files_properties(AtViewerManager.cxx PROPERTIES COMPILE_FLAGS -Wno-shadow)
set_source_files_properties(AtEventDrawTaskProto.cxx PROPERTIES COMPILE_FLAGS -Wno-shadow)
set_source_files_properties(AtEventDrawTaskS800.cxx PROPERTIES COMPILE_FLAGS -Wno-shadow)
#set_source_files_properties(AtTabMain.cxx PROPERTIES COMPILE_FLAGS -Wno-shadow)
