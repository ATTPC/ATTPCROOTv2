# Library to hold extentions to fairroot classes

Set(LIBRARY_NAME AtAnalysis)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)

set(INCLUDE_DIRECTORIES
# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/AtAnalysis/E12014
)

set(DEPENDENCIES
  FairRoot::Base
  FairRoot::FairTools

  ROOT::GenVector
  ROOT::Core

  ATTPCROOT::AtData
  ATTPCROOT::AtTools
  ATTPCROOT::AtMap

  )

set(SRCS
  AtRunAna.cxx
  
  E12014/AtFissionTask.cxx
  E12014/AtE12014.cxx
  )

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  )
