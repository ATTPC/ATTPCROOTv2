# Library to hold extentions to fairroot classes

Set(LIBRARY_NAME AtAnalysis)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)

set(DEPENDENCIES
  FairRoot::Base
  FairRoot::FairTools
  ROOT::Core

  ROOT::VMC
  )

set(SRCS
  AtRunAna.cxx
  )

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

