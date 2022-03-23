# Library to hold extentions to fairroot classes

Set(LIBRARY_NAME AtAnalysis)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)

set(LINK_DIRECTORIES
  ${FAIRROOT_LIBRARY_DIR}
  )

set(INCLUDE_DIRECTORIES
  ${FAIRROOT_INCLUDE_DIR}
  )

set(DEPENDENCIES
  Base
  FairTools
  ROOT::Core
  #ROOT::VMC
  )

set(SRCS
  AtRunAna.cxx
  )

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

