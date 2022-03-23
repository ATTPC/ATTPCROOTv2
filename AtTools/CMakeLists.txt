Set(LIBRARY_NAME AtTools)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)

set(SRCS
  AtParsers.cxx
  AtELossManager.cxx
  AtCSVReader.h
  AtEulerTransformation.cxx
  )


set(INCLUDE_DIRECTORIES
  ${FAIRROOT_INCLUDE_DIR}
)

set(LINK_DIRECTORIES
  ${FAIRROOT_LIBRARY_DIR}
)

Set(DEPENDENCIES
  Base
  FairTools
  ROOT::XMLParser
  ROOT::Core
)

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

