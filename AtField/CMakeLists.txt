# Create a library called "libField” which includes the source files given in
# the array .
# The extension is already found.  Any number of sources could be listed here.

set(target AtField)
set(LINKDEF FieldLinkDef.h)

set(SRCS
AtConstField.cxx   
AtFieldCreator.cxx 
AtFieldPar.cxx
)

set(INCLUDE_DIRECTORIES
${BASE_INCLUDE_DIRECTORIES} 
)

set(LINK_DIRECTORIES
${FAIRROOT_LIBRARY_DIR}
)

Set(DEPENDENCIES
    Base 
    FairTools
    ROOT::Core
)


generate_target_root_library(${target}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

