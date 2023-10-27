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

Set(DEPENDENCIES
  FairRoot::Base 
  ROOT::Core
  FairLogger::FairLogger
)


generate_target_and_root_library(${target}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

