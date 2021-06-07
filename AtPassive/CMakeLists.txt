# Create a library called "libPassive" which includes the source files given in
# the array .
# The extension is already found.  Any number of sources could be listed here.
Set(LINKDEF PassiveLinkDef.h)
Set(LIBRARY_NAME AtPassive)

set(INCLUDE_DIRECTORIES
${BASE_INCLUDE_DIRECTORIES}
)
set(LINK_DIRECTORIES
${FAIRROOT_LIBRARY_DIR}
)
set(SRCS
AtCave.cxx           
AtPipe.cxx 
AtGeoCave.cxx        
AtMagnet.cxx         
AtPassiveContFact.cxx
)

Set(DEPENDENCIES
  Base
  FairTools
  GeoBase
  ParBase
  Geom
  ROOT::Core
  )


generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
