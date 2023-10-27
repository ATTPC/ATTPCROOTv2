# Create a library called "libPassive" which includes the source files given in
# the array .
# The extension is already found.  Any number of sources could be listed here.
Set(LINKDEF PassiveLinkDef.h)
Set(LIBRARY_NAME AtPassive)

set(SRCS
AtCave.cxx           
AtPipe.cxx 
AtGeoCave.cxx        
AtMagnet.cxx         
AtPassiveContFact.cxx
)

Set(DEPENDENCIES
  FairRoot::Base
  FairRoot::GeoBase
  FairRoot::ParBase
  FairLogger::FairLogger
  ROOT::Geom
  ROOT::Core
  )


generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
