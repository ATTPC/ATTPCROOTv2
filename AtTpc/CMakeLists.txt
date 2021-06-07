 ################################################################################
 #    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    #
 #                                                                              #
 #              This software is distributed under the terms of the             # 
 #         GNU Lesser General Public Licence version 3 (LGPL) version 3,        #  
 #                  copied verbatim in the file "LICENSE"                       #
 ################################################################################
# Create a library called "libAtTpc" which includes the source files given in
# the array .
# The extension is already found.  Any number of sources could be listed here.
Set(LIBRARY_NAME AtTpc)
Set(LINKDEF AtTpcLinkDef.h)


set(INCLUDE_DIRECTORIES
#${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}

#put here all directories where header files are located
#${CMAKE_SOURCE_DIR}/AtData
#${CMAKE_SOURCE_DIR}/AtTpc
#${CMAKE_SOURCE_DIR}/AtGenerators

)
set(LINK_DIRECTORIES
${FAIRROOT_LIBRARY_DIR}
) 

set(SRCS
#Put here your sourcefiles
AtTpc.cxx
AtTpcContFact.cxx
AtTpcGeo.cxx
AtTpcGeoPar.cxx
AtTpcPoint.cxx
AtVertexPropagator.cxx

)

Set(DEPENDENCIES
  Base
  FairTools
  ROOT::Core
  Cling
  AtData
  )
  
generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

