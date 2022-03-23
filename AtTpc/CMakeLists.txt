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
${FAIRROOT_INCLUDE_DIR}
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
)

Set(DEPENDENCIES
  Base
  FairTools
  ROOT::Core
  #Cling
  AtSimulationData
  )

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

