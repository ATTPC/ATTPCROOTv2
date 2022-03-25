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

set(SRCS
#Put here your sourcefiles
AtTpc.cxx
AtTpcContFact.cxx
AtTpcGeo.cxx
AtTpcGeoPar.cxx
)

Set(DEPENDENCIES
  FairRoot::Base
  FairRoot::FairTools
  ROOT::Core
  #Cling
  AtSimulationData
  )

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )

