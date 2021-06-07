 ################################################################################
 #    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    #
 #                                                                              #
 #              This software is distributed under the terms of the             #
 #         GNU Lesser General Public Licence version 3 (LGPL) version 3,        #
 #                  copied verbatim in the file "LICENSE"                       #
 ################################################################################
# Create a library called "libAtApollo" which includes the source files given in
# the array .
# The extension is already found.  Any number of sources could be listed here

Set(LINKDEF AtApolloLinkDef.h)
Set(LIBRARY_NAME AtApollo)
Set(DEPENDENCIES
  Base
  FairTools
  ROOT::Core
  Cling
  AtGenerators
  AtData
)

set(SRCS
#Put here your sourcefiles
AtApollo.cxx
AtApolloContFact.cxx
AtApolloPoint.cxx
AtApolloCrystalCalData.cxx
AtApolloDigitizer.cxx
)
set(INCLUDE_DIRECTORIES
${BASE_INCLUDE_DIRECTORIES}
)
set(LINK_DIRECTORIES
${FAIRROOT_LIBRARY_DIR}
)

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
