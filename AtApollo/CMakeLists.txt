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
    Base Core Cling AtGenerators FairLogger::FairLogger
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
${ROOT_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
#put here all directories where header files are located
${CMAKE_SOURCE_DIR}/AtData
${CMAKE_SOURCE_DIR}/AtApollo
${CMAKE_SOURCE_DIR}/AtGenerators
)

include_directories( ${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}
)

link_directories( ${LINK_DIRECTORIES})



GENERATE_LIBRARY()
