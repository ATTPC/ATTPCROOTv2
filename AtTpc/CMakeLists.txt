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

set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
#put here all directories where header files are located
${CMAKE_SOURCE_DIR}/AtData
${CMAKE_SOURCE_DIR}/AtTpc
${CMAKE_SOURCE_DIR}/AtGenerators

)

include_directories( ${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}

) 


link_directories( ${LINK_DIRECTORIES})

set(SRCS
#Put here your sourcefiles
AtTpc.cxx
AtTpcContFact.cxx
AtTpcGeo.cxx
AtTpcGeoPar.cxx
AtTpcPoint.cxx
)

Set(LINKDEF AtTpcLinkDef.h)
Set(LIBRARY_NAME AtTpc)
Set(DEPENDENCIES
    Base Core Cling AtGen FairLogger::FairLogger
)

GENERAtE_LIBRARY()
