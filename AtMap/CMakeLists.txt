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
Set(LIBRARY_NAME AtMap)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)

set(SRCS
#Put here your sourcefiles
AtMap.cxx
AtTpcMap.cxx
AtSiMap.cxx
AtGAGGMap.cxx
AtTpcProtoMap.cxx
AtGadgetIIMap.cxx
AtSpecMATMap.cxx
)

Set(DEPENDENCIES
  FairRoot::FairTools
  ROOT::XMLParser
  ROOT::Core

  Boost::headers

  ATTPCROOT::AtData
  )

set(TEST_SRCS
  AtTpcMapTest.cxx
)

attpcroot_generate_tests(${LIBRARY_NAME}Tests
  SRCS ${TEST_SRCS}
  DEPS ${LIBRARY_NAME}
)

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
