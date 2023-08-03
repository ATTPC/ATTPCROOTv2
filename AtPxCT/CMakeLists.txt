
 ################################################################################
 #    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    #
 #                                                                              #
 #              This software is distributed under the terms of the             #
 #         GNU Lesser General Public Licence version 3 (LGPL) version 3,        #
 #                  copied verbatim in the file "LICENSE"                       #
 ################################################################################
# Create a library called "libAtPxCT" which includes the source files given in
# the array .
# The extension is already found.  Any number of sources could be listed here
cmake_minimum_required(VERSION 3.13)
project(AtPxCT)
Set(LINKDEF AtPxCTLinkDef.h)
Set(LIBRARY_NAME AtPxCT)
Set(DEPENDENCIES
  FairRoot::Base
  ROOT::Core

  ATTPCROOT::AtSimulationData
)

set(SRCS
#Put here your sourcefiles
AtPxCT.cxx
AtPxCTContFact.cxx
AtPxCTCrystalCalData.cxx
AtPxCTDigitizer.cxx
)

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
