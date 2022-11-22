 ################################################################################
 #    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    #
 #                                                                              #
 #              This software is distributed under the terms of the             #
 #         GNU Lesser General Public Licence version 3 (LGPL) version 3,        #
 #                  copied verbatim in the file "LICENSE"                       #
 ################################################################################
# Create a library called "libS800" which includes the source files given in
# the array .
# The extension is already found.  Any number of sources could be listed here.

Set(LINKDEF S800LinkDef.h)
Set(LIBRARY_NAME AtS800)

set(SRCS
#Put here your sourcefiles
S800Ana.cxx
S800Calc.cxx
S800Event.cxx
S800.cxx
TInverseMap.cxx
AtS800Task.cxx
S800Calibration.cxx
S800Settings.cxx
S800TSettings.cxx
AtMergeTask.cxx

lmfit/lmmin.cxx
lmfit/lmcurve.cxx
lmfit/lmfit.cxx
)

Set(DEPENDENCIES
    FairRoot::Base
    FairRoot::FairTools
    ROOT::Core

    ATTPCROOT::AtData
)

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  INCLUDE_DIR ${CMAKE_SOURCE_DIR}/AtS800/lmfit
  )
