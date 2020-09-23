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

set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${BASE_INCLUDE_DIRECTORIES}
#put here all directories where header files are located
${CMAKE_SOURCE_DIR}/s800
${CMAKE_SOURCE_DIR}/reco/ATDecoder
)

include_directories(${INCLUDE_DIRECTORIES} )

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}

)

link_directories( ${LINK_DIRECTORIES})

set(SRCS
#Put here your sourcefiles
S800Calc.cc
S800Event.cc
S800.cc
TInverseMap.cc
ATS800Task.cc
S800Calibration.cc
lmmin.cc
lmcurve.cc
lmfit.cc
S800Settings.cc
S800TSettings.cc
ATMergeTask.cc

)

CHANGE_FILE_EXTENSION(*.cc *.hh HEADERS "${SRCS}")

Set(LINKDEF S800LinkDef.h)
Set(LIBRARY_NAME S800)
Set(DEPENDENCIES
    Base
    FairLogger::FairLogger
# Add all library names your code depending on.
# Basically those are the libraries in the header files' directory.
  TreePlayer
)


GENERATE_LIBRARY()
