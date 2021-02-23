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
${CMAKE_SOURCE_DIR}/AtS800
${CMAKE_SOURCE_DIR}/AtReconstruction/AtDecoder
)

include_directories(${INCLUDE_DIRECTORIES} )

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}

)

link_directories( ${LINK_DIRECTORIES})

set(SRCS
#Put here your sourcefiles
S800Calc.cxx
S800Event.cxx
S800.cxx
TInverseMap.cxx
AtS800Task.cxx
S800Calibration.cxx
lmmin.cxx
lmcurve.cxx
lmfit.cxx
S800Settings.cxx
S800TSettings.cxx
AtMergeTask.cxx

)

Set(LINKDEF S800LinkDef.h)
Set(LIBRARY_NAME AtS800)
Set(DEPENDENCIES
    Base
    FairLogger::FairLogger
# Add all library names your code depending on.
# Basically those are the libraries in the header files' directory.
  TreePlayer
)


GENERATE_LIBRARY()
