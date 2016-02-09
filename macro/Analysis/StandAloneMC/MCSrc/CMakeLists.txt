set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
${ATTPCROOTPATH}/include
${ATTPCROOTPATH}/reco
${ATTPCROOTPATH}/reco/GETDecoder
${ATTPCROOTPATH}/reco/GETDecoder2
${ATTPCROOTPATH}/reco/ATDecoder
${ATTPCROOTPATH}/reco/ATPulseAnalyzer
${ATTPCROOTPATH}/reco/HoughTransformation
${ATTPCROOTPATH}/reco/ATProtoPhiReco
${ATTPCROOTPATH}/reco/VMEDecoder
${ATTPCROOTPATH}/reco/ATMinimization
${ATTPCROOTPATH}/reco/ATAnalysis
# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/MCSrc
)

include_directories(${INCLUDE_DIRECTORIES} ${ATTPCROOTPATH}/include)

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}
${ATTPCROOTPATH}/build/lib
)

list(APPEND CMAKE_PREFIX_PATH $ENV{ROOTSYS})
link_directories(${LINK_DIRECTORIES})

set(SRCS
# Add all the source files below this line. Those must have cc for their extension.
MCSrc.cc
)

#CHANGE_FILE_EXTENSION(*.cc *.hh HEADERS "${SRCS}")

#Set(LINKDEF MCSrcLinkDef.h)
Set(LIBRARY_NAME MCSrc)
Set(DEPENDENCIES
    XMLParser
    Spectrum
    ATTPCReco
    Base
    MCStack
    AtTpcMap
    ATPar
)


set(MCSrc_HEADERS MCSrc.hh)
#set(MCSrc_LINKDEF MCSrcLinkDef.h)

#set(MCSrc_DICTIONARY G__MCSrc)
#ROOT_GENERATE_DICTIONARY("${MCSrc_HEADERS}" "${MCSrc_LINKDEF}" "${MCSrc_DICTIONARY}" "${INCLUDE_DIRECTORIES}")

ROOT_GENERATE_DICTIONARY(G__MCSrc ${MCSrc_HEADERS}  LINKDEF MCSrcLinkDef.h)

#add_library(MCSrc SHARED MCSrc.cc G__MCSrcDict.cxx)
GENERATE_LIBRARY()
target_link_libraries(MCSrc
Core
Gpad
Graf
Graf3d
Gui
Hist
Hist
MathCore
Matrix
Minuit
Net
Physics
Postscript
Rint
RIO
Thread
Tree
TreePlayer
ATTPCReco
AtTpcMap
FairTools
Base
)

#add_executable(MCExe MCSrc.cc)
#target_link_libraries(MCExe
#MCSrc
#Core
#Gpad
#Graf
#Graf3d
#Gui
#Hist
#Hist
#MathCore
#Matrix
#Minuit
#Net
#Physics
#Postscript
#Rint
#RIO
#Thread
#Tree
#TreePlayer
#ATTPCReco
#AtTpcMap
#FairTools
#Base
#)
