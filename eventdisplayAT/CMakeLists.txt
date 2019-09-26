# Create a library called "libSTEventDisplay" which includes the source files given in the array .

set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
${PCL_INCLUDE_DIRS}
#${CMAKE_SOURCE_DIR}/fairtools
#${CMAKE_SOURCE_DIR}/parbase
#${CMAKE_SOURCE_DIR}/geobase
#${CMAKE_SOURCE_DIR}/eventdisplay
# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/AtMap
${CMAKE_SOURCE_DIR}/eventdisplayAT
${CMAKE_SOURCE_DIR}/reco/ATDecoder
${CMAKE_SOURCE_DIR}/reco/HoughTransformation
${CMAKE_SOURCE_DIR}/reco/ATRansac
${CMAKE_SOURCE_DIR}/reco/ATPatternRecognition
${CMAKE_SOURCE_DIR}/par

)

include_directories(${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}
)

link_directories(${LINK_DIRECTORIES})

set(SRCS
# Add all the source files below this line. Those must have cc for their extension.
ATEventManager.cc
ATEventManagerProto.cc
ATEventDrawTask.cc
ATEventDrawTaskProto.cc
ATEventProtoDrawTask.cc
)

CHANGE_FILE_EXTENSION(*.cc *.hh HEADERS "${SRCS}")

Set(LINKDEF ATEventDisplayLinkDef.h) # Change the linkdef name and delete this comment
Set(LIBRARY_NAME ATEventDisplay) # Change the library name and delete this comment
Set(DEPENDENCIES
    Base
    #MCStack
    EventDisplay
# ROOT Eve package
    Eve
# Add all library names your code depending on.
# Basically those are the libraries in the header files' directory.
    ATTPCReco
    AtTpcMap
    FairLogger::FairLogger
)

GENERATE_LIBRARY()
