# Create a library called "libAtPar" which includes the source files given in the array .

set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${BASE_INCLUDE_DIRECTORIES}
#${CMAKE_SOURCE_DIR}/fairtools
#${CMAKE_SOURCE_DIR}/parbase
#${CMAKE_SOURCE_DIR}/geobase
# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/AtParameter
)

include_directories(${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}
)

link_directories( ${LINK_DIRECTORIES})

set(SRCS
# Add all the source files below this line. Those must have cc for their extension.
AtContFact.cxx
#STGeoPar.cc
AtDigiPar.cxx
AtGas.cxx
AtTriggerPar.cxx
)


Set(LINKDEF AtParLinkDef.h)
Set(LIBRARY_NAME AtParameter)
Set(DEPENDENCIES
    Base
    #MCStack
    FairLogger::FairLogger
# Add all library names your code depending on.
# Basically those are the libraries in the header files' directory.
)

GENERAtE_LIBRARY()
