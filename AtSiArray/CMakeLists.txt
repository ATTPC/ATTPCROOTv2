set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
#put here all directories where header files are located
${CMAKE_SOURCE_DIR}/AtData
${CMAKE_SOURCE_DIR}/AtSiArray
${CMAKE_SOURCE_DIR}/AtGenerators

)

include_directories( ${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}

) 

link_directories( ${LINK_DIRECTORIES})

set(SRCS
AtSiArrayPoint.cxx
AtSiArrayContFact.cxx
AtSiArrayGeo.cxx
AtSiArrayGeoPar.cxx
AtSiArray.cxx
)

Set(LINKDEF AtSiArrayLinkDef.h)
Set(LIBRARY_NAME AtSiArray)
Set(DEPENDENCIES
    Base Core Cling AtGen FairLogger::FairLogger
)

GENERATE_LIBRARY()
