 # Create a library called "libAtData" 

set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
${CMAKE_SOURCE_DIR}/AtData
)

include_directories( ${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}
${CMAKE_SOURCE_DIR}/AtData

)
 
link_directories( ${LINK_DIRECTORIES})

set(SRCS
AtStack.cxx 
AtMCTrack.cxx
)

Set(HEADERS )
Set(LINKDEF MCStackLinkDef.h)
Set(LIBRARY_NAME AtData)
Set(DEPENDENCIES Base EG Physics Cling Core )

GENERATE_LIBRARY()

