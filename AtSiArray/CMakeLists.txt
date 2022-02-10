Set(LIBRARY_NAME AtSiArray)
Set(LINKDEF AtSiArrayLinkDef.h)


set(INCLUDE_DIRECTORIES
#${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
#put here all directories where header files are located
#${CMAKE_SOURCE_DIR}/AtData
#${CMAKE_SOURCE_DIR}/AtSiArray
#${CMAKE_SOURCE_DIR}/AtGenerators
)

set(LINK_DIRECTORIES
${FAIRROOT_LIBRARY_DIR}
)

set(SRCS
AtSiArrayContFact.cxx
AtSiArrayGeo.cxx
AtSiArrayGeoPar.cxx
AtSiArray.cxx
AtSiPoint.cxx
)

Set(DEPENDENCIES
  Base
  FairTools
  ROOT::Core
  AtData
)

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
