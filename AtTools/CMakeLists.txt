set(INCLUDE_DIRECTORIES
${BASE_INCLUDE_DIRECTORIES}
${ROOT_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${CMAKE_SOURCE_DIR}
${CMAKE_SOURCE_DIR}/AtTools
)


include_directories(${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
  ${ROOT_LIBRARY_DIR}
  ${FAIRROOT_LIBRARY_DIR}
  ${Boost_LIBRARY_DIRS}
  )

link_directories( ${LINK_DIRECTORIES})

set(SRCS

  AtParsers.cxx
  AtELossManager.cxx

  )

Set(DEPENDENCIES
  Base
  FairTools
  XMLParser
  Core
  FairLogger::FairLogger
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
  ${Boost_THREAD_LIBRARY}
  ${Boost_DAtE_TIME_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)

Set(LIBRARY_NAME AtTools)
Set(LINKDEF AtToolsLinkDef.h)

GENERATE_LIBRARY()
