set(INCLUDE_DIRECTORIES
${ROOT_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
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

CHANGE_FILE_EXTENSION(*.cc *.hh HEADERS "${SRCS}")

Set(LINKDEF MCSrcLinkDef.h)
Set(LIBRARY_NAME MCSrc)
Set(DEPENDENCIES
    Cling
    XMLParser
# Add all library names your code depending on.
# Basically those are the libraries in the header files' directory.
    Spectrum
    ATTPCReco
)

ROOT_GENERATE_DICTIONARY(G__MCSrc MCSrc.hh LINKDEF MCSrcLinkDef.h)

GENERATE_LIBRARY()
target_link_libraries(MCSrc ${ROOT_LIBRARIES})

add_executable(MCExe MCSrc.cc)
target_link_libraries(MCExe MCSrc)
