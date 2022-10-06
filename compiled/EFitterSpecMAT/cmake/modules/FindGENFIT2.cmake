# - Find GENFIT2 instalation
# This module tries to find the GENFIT2 installation on your system.
#
# Variables defined by this module:
#
#   GENFIT2_FOUND               System has GENFIT2
#   GENFIT2_INCLUDE_DIR         GENFIT2 include directories: not cached
#   GENFIT2_LIBRARY_DIR         The path to where the GENFIT2 library files are.
#

Message(STATUS "Looking for GENFIT2...")
Message(STATUS $ENV{GENFIT})
Set(GENFIT $ENV{GENFIT})


Set(GENFIT2_LIBRARY_SEARCHPATH
  ${GENFIT}/lib
)

Message(STATUS ${GENFIT}/lib)

Set(GENFIT2_FOUND FALSE)

Find_Library(GENFIT2_LIBRARY NAMES genfit2
             PATHS ${GENFIT2_LIBRARY_SEARCHPATH}
             NO_DEFAULT_PATH
            )

Message(STATUS "Genfit2 ${GENFIT2_LIBRARY}")

If(GENFIT2_LIBRARY)

  MESSAGE(STATUS "Looking for GENFIT2... - found ${GENFIT}/lib")

  Set(GENFIT2_LIBRARY_DIR ${GENFIT}/lib)
  Set(GENFIT2_LDFLAGS "-L${GENFIT}/lib -lgenfit2")

  #MESSAGE(STATUS ${GENFIT2_LDFLAGS})

  Set(GENFIT2_INCLUDE_DIR ${GENFIT}/include)

  Mark_As_Advanced(GENFIT2_LIBRARY_DIR GENFIT2_INCLUDE_DIR)

  Set(LD_LIBRARY_PATH ${LD_LIBRARY_PATH} ${GENFIT2_LIBRARY_DIR})

  Set(GENFIT2_FOUND TRUE)

Else(GENFIT2_LIBRARY)

  If(GENFIT2_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Looking for GENFIT2... - Not found!")
  EndIf(GENFIT2_FIND_REQUIRED)

EndIf(GENFIT2_LIBRARY)
