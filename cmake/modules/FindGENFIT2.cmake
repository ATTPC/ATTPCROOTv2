# - Find GENFIT2 instalation
# This module tries to find the GENFIT2 installation on your system.
#
# Variables defined by this module:
#
#   GENFIT2_FOUND               System has GENFIT2
#   GENFIT2_INCLUDE_DIR         GENFIT2 include directories: not cached
#   GENFIT2_LIBRARY_DIR         The path to where the GENFIT2 library files are.
#

if(NOT DEFINED ENV{GENFIT} AND NOT DEFINED GENFIT)
  message(STATUS "Looking for GENFIT but did not set enviroment or cmake variable GENFIT")
endif()

if(DEFINED ENV{GENFIT})
  message(STATUS "Using enviroment variable GENFIT to search")
  Set(GENFIT $ENV{GENFIT})
endif()


Set(GENFIT2_LIBRARY_SEARCHPATH
  ${GENFIT}/lib
)


Set(GENFIT2_FOUND FALSE)

Find_Library(GENFIT2_LIBRARY NAMES genfit2
  PATHS ${GENFIT2_LIBRARY_SEARCHPATH}
  NO_DEFAULT_PATH
  )

If(GENFIT2_LIBRARY)

  MESSAGE(STATUS "Looking for GENFIT2... - found ${GENFIT}/lib")

  get_filename_component(GENFIT2_LIBRARY_DIR ${GENFIT2_LIBRARY} DIRECTORY)
  get_filename_component(GENFIT2_INCLUDE_DIR ${GENFIT2_LIBRARY_DIR}/../include ABSOLUTE)

  Mark_As_Advanced(GENFIT2_LIBRARY_DIR GENFIT2_INCLUDE_DIR)

  Set(GENFIT2_FOUND TRUE)

  add_library(GENFIT2::genfit2 UNKNOWN IMPORTED GLOBAL)
  set_target_properties(GENFIT2::genfit2 PROPERTIES
    IMPORTED_LOCATION ${GENFIT2_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${GENFIT2_INCLUDE_DIR})
  target_link_libraries(GENFIT2::genfit2 INTERFACE
    ROOT::Eve
    ROOT::EG
    ROOT::Geom
    ROOT::Ged
    ROOT::RGL
    ROOT::Gui
    ROOT::Core
    ROOT::Imt
    ROOT::RIO
    ROOT::Net
    ROOT::Hist
    ROOT::Graf
    ROOT::ROOTVecOps
    ROOT::Tree
    ROOT::TreePlayer
    ROOT::Rint
    ROOT::Postscript
    ROOT::Matrix
    ROOT::Physics
    ROOT::MathCore
    ROOT::Thread
    ROOT::MultiProc
    ROOT::ROOTDataFrame
    )
  
Else(GENFIT2_LIBRARY)

  If(GENFIT2_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Looking for GENFIT2... - Not found!")
  EndIf(GENFIT2_FIND_REQUIRED)

EndIf(GENFIT2_LIBRARY)
