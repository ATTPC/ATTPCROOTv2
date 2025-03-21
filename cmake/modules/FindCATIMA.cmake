# - Find CATima installation
# This module tries to find CATima in your system



if(NOT DEFINED ENV{CATIMA} AND NOT DEFINED CATIMA)
  message(STATUS "Looking for CATima but did not set enviroment or cmake variable CATIMA")
endif()

if(DEFINED ENV{CATIMA})
  message(STATUS "Using enviroment variable CATIMA to search")
  Set(CATIMA $ENV{CATIMA})
endif()

set(CATIMA_LIBRARY_SEARCHPATH
  ${CATIMA}/lib
)

set(CATIMA_FOUND FALSE)

find_library(CATIMA_LIBRARY NAMES libcatima.so
  PATHS ${CATIMA_LIBRARY_SEARCHPATH}
  NO_DEFAULT_PATH
)

if(CATIMA_LIBRARY)

  message(STATUS "Looking for CATima... - found ${CATIMA}/lib")

  get_filename_component(CATIMA_LIBRARY_DIR ${CATIMA_LIBRARY} DIRECTORY)
  get_filename_component(CATIMA_INCLUDE_DIR ${CATIMA_LIBRARY}/../../include ABSOLUTE)

  message(STATUS "CATIMA_INCLUDE_DIR = ${CATIMA_INCLUDE_DIR}")


  mark_as_advanced(CATIMA_LIBRARY_DIR CATIMA_INCLUDE_DIR)

  set(CATIMA_FOUND TRUE)

  add_library(CATIMA::catima UNKNOWN IMPORTED GLOBAL)
  set_target_properties(CATIMA::catima PROPERTIES
    IMPORTED_LOCATION ${CATIMA_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${CATIMA_INCLUDE_DIR})

else(CATIMA_LIBRARY)

  if(CATIMA_FIND_REQUIRED)
    message(FATAL_ERROR "Looking for CATima... - Not found!")
  endif(CATIMA_FIND_REQUIRED)

endif(CATIMA_LIBRARY)
