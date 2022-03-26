 ################################################################################
 #    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    #
 #                                                                              #
 #              This software is distributed under the terms of the             # 
 #              GNU Lesser General Public Licence (LGPL) version 3,             #  
 #                  copied verbatim in the file "LICENSE"                       #
 ################################################################################
# Find FairRoot installation 
# Check the environment variable "FAIRROOTPATH"
# FairRoot does not (as of yet) export a FairRootConfig.cmake file for us to digest


# Set the FAIRROOTPATH variable to get location of install
if(FairRoot_DIR)
  set(FAIRROOTPATH ${FairRoot_DIR})
else()
  if(NOT DEFINED ENV{FAIRROOTPATH} AND NOT DEFINED FAIRROOTPATH)
    set(user_message "You did not define the environment or cmake variable FAIRROOTPATH which is needed to find FairRoot.\
         Please set this variable and execute cmake again." )
    if(FairRoot_FIND_REQUIRED)
      message(FATAL_ERROR ${user_message})
    else(FairRoot_FIND_REQUIRED)
      message(WARNING ${user_message})
      return()
    endif(FairRoot_FIND_REQUIRED)
  endif(NOT DEFINED ENV{FAIRROOTPATH} AND NOT DEFINED FAIRROOTPATH)
  if(DEFINED ENV{FAIRROOTPATH})
    message(STATUS "Using eviroment FAIRROOTPATH variable to find FairRoot")
    set(FAIRROOTPATH $ENV{FAIRROOTPATH})
  else()
    message(STATUS "Using CMake variable FAIRROOTPATH to find FairRoot")
  endif()
endif()

message(STATUS "Setting FairRoot environment:")

find_path(FAIRROOT_INCLUDE_DIR NAMES FairRun.h PATHS
  ${FAIRROOTPATH}/include
  NO_DEFAULT_PATH
)

find_path(FAIRROOT_LIBRARY_DIR NAMES libBase.so libBase.dylib PATHS
   ${FAIRROOTPATH}/lib
   ${FAIRROOTPATH}/lib64
  NO_DEFAULT_PATH
)

# look for exported FairMQ targets and include them
find_file(_fairroot_fairmq_cmake
    NAMES FairMQ.cmake
    HINTS ${FAIRROOTPATH}/include/cmake
)
if(_fairroot_fairmq_cmake)
    include(${_fairroot_fairmq_cmake})
endif()

if(FAIRROOT_INCLUDE_DIR AND FAIRROOT_LIBRARY_DIR)
   set(FairRoot_FOUND TRUE)
   message(STATUS "  FairRoot prefix            : ${FAIRROOTPATH}")
   message(STATUS "  FairRoot Library directory : ${FAIRROOT_LIBRARY_DIR}")
   message(STATUS "  FairRoot Include path      : ${FAIRROOT_INCLUDE_DIR}")

else(FAIRROOT_INCLUDE_DIR AND FAIRROOT_LIBRARY_DIR)
   set(FairRoot_FOUND FALSE)
   if(FairRoot_FIND_REQUIRED)
     message(FATAL_ERROR "FairRoot installation not found")
   endif()
endif (FAIRROOT_INCLUDE_DIR AND FAIRROOT_LIBRARY_DIR)

# Manually export targets
set(_fairroot_deps Base FairTools Alignment GeoBase ParBase Gen EventDisplay)
foreach(_fairroot_dep ${_fairroot_deps})
  find_library(${_fairroot_dep}_LIB ${_fairroot_dep} PATHS ${FAIRROOT_LIBRARY_DIR})
  if(${_fairroot_dep}_LIB)
    add_library(FairRoot::${_fairroot_dep} SHARED IMPORTED GLOBAL)
    set_target_properties(FairRoot::${_fairroot_dep} PROPERTIES IMPORTED_LOCATION ${${_fairroot_dep}_LIB})
    target_include_directories(FairRoot::${_fairroot_dep} INTERFACE ${FAIRROOT_INCLUDE_DIR})
    target_link_directories(FairRoot::${_fairroot_dep} INTERFACE ${SIMPATH}/lib)
    message(STATUS "FairRoot::${_fairroot_dep} (${${_fairroot_dep}_LIB}) target added by hand.")
  else()
    message(STATUS "Failed to add target FairRoot::${_fairroot_dep}")
  endif()
endforeach()

