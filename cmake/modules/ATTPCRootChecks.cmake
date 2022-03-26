###############################################################################
#              This software is distributed under the terms of the             #
#              GNU Lesser General Public Licence (LGPL) version 3,             #
#                  copied verbatim in the file "LICENSE"                       #
# Based on code from FairRoot
################################################################################

function(attpcroot_check_out_of_source_build)
  if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "In-source builds not supported!")
  endif()
endfunction()

function(attpcroot_check_install_prefix)
  if(CMAKE_INSTALL_PREFIX STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Cannot install into the build directory!")
  endif()
endfunction()

function(attpcroot_check_platform)
  if(NOT UNIX)
    message(FATAL_ERROR "You're not on an UNIX system. The project was up to now only tested on UNIX systems, so we break here. If you want to go on please edit the CMakeLists.txt in the source directory.")
  endif()
endfunction()

function(attpcroot_check_enviroment)

  if(NOT DEFINED ENV{FAIRROOTPATH} AND NOT DEFINED FAIRROOTPATH)
    message(FATAL_ERROR "You did not define the environment or cmake variable FAIRROOTPATH which is needed to find FairRoot. Please set this variable and execute cmake again.")
  endif()
  set(FAIRROOTPATH "$ENV{FAIRROOTPATH}" CACHE INTERNAL "FAIRROOTPATH")
  
  if(NOT DEFINED ENV{SIMPATH} AND NOT DEFINED SIMPATH)
    message(FATAL_ERROR "You did not define the environment or cmake variable SIMPATH which is nedded to find the external packages. Please set this variable and execute cmake again.")
  endif()
  set(SIMPATH "$ENV{SIMPATH}" CACHE INTERNAL "SIMPATH")
  
endfunction()

function(attpcroot_sanity_checks)
  attpcroot_check_out_of_source_build()
  attpcroot_check_install_prefix()
  attpcroot_check_platform()
  attpcroot_check_enviroment()
endfunction()
