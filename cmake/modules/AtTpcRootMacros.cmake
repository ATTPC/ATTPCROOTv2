################################################################################
#    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    #
#                                                                              #
#              This software is distributed under the terms of the             #
#              GNU Lesser General Public Licence (LGPL) version 3,             #
#                  copied verbatim in the file "LICENSE"                       #
################################################################################
# Functions for ATTPCROOT code. Based on FairMacros.cmake from FairRoot.
if(NOT WIN32 AND NOT DISABLE_COLOR)
  string(ASCII 27 Esc)
  set(CR       "${Esc}[m")
  set(CB       "${Esc}[1m")
  set(Red      "${Esc}[31m")
  set(Green    "${Esc}[32m")
  set(Yellow   "${Esc}[33m")
  set(Blue     "${Esc}[34m")
  set(Magenta  "${Esc}[35m")
  set(Cyan     "${Esc}[36m")
  set(White    "${Esc}[37m")
  set(BRed     "${Esc}[1;31m")
  set(BGreen   "${Esc}[1;32m")
  set(BYellow  "${Esc}[1;33m")
  set(BBlue    "${Esc}[1;34m")
  set(BMagenta "${Esc}[1;35m")
  set(BCyan    "${Esc}[1;36m")
  set(BWhite   "${Esc}[1;37m")
endif()

# Must be a macro so the variables set are in the scope of the caller (unlike a function)
macro(check_and_set_enviroment)
  # Check for needed environment variables
  if(NOT DEFINED ENV{FAIRROOTPATH})
    message(FATAL_ERROR "You did not define the environment variable FAIRROOTPATH which is needed to find FairRoot. Please set this variable and execute cmake again.")
  endif(NOT DEFINED ENV{FAIRROOTPATH})
  
  if(NOT DEFINED ENV{SIMPATH})
    message(FATAL_ERROR "You did not define the environment variable SIMPATH which is nedded to find the external packages. Please set this variable and execute cmake again.")
  endif(NOT DEFINED ENV{SIMPATH})

  if(NOT UNIX)
    message(FATAL_ERROR "You're not on an UNIX system. The project was up to now only tested on UNIX systems, so we break here. If you want to go on please edit the CMakeLists.txt in the source directory.")
  endif(NOT UNIX)

set(SIMPATH $ENV{SIMPATH})
set(FAIRROOTPATH $ENV{FAIRROOTPATH})
set(CMAKE_PREFIX_PATH ${SIMPATH} ${CMAKE_PREFIX_PATH})
set(LD_LIBRARY_PATH ${LD_LIBRARY_PATH} ${SIMPATH}/lib)
#set(LD_LIBRARY_PATH ${LD_LIBRARY_PATH} ${FAIRROOTPATH}/lib)
Set(VMCWORKDIR ${CMAKE_SOURCE_DIR})

endmacro()
  
macro (check_out_of_source_build)
  if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "${PROJECT_NAME} should be installed as an out of source build, to keep the source directory clean. Please create a extra build directory and run the command 'cmake path_to_source_dir' in this newly created directory. You have also to delete the directory CMakeFiles and the file CMakeCache.txt in the source directory. Otherwise cmake will complain even if you run it from an out-of-source directory.")
  endif()
endmacro()

#################################################################################
# The macro checks if the build directory is different from the
# installation directory. In case both are the same
# stop the execution of cmake with an error message.
#
################################################################################

macro (check_install_directory)
  if(CMAKE_INSTALL_PREFIX STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Cannot install into the build directory!")
  endif()
endmacro ()

################################################################################

################################################################################
#
# find_package2(PRIVATE|PUBLIC|INTERFACE <pkgname>
#               [VERSION <version>]
#               [COMPONENTS <list of components>]
#               [ADD_REQUIREMENTS_OF <list of dep_pgkname>]
#               [any other option the native find_package supports]...)
#
# Wrapper around CMake's native find_package command to add some features and bookkeeping.
#
# The qualifier (PRIVATE|PUBLIC|INTERFACE) to the package to populate
# the variables PROJECT_[INTERFACE]_<pkgname>_([VERSION]|[COMPONENTS]|PACKAGE_DEPENDENCIES)
# accordingly. This bookkeeping information is used to print our dependency found summary
# table and to generate a part of our CMake package.
#
# When a dependending package is listed with ADD_REQUIREMENTS_OF the variables
# <dep_pkgname>_<pkgname>_VERSION|COMPONENTS are looked up to and added to the native
# VERSION (selected highest version) and COMPONENTS (deduplicated) args.
#
# COMPONENTS and VERSION args are then just passed to the native find_package.
#
macro(find_package2 qualifier pkgname)
  cmake_parse_arguments(ARGS "" "VERSION" "COMPONENTS;ADD_REQUIREMENTS_OF" ${ARGN})

  string(TOUPPER ${pkgname} pkgname_upper)
  set(__old_cpp__ ${CMAKE_PREFIX_PATH})
  set(CMAKE_PREFIX_PATH ${${pkgname_upper}_ROOT} $ENV{${pkgname_upper}_ROOT} ${CMAKE_PREFIX_PATH})

  # build lists of required versions and components
  unset(__required_versions__)
  unset(__components__)
  if(ARGS_VERSION)
    list(APPEND __required_versions__ ${ARGS_VERSION})
  endif()
  if(ARGS_COMPONENTS)
    list(APPEND __components__ ${ARGS_COMPONENTS})
  endif()
  if(ARGS_ADD_REQUIREMENTS_OF)
    foreach(dep_pkgname IN LISTS ARGS_ADD_REQUIREMENTS_OF)
      if(${dep_pkgname}_${pkgname}_VERSION)
        list(APPEND __required_versions__ ${${dep_pkgname}_${pkgname}_VERSION})
      endif()
      if(${dep_pkgname}_${pkgname}_COMPONENTS)
        list(APPEND __components__ ${${dep_pkgname}_${pkgname}_COMPONENTS})
      endif()
    endforeach()
  endif()

  # select highest required version
  unset(__version__)
  if(__required_versions__)
    list(GET __required_versions__ 0 __version__)
    foreach(v IN LISTS __required_versions__)
      if(${v} VERSION_GREATER ${__version__})
        set(__version__ ${v})
      endif()
    endforeach()
  endif()
  # deduplicate required component list
  if(__components__)
    list(REMOVE_DUPLICATES __components__)
  endif()

  # call native find_package
  #message("Looking in ${CMAKE_PREFIX_PATH} for ${pkgname}")
  if(__components__)
    find_package(${pkgname} ${__version__} QUIET COMPONENTS ${__components__} ${ARGS_UNPARSED_ARGUMENTS})
  else()
    find_package(${pkgname} ${__version__} QUIET ${ARGS_UNPARSED_ARGUMENTS})
  endif()

  if(${pkgname}_FOUND)
    if(${qualifier} STREQUAL PRIVATE)
      set(PROJECT_${pkgname}_VERSION ${__version__})
      set(PROJECT_${pkgname}_COMPONENTS ${__components__})
      set(PROJECT_PACKAGE_DEPENDENCIES ${PROJECT_PACKAGE_DEPENDENCIES} ${pkgname})
    elseif(${qualifier} STREQUAL PUBLIC)
      set(PROJECT_${pkgname}_VERSION ${__version__})
      set(PROJECT_${pkgname}_COMPONENTS ${__components__})
      set(PROJECT_PACKAGE_DEPENDENCIES ${PROJECT_PACKAGE_DEPENDENCIES} ${pkgname})
      set(PROJECT_INTERFACE_${pkgname}_VERSION ${__version__})
      set(PROJECT_INTERFACE_${pkgname}_COMPONENTS ${__components__})
      set(PROJECT_INTERFACE_PACKAGE_DEPENDENCIES ${PROJECT_INTERFACE_PACKAGE_DEPENDENCIES} ${pkgname})
    elseif(${qualifier} STREQUAL INTERFACE)
      set(PROJECT_INTERFACE_${pkgname}_VERSION ${__version__})
      set(PROJECT_INTERFACE_${pkgname}_COMPONENTS ${__components__})
      set(PROJECT_INTERFACE_PACKAGE_DEPENDENCIES ${PROJECT_INTERFACE_PACKAGE_DEPENDENCIES} ${pkgname})
    endif()

  endif()
  
  unset(__version__)
  unset(__components__)
  unset(__required_versions__)
  set(CMAKE_PREFIX_PATH ${__old_cpp__})
  unset(__old_cpp__)
endmacro()

