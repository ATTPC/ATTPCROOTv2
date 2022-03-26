 ################################################################################
 #    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    #
 #                                                                              #
 #              This software is distributed under the terms of the             # 
 #         GNU Lesser General Public Licence version 3 (LGPL) version 3,        #  
 #                  copied verbatim in the file "LICENSE"                       #
 ################################################################################
# - Find ROOT instalation
# This module tries to find the ROOT installation on your system.
# It will start by looking for a ROOTConfig.cmake file and use that to import ROOT
#
# If it cannot find ROOT that way, it will fall back to trying to use root-config
# script which gives you all the needed information.
# If the system variable ROOTSYS is set this is straight forward.
# If not the module uses the pathes given in ROOT_CONFIG_SEARCHPATH.
# If you need an other path you should add this path to this varaible.  
# The root-config script is then used to detect basically everything else.
# This module defines a number of key variables, macros, and targets.
# Variables defined by this module using root-config version:
#
#   ROOT_FOUND               System has ROOT, this means the root-config 
#                            executable was found.
#
#   ROOT_INCLUDE_DIR         ROOT include directories: not cached
#
#   ROOT_LIBRARIES          Link to these to use the ROOT libraries, not cached
#
#   ROOT_LIBRARY_DIR         The path to where the ROOT library files are.
#
#   ROOT_VERSION_STRING      The version string of the ROOT libraries which
#                            is reported by root-config
#
#   ROOT_VERSION_MAJOR       Major version number of ROOT
#   ROOT_VERSION_MINOR       Minor version number of ROOT
#   ROOT_VERSION_PATCH       Patch version number of ROOT
#
#   ROOT_VERSION_NUMBER      A unique version number which is calculated from 
#                            major, minor and patch version found
#
#   ROOT_CINT_EXECUTABLE     The rootcint executable.
#
#   RLIBMAP_EXECUTABLE       The rlibmap executable.
#
# Targets manually defined by this module if using root-config version:
# VMC Core RIO Tree Rint Physics MathCore Thread Geom EG Eve Spectrum TreePlayer GenVector
#
# You can change the behavior of this script by using the following variables:
#
#   ROOT_ROOT
#     Specific path to the ROOTConfig.cmake file    
#
#   ROOT_NO_FIND_PACKAGE_CONFIG_FILE
#     Skip looking for ROOTConfig.cmake file



if(NOT ROOT_ROOT)
  set(ROOT_ROOT $ENV{ROOT_ROOT})
endif()
if(ROOT_ROOT)
  set(_ROOT_SEARCH_OPTS NO_DEFAULT_PATH)
else()
  set(_ROOT_SEARCH_OPTS)
endif()

if(NOT ROOT_FOUND AND NOT ROOT_NO_FIND_PACKAGE_CONFIG_FILE)
  message(STATUS "Looking for ROOT using config")
  # Require at least 6.16 to be considered valid
  # previous versions were not configured correctly
  find_package(ROOT QUIET CONFIG 6.16
    HINTS ${ROOT_ROOT}
    ${_ROOT_SEARCH_OPTS}
    )

  if(ROOT_FOUND)
    message(STATUS "Found ROOT version ${ROOT_VERSION}")
    return()
  endif()
endif()

if(NOT ROOT_FOUND)
  message(STATUS "Looking for ROOT using root-config")

  Set(ROOT_CONFIG_SEARCHPATH
    ${SIMPATH}/bin
    ${SIMPATH}/tools/root/bin
    $ENV{ROOTSYS}/bin
    )

  Set(ROOT_FOUND FALSE)
  Set(ROOT_DEFINITIONS "")
  Set(ROOT_INSTALLED_VERSION_TOO_OLD FALSE)
  Set(ROOT_CONFIG_EXECUTABLE ROOT_CONFIG_EXECUTABLE-NOTFOUND)

  Find_Program(ROOT_CONFIG_EXECUTABLE NAMES root-config 
    PATHS ${ROOT_CONFIG_SEARCHPATH}
    NO_DEFAULT_PATH
    )
  
  If(ROOT_CONFIG_EXECUTABLE)
    
    String(REGEX REPLACE "(^.*)/bin/root-config" "\\1" test ${ROOT_CONFIG_EXECUTABLE}) 
    Set(ENV{ROOTSYS} ${test})
    Set(ROOTSYS ${test})

    Execute_Process(COMMAND ${ROOT_CONFIG_EXECUTABLE} --version 
      OUTPUT_VARIABLE ROOT_VERSION_STRING
      )
    Execute_Process(COMMAND ${ROOT_CONFIG_EXECUTABLE} --prefix
      OUTPUT_VARIABLE ROOT_INSTALL_DIR
      )
    String(STRIP ${ROOT_VERSION_STRING} ROOT_VERSION_STRING)
    String(STRIP ${ROOT_INSTALL_DIR} ROOT_INSTALL_DIR)


    MESSAGE(STATUS "Looking for Root... - Found ${ROOT_INSTALL_DIR}/bin/root")
    MESSAGE(STATUS "Looking for Root... - Found version is ${ROOT_VERSION_STRING} ")   
    
    # extract major, minor, and patch versions from
    # the version string given by root-config
    String(REGEX REPLACE "^([0-9]+)\\.[0-9][0-9]+\\/[0-9][0-9]+.*" "\\1" ROOT_VERSION_MAJOR "${ROOT_VERSION_STRING}")
    String(REGEX REPLACE "^[0-9]+\\.([0-9][0-9])+\\/[0-9][0-9]+.*" "\\1" ROOT_VERSION_MINOR "${ROOT_VERSION_STRING}")
    String(REGEX REPLACE "^[0-9]+\\.[0-9][0-9]+\\/([0-9][0-9]+).*" "\\1" ROOT_VERSION_PATCH "${ROOT_VERSION_STRING}")

    # compute overall version numbers which can be compared at once
    if(DEFINED ROOT_FIND_VERSION_MAJOR AND DEFINED ROOT_FIND_VERSION_MINOR)
      Math(EXPR req_vers "${ROOT_FIND_VERSION_MAJOR}*10000 + ${ROOT_FIND_VERSION_MINOR}*100 + ${ROOT_FIND_VERSION_PATCH}")
    endif()
    Math(EXPR found_vers "${ROOT_VERSION_MAJOR}*10000 + ${ROOT_VERSION_MINOR}*100 + ${ROOT_VERSION_PATCH}")

    Set(ROOT_Version ${found_vers})
    Set(ROOT_VERSION_NUMBER ${found_vers})

    If(found_vers LESS req_vers)
      Set(ROOT_FOUND FALSE)
      Set(ROOT_INSTALLED_VERSION_TOO_OLD TRUE)
    Else(found_vers LESS req_vers)
      Set(ROOT_FOUND TRUE)
    EndIf(found_vers LESS req_vers)

  Else(ROOT_CONFIG_EXECUTABLE)
    Message(STATUS "Looking for Root... - Not found")
    Message(FATAL_ERROR "ROOT not installed in the searchpath and ROOTSYS is not set. Please set ROOTSYS or add the path to your ROOT installation in the Macro FindROOT.cmake in the subdirectory cmake/modules.")
  Endif(ROOT_CONFIG_EXECUTABLE)
endif()

If(ROOT_FOUND)

  # ask root-config for the library dir
  # Set ROOT_LIBRARY_DIR
  Execute_Process(COMMAND ${ROOT_CONFIG_EXECUTABLE} --libdir
                  OUTPUT_VARIABLE ROOT_LIBRARY_DIR
                 )
  String(STRIP ${ROOT_LIBRARY_DIR} ROOT_LIBRARY_DIR)

  # ask root-config for the binary dir
  Execute_Process(COMMAND ${ROOT_CONFIG_EXECUTABLE} --bindir
                  OUTPUT_VARIABLE ROOT_BINARY_DIR
                 )
  String(STRIP ${ROOT_BINARY_DIR} ROOT_BINARY_DIR)

  # ask root-config for the include dir
  Execute_Process(COMMAND ${ROOT_CONFIG_EXECUTABLE} --incdir
                  OUTPUT_VARIABLE ROOT_INCLUDE_DIR
                 )
  String(STRIP ${ROOT_INCLUDE_DIR} ROOT_INCLUDE_DIR)

  # ask root-config for the library varaibles
  Execute_Process(COMMAND ${ROOT_CONFIG_EXECUTABLE} --glibs
                  OUTPUT_VARIABLE ROOT_LIBRARIES
                 )
  String(STRIP ${ROOT_LIBRARIES} ROOT_LIBRARIES)

  if("${ROOT_VERSION_MAJOR}.${ROOT_VERSION_MINOR}" VERSION_GREATER 6.16)
    Execute_Process(COMMAND ${ROOT_CONFIG_EXECUTABLE} --has-vmc
      OUTPUT_VARIABLE ROOT_vmc_FOUND
      )
    String(STRIP ${ROOT_vmc_FOUND} ROOT_vmc_FOUND)
  else()
    set(ROOT_vmc_FOUND yes)
  endif()

  # Try to extract the CXX standard used to compile root
  execute_process(COMMAND ${ROOT_CONFIG_EXECUTABLE} --cflags
    OUTPUT_VARIABLE _root_c_flags)

  unset(_root_standard)
  String(REGEX MATCH "std=c\\+\\+([0-9]+)"  _root_standard "${_root_c_flags}")
  string(REGEX MATCH "([0-9]+)" _root_standard "${_root_standard}")
  message(STATUS "ROOT was built with CXX standard ${_root_standard} from flags ${_root_c_flags}")
  
  # Make variables changeble to the advanced user
  Mark_As_Advanced(ROOT_LIBRARY_DIR ROOT_INCLUDE_DIR ROOT_DEFINITIONS)


  #######################################
  #
  #       Check the executables of ROOT 
  #          ( rootcint ) 
  #
  #######################################

  Find_Program(ROOT_CINT_EXECUTABLE
    NAMES rootcint
    PATHS ${ROOT_BINARY_DIR}
    NO_DEFAULT_PATH
    )
  # Set rootcling to have the same name as the internal root target
  set(ROOT_rootcling_CMD ${ROOT_CINT_EXECUTABLE})
  
  Find_Program(RLIBMAP_EXECUTABLE
    NAMES rlibmap
    PATHS ${ROOT_BINARY_DIR}
    NO_DEFAULT_PATH
    )

  # Aliases for imported ROOT dependencies will probably need a lot more listed here eventually
  set(_root_deps 
    Eve
    EG
    Geom
    Ged
    Gpad
    Graf3d
    RGL
    Gui
    Core
    Imt
    RIO
    Net
    Hist
    Graf
    ROOTVecOps
    Tree
    TreePlayer
    Rint
    Postscript
    Matrix
    Physics
    MathCore
    Minuit
    Thread
    XMLParser
    Spectrum
    GenVector
    MultiProc
    ROOTDataFrame
    )
  if(ROOT_vmc_FOUND)
    list(APPEND _root_deps VMC)
  endif()

  foreach(_root_dep ${_root_deps})
    find_library(${_root_dep}_LIB ${_root_dep} PATHS ${ROOT_LIBRARY_DIR})
    if(${_root_dep}_LIB)
      add_library(${_root_dep} SHARED IMPORTED GLOBAL)
      set_target_properties(${_root_dep} PROPERTIES
	IMPORTED_LOCATION ${${_root_dep}_LIB}
	)
      target_include_directories(${_root_dep} INTERFACE ${ROOT_INCLUDE_DIR})
      target_compile_features(${_root_dep} INTERFACE cxx_std_${_root_standard})
      add_library(ROOT::${_root_dep} ALIAS ${_root_dep})
      message(STATUS "ROOT::${_root_dep} (${${_root_dep}_LIB}) target added by hand.")
    endif()
  endforeach(_root_dep)
  unset(_root_standard)
Else(ROOT_FOUND)

  If(ROOT_FIND_REQUIRED)
    Message(STATUS "Looking for ROOT... - Found version to old.")
    Message(STATUS "Looking for ROOT... - Minimum required version is ${ROOT_FIND_VERSION}")
    Message(FATAL_ERROR "Stop here because of a wrong Root version.")
  EndIf(ROOT_FIND_REQUIRED)

Endif(ROOT_FOUND)
