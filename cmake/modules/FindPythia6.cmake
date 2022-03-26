################################################################################
# Copyright (C) 2014-2019 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH  #
#                                                                              #
#              This software is distributed under the terms of the             #
#              GNU Lesser General Public Licence (LGPL) version 3,             #
#                  copied verbatim in the file "LICENSE"                       #
################################################################################

find_library(Pythia6_LIBRARY
  NAMES Pythia6 pythia6
  HINTS $ENV{SIMPATH}
  PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Pythia6
  REQUIRED_VARS Pythia6_LIBRARY
)

if(Pythia6_FOUND)
  get_filename_component(Pythia6_LIBRARY_DIR ${Pythia6_LIBRARY} DIRECTORY)
  get_filename_component(Pythia6_PREFIX ${Pythia6_LIBRARY_DIR}/.. ABSOLUTE)

  if(NOT TARGET Pythia6)
    add_library(Pythia6 UNKNOWN IMPORTED GLOBAL)
    set_target_properties(Pythia6 PROPERTIES
      IMPORTED_LOCATION ${Pythia6_LIBRARY}
      )
    # Requires CMake 3.15
    #add_library(Pythia6::Pythia6 ALIAS Pythia6) 
  endif()
endif()
