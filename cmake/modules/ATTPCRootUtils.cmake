# A set of utility functions/macros for the ATTPCROOT build system
# Based heavily on the work to modernize the FairRoot build system
# https://github.com/dennisklein/FairRoot/blob/modernize_cmake_phase2
# Adam Anthony 3/25/2022

macro(set_attpcroot_defaults)

  set(CMAKE_CXX_STANDARD ${PROJECT_MINIMUM_CXX_STANDARD})
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Create JSON compilation database

  # Install path is changed to "install" if not given a value by the user
  if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "install" CACHE PATH "..." FORCE)
    message("Setting default install prefix to: ${CMAKE_INSTALL_PREFIX}")
  endif()

  # Set default build type if not specified
  If(NOT CMAKE_BUILD_TYPE)
    Message("Set BuildType to RELWITHDEBINFO")
    set(CMAKE_BUILD_TYPE RELWITHDEBINFO)
  EndIf(NOT CMAKE_BUILD_TYPE)

  # https://cmake.org/Wiki/CMake_RPATH_handling
  # Set the RPATH for installed libraries as well as build libraries
  # so the user does not have to fiddle around with the LD_LIBRARY_PATH
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES
    "${CMAKE_INSTALL_PREFIX}/${PROJECT_INSTALL_LIBDIR}" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
      # If we are on linux, set the DT_RUNPATH tag instead of the DT_RPATH tag this
      # has a lower presedence than LD_LIBRARY_PATH if the user would like to override
      # the automatic selection of libraries to link against
      # https://news.ycombinator.com/item?id=14222349
      set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} "-Wl,--enable-new-dtags")
      set(CMAKE_SHARED_LINKER_FLAGS ${CMAKE_SHARED_LINKER_FLAGS} "-Wl,--enable-new-dtags")
      set(CMAKE_INSTALL_RPATH "$ORIGIN/${PROJECT_INSTALL_LIBDIR}")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      set(CMAKE_INSTALL_RPATH "@loader_path/${PROJECT_INSTALL_LIBDIR}")
    else()
      set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${PROJECT_INSTALL_LIBDIR}")
    endif()
  endif()

  # Add SIMPATH to module path so we can find everything...
  list(APPEND CMAKE_PREFIX_PATH ${SIMPATH})
  
endmacro(set_attpcroot_defaults)

function(join VALUES GLUE OUTPUT)
  string(REGEX REPLACE "([^\\]|^);" "\\1${GLUE}" _TMP_STR "${VALUES}")
  string(REGEX REPLACE "[\\](.)" "\\1" _TMP_STR "${_TMP_STR}") #fixes escaping
  set(${OUTPUT} "${_TMP_STR}" PARENT_SCOPE)
endfunction()

function(generate_package_dependencies)
  join("${PROJECT_INTERFACE_PACKAGE_DEPENDENCIES}" " " DEPS)
  set(PACKAGE_DEPENDENCIES "\
####### Expanded from @PACKAGE_DEPENDENCIES@ by configure_package_config_file() #######
set(${PROJECT_NAME}_PACKAGE_DEPENDENCIES ${DEPS})
")
  foreach(dep IN LISTS PROJECT_INTERFACE_PACKAGE_DEPENDENCIES)
    join("${PROJECT_INTERFACE_${dep}_COMPONENTS}" " " COMPS)
    if(COMPS)
      string(CONCAT PACKAGE_DEPENDENCIES ${PACKAGE_DEPENDENCIES} "\
set(${PROJECT_NAME}_${dep}_COMPONENTS ${COMPS})
")
    endif()
    join("${PROJECT_INTERFACE_${dep}_OPTIONAL_COMPONENTS}" " " OPT_COMPS)
    if(OPT_COMPS)
      string(CONCAT PACKAGE_DEPENDENCIES ${PACKAGE_DEPENDENCIES} "\
set(${PROJECT_NAME}_${dep}_OPTIONAL_COMPONENTS ${OPT_COMPS})
")
    endif()
    if(PROJECT_INTERFACE_${dep}_VERSION)
      string(CONCAT PACKAGE_DEPENDENCIES ${PACKAGE_DEPENDENCIES} "\
set(${PROJECT_NAME}_${dep}_VERSION ${PROJECT_INTERFACE_${dep}_VERSION})
")
    endif()
  endforeach()
  string(CONCAT PACKAGE_DEPENDENCIES ${PACKAGE_DEPENDENCIES} "\
#######################################################################################
")
  set(PACKAGE_DEPENDENCIES ${PACKAGE_DEPENDENCIES} PARENT_SCOPE)
endfunction()

function(generate_package_components)
  join("${PROJECT_PACKAGE_COMPONENTS}" " " COMPS)
  set(PACKAGE_COMPONENTS "\
####### Expanded from @PACKAGE_COMPONENTS@ by configure_package_config_file() #########
set(${PROJECT_NAME}_PACKAGE_COMPONENTS ${COMPS})
")
  foreach(comp IN LISTS PROJECT_PACKAGE_COMPONENTS)
    string(CONCAT PACKAGE_COMPONENTS ${PACKAGE_COMPONENTS} "\
set(${PROJECT_NAME}_${comp}_FOUND TRUE)
")
  endforeach()
  string(CONCAT PACKAGE_COMPONENTS ${PACKAGE_COMPONENTS} "\
check_required_components(${PROJECT_NAME})
")
set(PACKAGE_COMPONENTS ${PACKAGE_COMPONENTS} PARENT_SCOPE)
endfunction()
