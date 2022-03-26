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

endmacro(set_attpcroot_defaults)
