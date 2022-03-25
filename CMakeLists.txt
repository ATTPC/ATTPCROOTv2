cmake_minimum_required(VERSION  3.11.0 FATAL_ERROR)
cmake_policy(VERSION 3.11...3.14)

# Set project name and version
project(ATTPCROOT VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

include(AtTpcRootMacros)
include(CheckCompiler)
include(WriteConfigFile)
include(CTest)
include(ROOTTargetMacros)
include(GNUInstallDirs)
include(PrintSummary)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Create JSON compilation database

# Check for needed enviroment variables
check_and_set_enviroment()
check_out_of_source_build()
check_install_directory()
check_compiler()


message("Library paht: ${LD_LIBRARY_PATH}")

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(INCLUDE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/include")

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


find_package2(PUBLIC FairRoot REQUIRED)
set(ROOT_NO_FIND_PACKAGE_CONFIG_FILE TRUE)
find_package2(PUBLIC ROOT REQUIRED)

if(NOT TARGET ROOT::VMC)
  message("NO VMC target in ROOT install, looking for seperate package")
  find_package2(PUBLIC VMC REQUIRED CONFIG)
  set_target_properties(VMCLibrary PROPERTIES IMPORTED_GLOBAL TRUE)
  add_library(ROOT::VMC ALIAS VMCLibrary)
endif()
 
find_package2(PUBLIC FairLogger  VERSION 1.2.0 REQUIRED)
foreach(dep IN LISTS FairLogger_PACKAGE_DEPENDENCIES)
  if(NOT dep STREQUAL "Boost")
    find_package2(PUBLIC ${dep} REQUIRED VERSION ${FairLogger_${dep}_VERSION})
    set(PROJECT_${dep}_VERSION ${FairLogger_${dep}_VERSION})
    message("Loaded ${dep}")
  endif()
endforeach()

find_package2(PRIVATE Pythia6)
find_package2(PRIVATE Pythia8)
find_package2(PRIVATE Geant3)
find_package2(PRIVATE Geant4)
Find_Package2(PRIVATE VGM)
find_package2(PRIVATE Geant4VMC)
find_package2(PRIVATE CLHEP)
find_package2(PRIVATE HEPMC)

# Right now, we just use header-only features of Boost (multi_array)
find_package2(PUBLIC Boost 1.67
  REQUIRED headers
  ADD_REQUIREMENTS_OF FairRoot FairMQ FairLogger
  )
  
if(CMAKE_VERSION VERSION_LESS 3.15)
  set(Boost_VERSION "${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION}")
endif()


# Look for HDF5 and if it didn't import the target becuase is couldn't
# find a config file, generate the target by hand using the output
# of FindHDF5.cmake
#set(HDF5_FIND_DEBUG TRUE)
find_package2(PRIVATE HDF5 REQUIRED)
if(NOT TARGET hdf5::hdf5-shared)
  add_library(hdf5::hdf5-shared SHARED IMPORTED GLOBAL)
  # This may have to be changed to HDF5_CXX_LIBRARIES depending on your hdf5 install
  # set(HDF5_FIND_DEBUG TRUE) before the find_package to see what you're working with
  foreach(_lib ${HDF5_C_LIBRARIES})
    get_filename_component(libName "${_lib}" NAME_WE)
    if(${libName} STREQUAL libhdf5)
      set_target_properties(hdf5::hdf5-shared PROPERTIES
	IMPORTED_LOCATION ${_lib})
    else()
      # This is a dependecy so remove lib prefix and add it to the interface
      # because it is a raw library name we will hope the linker can sort it
      # out. These should all be standard libraries. If that doesn't work force
      # you, add ${_lib} to the interface and it will hard-code the link
      # to the version of the dependencies hdf5 was compiled against on the system.
      string(REGEX REPLACE "^lib" "" lib_name_wo_prefix ${libName})
      target_link_libraries(hdf5::hdf5-shared INTERFACE ${lib_name_wo_prefix})
    endif()
  endforeach()
  set_target_properties(hdf5::hdf5-shared PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${HDF5_INCLUDE_DIRS}
    )
  message(STATUS "hdf5::hdf5-shared target added by hand.")
  get_target_property(hdf_libdir hdf5::hdf5-shared LOCATION)
  get_target_property(hdf_inc hdf5::hdf5-shared INTERFACE_INCLUDE_DIRECTORIES)
  message(STATUS "hdf5 library in use: ${hdf_libdir}")
  message(STATUS "hdf5 include directory: ${hdf_inc}")
endif()

# Now find the rest of out dependencies
find_package2(PRIVATE PCL 1.8 CONFIG REQUIRED QUIET)
find_package2(PRIVATE GEANT4DATA)
find_package2(PRIVATE OpenMP)
find_package2(PRIVATE CUDA)
find_package2(PUBLIC HiRAEVT CONFIG)
find_package2(PUBLIC GENFIT2)

# FAIRROOT depends on VMC in a way that it exposes the headers, but does not
# seem to need someone who is linking against it to also link against the VMCLibrary.
# Until FairRoot finally finished the move to modern CMake this is a hacky way of ensure that if
# you include FairRoot, you are also getting the headers it depends on
# This replaces the call to SetBasicVariables in FairMacros.cmake which did some
# nasty things like calling link_directories() and include_directories()
set(ROOT_INCLUDE_PATH ${FAIRROOT_INCLUDE_DIR})
set(FAIRROOT_INCLUDE_DIR ${FAIRROOT_INCLUDE_DIR}
  ${VMC_INCLUDE_DIRS}
  )

if (OPENMP_CXX_FOUND)
 # MESSAGE("${Green} OpenMP Support Found ${OpenMP_CXX_SPEC_DATE} ${ColourReset}")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  # set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS_DEBUG} ${OpenMP_C_FLAGS}")
  # set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} ${OpenMP_CXX_FLAGS}")
  
endif()

#set(LD_LIBRARY_PATH  ${CMAKE_BINARY_DIR}/lib ${LD_LIBRARY_PATH})

# Recurse into the given subdirectories.  This does not actually
# cause another cmake executable to run.  The same process will walk through
# the project's entire directory structure.
add_subdirectory (AtSimulationData)
add_subdirectory (AtData)
add_subdirectory (AtUnpack)
add_subdirectory (AtGenerators)
add_subdirectory (AtTpc)
add_subdirectory (AtSiArray)
add_subdirectory (AtPassive)
add_subdirectory (AtField)
add_subdirectory (AtMap)
add_subdirectory (AtReconstruction)
add_subdirectory (AtParameter)
add_subdirectory (AtDigitization)
add_subdirectory (AtS800)
add_subdirectory (AtApollo)
add_subdirectory (AtAnalysis)
add_subdirectory (AtTools)
if (PCL_FOUND)
  add_subdirectory (AtEventDisplay)
endif()

WRITE_CONFIG_FILE(config.sh)

configure_file(${CMAKE_SOURCE_DIR}/CTestCustom.cmake
  ${CMAKE_BINARY_DIR}/CTestCustom.cmake
  )



####### Install files and make config file for loading into cmake project ########

# Add install targets for libraries
install(EXPORT GeneratedRootLibTargets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME} )

include(CMakePackageConfigHelpers)

#generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

# Generate the version file for the config
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
  )

PrintSummary()

