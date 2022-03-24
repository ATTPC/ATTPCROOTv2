cmake_minimum_required(VERSION  3.11.0 FATAL_ERROR)
cmake_policy(VERSION 3.11...3.14)
foreach(p
  CMP0025 # Compiler id for Apple Clang is now AppleClang.
  CMP0028 # double colon for imported and alias targets
  CMP0042 # MACOSX_RPATH is enabled by default.
  )
  if(POLICY ${p})
  cmake_policy(SET ${p} NEW)
  endif()
endforeach()

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
# If we are building debug then
#f(CMAKE_BUILD_TYPE STREQUAL "Debug")
#  message("Suppressing warning from precompiler conditions because of FairLogger!")
#  add_compile_options(-Wno-cpp "-Wno-#warnings")
#endif()

# searches for needed packages
# REQUIRED means that cmake will stop if this packages are not found
# For example the framework can run without GEANT4, but ROOT is
# mandatory

find_package2(PUBLIC FairRoot REQUIRED)

find_package2(PUBLIC ROOT 6.16 CONFIG)
if(NOT ROOT_FOUND)
  find_package2(PUBLIC ROOT 6.0 REQUIRED)
  message("Looked for ROOT using modules and found ${ROOT_VERSION}")
endif()

if(NOT ROOT_vmc_FOUND)
  message("NO VMC in ROOT install, looking for seperate package")
  find_package2(PUBLIC VMC REQUIRED CONFIG)
  set_target_properties(VMCLibrary PROPERTIES IMPORTED_GLOBAL TRUE)
  add_library(ROOT::VMC ALIAS VMCLibrary)
endif()
 
find_package2(PUBLIC FairLogger  VERSION 1.2.0 REQUIRED)
foreach(dep IN LISTS FairLogger_PACKAGE_DEPENDENCIES)
  if(NOT dep STREQUAL "Boost")
    find_package2(PUBLIC ${dep} REQUIRED VERSION ${FairLogger_${dep}_VERSION})
    set(PROJECT_${dep}_VERSION ${FairLogger_${dep}_VERSION})
  endif()
endforeach()

find_package2(PUBLIC Pythia6 REQUIRED)
find_package2(PUBLIC Pythia8)
find_package2(PUBLIC Geant3)
find_package2(PUBLIC Geant4)
Find_Package2(PUBLIC VGM)
find_package2(PUBLIC Geant4VMC)
find_package2(PUBLIC CLHEP)
find_package2(PUBLIC HEPMC)

if(NOT DEFINED Boost_NO_BOOST_CMAKE AND CMAKE_VERSION VERSION_LESS 3.15)
  Set(Boost_NO_BOOST_CMAKE ON)
endif()

if(Boost_NO_BOOST_CMAKE)
  # If an older version of boost is found both of the variables below are
  # cached and in a second cmake run, a good boost version is found even
  # if the version is to old.
  # To overcome this problem both variables are cleared before checking
  # for boost.
  Unset(Boost_INCLUDE_DIR CACHE)
  Unset(Boost_LIBRARY_DIRS CACHE)
endif()

find_package2(PUBLIC Boost
  VERSION 1.67
  ADD_REQUIREMENTS_OF FairRoot FairMQ FairLogger
)

if(CMAKE_VERSION VERSION_LESS 3.15)
  set(Boost_VERSION "${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION}")
endif()

## Add additional packages beyond the initial required for FairRoot
find_package2(PUBLIC HDF5 REQUIRED CONFIG)
find_package2(PUBLIC PCL 1.8 CONFIG)
find_package2(PUBLIC GEANT4DATA)
find_package2(PUBLIC GENFIT2)
find_package2(PUBLIC OpenMP)
find_package2(PUBLIC CUDA)
find_package2(PUBLIC Eigen3 CONFIG)
find_package2(PUBLIC FLANN)
find_package2(PUBLIC HiRAEVT CONFIG)

# FAIRROOT depends on VMC in a way that it exposes the headers, but does not
# need someone who is linking against it to also link against VMC. Until FairRoot
# finally finished the move to modern CMake this is a hacky way of ensure that if
# you include FairRoot, you are also getting the headers it depends on
# This replaces the call to SetBasicVariables in FairMacros.cmake which did some
# nasty things like calling link_directories() and include_directories()
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

# Install files and make config file
message("${headers} Include dir: ${CMAKE_INSTALL_LIBDIR}")

# Add install targets for libraries
install(EXPORT GeneratedRootLibTargets
  FILE ${PROJECT_NAME}.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME} )

include(CMakePackageConfigHelpers)

#generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
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

