# Fairsoft requires an even higher version, so you can use that version if needed
cmake_minimum_required(VERSION  3.15.0 FATAL_ERROR)
cmake_policy(VERSION 3.11...3.15)

# Set project name and version
project(ATTPCROOT VERSION 0.3.0)

set(PROJECT_MINIMUM_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

include(ATTPCRootChecks)
include(FairSoftCheckCompiler)
attpcroot_sanity_checks()
check_compiler()

include(ATTPCRootUtils)
include(ATTPCRootFindPackage2)
include(ROOTTargetMacros)
include(WriteConfigFile)
include(CTest)
include(GNUInstallDirs)
include(PrintSummary)

set_attpcroot_defaults()

find_package2(PUBLIC FairRoot REQUIRED)
#set(ROOT_NO_FIND_PACKAGE_CONFIG_FILE TRUE)
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

find_package2(PUBLIC Pythia6)
find_package2(PUBLIC Pythia8)
find_package2(PUBLIC Geant3)
find_package2(PUBLIC Geant4)
Find_Package2(PUBLIC VGM)
find_package2(PUBLIC Geant4VMC)
find_package2(PUBLIC CLHEP)
find_package2(PUBLIC HEPMC)

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
find_package2(PUBLIC HDF5 REQUIRED)
if(TARGET hdf5-shared AND NOT TARGET hdf5::hdf5-shared)
  # Until 3.18 we can only alias global targets
  if(CMAKE_VERSION VERSION_LESS 3.18)
    set_target_properties(hdf5-shared PROPERTIES IMPORTED_GLOBAL TRUE)
  endif()
  add_library(hdf5::hdf5-shared ALIAS hdf5-shared)
endif()
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
if(TARGET hdf5_cpp-shared AND NOT TARGET hdf5::hdf5_cpp-shared)
  # Until 3.18 we can only alias global targets
  if(CMAKE_VERSION VERSION_LESS 3.18)
    set_target_properties(hdf5_cpp-shared PROPERTIES IMPORTED_GLOBAL TRUE)
  endif()
  add_library(hdf5::hdf5_cpp-shared ALIAS hdf5_cpp-shared)
endif()

# Include PCL and hide some of their warnings
# see here: https://github.com/PointCloudLibrary/pcl/issues/3680
# when this is fixed, we can remove the following 3 lines.
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)
  set(_temp_mod_dev_warning 1)
endif()
find_package2(PUBLIC PCL 1.8 CONFIG QUIET)
if(_temp_mod_dev_warning)
  unset(CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
endif()

#Include all of the other dependecies that  we need
#find_package2(PUBLIC OpenMP)
find_package2(PUBLIC CUDA)
find_package2(PUBLIC HiRAEVT CONFIG)
find_package2(PUBLIC GENFIT2)

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
add_subdirectory (AtSeGA)
add_subdirectory (AtAnalysis)
add_subdirectory (AtTools)
add_subdirectory (AtEventDisplay)


# Generate config file for build

fairroot_generate_config_sh(${CMAKE_BINARY_DIR}/config.sh
  BUILD
  PATH ${ROOT_BINARY_DIR}
  DEPENDENCIES ${PROJECT_PACKAGE_DEPENDENCIES}
  VMCWORKDIR ${CMAKE_SOURCE_DIR}
  )


####### Install files and make config file for loading into cmake project ########
include(CMakePackageConfigHelpers)

# Add install targets for libraries
install(EXPORT GeneratedRootLibTargets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::)

generate_package_dependencies()
generate_package_components()

# Generate the config file for others to use us and install it
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/scripts/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

# Generate the version file for the config and install it
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
  )
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

# Install our "Find..." modules so we can load dependecies when used
install(FILES
  cmake/modules/FindFairRoot.cmake
  cmake/modules/FindGeant3.cmake
  cmake/modules/FindGeant4.cmake
  cmake/modules/FindGeant4VMC.cmake
  cmake/modules/FindGENFIT2.cmake
  cmake/modules/FindHEPMC.cmake
  cmake/modules/FindPythia6.cmake
  cmake/modules/FindPythia8.cmake
  cmake/modules/FindROOT.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/modules)

# Install our utility scripts for others if they want
install(FILES
  cmake/modules/ATTPCRootChecks.cmake
  cmake/modules/ATTPCRootFindPackage2.cmake
  cmake/modules/ATTPCRootUtils.cmake
  cmake/modules/PrintSummary.cmake
  cmake/modules/ROOTTargetMacros.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/modules
  )

# Install config script and install it
fairroot_generate_config_sh(${CMAKE_BINARY_DIR}/bin/ATTPCROOTConfig.sh_install
  INSTALL
  PATH ${ROOT_BINARY_DIR}
  DEPENDENCIES ${PROJECT_PACKAGE_DEPENDENCIES}
  VMCWORKDIR ${CMAKE_SOURCE_DIR}
  )

install(PROGRAMS ${CMAKE_BINARY_DIR}/bin/ATTPCROOTConfig.sh_install
  DESTINATION ${CMAKE_INSTALL_BINDIR} RENAME ATTPCROOTConfig.sh)

PrintSummary()
