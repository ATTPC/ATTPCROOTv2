# CMakeLists.txt for event package. It creates a library with dictionary and a main program
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(ROOT2HDF)

SET(ATTPCROOTPATH $ENV{VMCWORKDIR})
list(APPEND CMAKE_PREFIX_PATH ${ATTPCROOTPATH}/build/install) #Need access to ATTPCROOT

# Will also load all of its dependecies
find_package(ATTPCROOT 0.3 REQUIRED)

add_executable(R2HExe R2HMain.cc)
target_link_libraries (R2HExe

  ATTPCROOT::AtData
  ATTPCROOT::AtTools
  ATTPCROOT::AtSimulationData

  ROOT::Core
  ROOT::Tree
  ROOT::TreePlayer
  
  hdf5::hdf5_cpp-shared
)

