cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(triplclust)

set (CMAKE_CXX_FLAGS "-O2")

# with MS Visual C++ we must explicity switch on proper exception handling
if (MSVC)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif (MSVC)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

# all source files
set(SRC src/cluster.cxx src/triplet.cxx src/main.cxx src/dnn.cxx src/hclust/fastcluster.cxx src/kdtree/kdtree.cxx src/pointcloud.cxx src/output.cxx src/option.cxx src/util.cxx src/graph.cxx)

# default target (created with "make")
add_executable (triplclust ${SRC})

# webdemo target (created with "make demo")
add_executable (triplclust-demo ${SRC})
set_target_properties(triplclust-demo PROPERTIES EXCLUDE_FROM_ALL TRUE COMPILE_FLAGS "-DWEBDEMO")
add_custom_target(demo DEPENDS triplclust-demo)
