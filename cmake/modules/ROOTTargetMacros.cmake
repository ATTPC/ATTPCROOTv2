################################################################################
#    Copyright (C) 2019 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    #
#    Copyright (C) 2019 CERN and copyright holders of ALICE O2                 #
#                                                                              #
#              This software is distributed under the terms of the             #
#              GNU Lesser General Public Licence (LGPL) version 3,             #
#                  copied verbatim in the file "LICENSE"                       #
################################################################################

# Creates a target for a new library, and adds everything needed to build it, including the dictionary
# All file paths should be relative ones to the calling CMakeLists.txt file
# By default all of the header files are assumed to be public headers and added to the target
# as such. That way anyone depending on this library will know about these headers.
# Also creates a target alias from target to PROJECT_NAME::target
#
# Adam Anthony 5/20/21
#
# Arguments to function:
#
# target: The name of the library to build (required)
# SRCS: Sources to build (required)
# LINKDEF: Name of the linkdef file (will skip dictionary generation of not present)
# HDRS: List of all header files to use in library generation.
#     - Will use all SRCS renamed from .cxx to .h if empty
# INCLUDE_DIR: List of include directories in addition to:
#     -CMAKE_CURRENT_SOURCE_DIR
# LIBRARY_DIR: List of link directories in addition to ROOT_LIBRARY_DIR
# DEPS_PUBLIC: Public dependencies
# DEPS_PRIVATE: Private dependencies

function(generate_target_and_root_library target)
  cmake_parse_arguments(PARSE_ARGV
    1
    HT
    ""
    "LINKDEF"
    "SRCS;HDRS;INCLUDE_DIR;LIBRARY_DIR;DEPS_PUBLIC;DEPS_PRIVATE"
    )

  # Check for required and set defaults
  if(HT_UNPARSED_ARGUMENTS)
    message(
      FATAL_ERROR "Unexpected unparsed arguments: ${HT_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT HT_SRCS)
    message(FATAL_ERROR "Missing required argument: SRCS")
  endif()

  if(NOT HT_HDRS)
    change_extensions_if_exists(.cxx .h
      FILES "${HT_SRCS}"
      OUTVAR HT_HDRS)
  endif()

  # Add defaults to include directories
  list(APPEND HT_INCLUDE_DIR 
    ${CMAKE_CURRENT_SOURCE_DIR} )
  
  #Check that files are relative
  foreach(h ${HT_SRCS} ${HT_HDRS} ${HT_LINKDEF})
    if(IS_ABSOLUTE ${h})
      message(
        FATAL_ERROR "add_root_dictionary only accepts relative paths, but the"
        "following path is absolute : ${h}")
    endif()
  endforeach()

  # Convert headers from relative paths to absolute and make sure they exist and copy them
  # to ${CMAKE_BINARY_DIR}/include
  unset(headers)
  foreach(h ${HT_HDRS})
    get_filename_component(habs ${CMAKE_CURRENT_SOURCE_DIR}/${h} ABSOLUTE)
    if(NOT EXISTS ${habs})
      message(
	FATAL_ERROR
	"generate_target_and_root_library was passed a non-existant input file: ${h}")
    endif()
    list(APPEND headers ${habs})
    get_filename_component(hName ${habs} NAME)
    configure_file(${habs} "${CMAKE_BINARY_DIR}/include/${hName}")
  endforeach()

  
  # Create the target and add the dependencies
  add_library(${target} SHARED)
  add_library(${PROJECT_NAME}::${target} ALIAS ${target})
  target_sources(${target} PRIVATE ${HT_SRCS})
  set_target_properties(${target} PROPERTIES
    VERSION "${PROJECT_VERSION}"
    SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
    )
  
  target_compile_options(${target} PRIVATE -Werror=return-type)
  
  # Add our includes either in the include directory for install or normal build paths
  # Build interface need to be commented because ${HT_INCLUDE_DIR} includes a ';'
  # which CMake does not like outside a string. It just appends the <BUILD_INTERFACE:...
  # to the last element of the list.
  # https://stackoverflow.com/questions/44425257/how-to-properly-use-target-include-directories-with-lists-of-includes
  target_include_directories(${target}
    PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> 
    "$<BUILD_INTERFACE:${HT_INCLUDE_DIR}>"
    )
  
  target_link_directories(${target} PUBLIC ${HT_LIBRARY_DIR})
  target_link_libraries(${target} PUBLIC ${HT_DEPS_PUBLIC})
  target_link_libraries(${target} PRIVATE ${HT_DEPS_PRIVATE})

  
  # Make the dictionary
  if(HT_LINKDEF)
    make_target_root_dictionary( ${target}
      HEADERS ${HT_HDRS}
      LINKDEF ${HT_LINKDEF})
  endif()

  install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(TARGETS ${target}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    EXPORT GeneratedRootLibTargets)
  
endfunction()

# based on the work in https://github.com/AliceO2Group/AliceO2

#
# make_target_root_dictionary generates one dictionary to be added to a target.
#
# Besides the dictionary source itself two files are also generated : a rootmap
# file and a pcm file. Those two will be installed alongside the target's
# library file
#
# arguments :
#
# * 1st parameter (required) is the target the dictionary should be added to
#
# * HEADERS (required) is a list of relative filepaths needed for the dictionary
#   definition
#
# * LINKDEF (required) is a single relative filepath to the LINKDEF file needed
#   by rootcling.
#
# LINKDEF and HEADERS must contain relative paths only (relative to the
# CMakeLists.txt that calls this add_root_dictionary function).
#
# The target must be of course defined _before_ calling this function (i.e.
# add_library(target ...) has been called).
#
# In addition :
#
# * target_include_directories _must_ have be called as well, in order to be
#   able to compute the list of include directories needed to _compile_ the
#   dictionary
#
# Note also that the generated dictionary is added to PRIVATE SOURCES list of
# the target.
#

function(make_target_root_dictionary target)
  cmake_parse_arguments(PARSE_ARGV
    1
    A
    ""
    "LINKDEF"
    "HEADERS;BASENAME")
  if(A_UNPARSED_ARGUMENTS)
    message(
      FATAL_ERROR "Unexpected unparsed arguments: ${A_UNPARSED_ARGUMENTS}")
  endif()

  if(A_BASENAME)
    message(STATUS "BASENAME parameter is deprecated. Will be ignored")
  endif()

  set(required_args "LINKDEF;HEADERS")
  foreach(required_arg IN LISTS required_args)
    if(NOT A_${required_arg})
      message(FATAL_ERROR "Missing required argument: ${required_arg}")
    endif()
  endforeach()

  # check all given filepaths are relative ones
  foreach(h ${A_HEADERS} ${A_LINKDEF})
    if(IS_ABSOLUTE ${h})
      message(
        FATAL_ERROR "add_root_dictionary only accepts relative paths, but the"
        "following path is absolute : ${h}")
    endif()
  endforeach()

  # convert all relative paths to absolute ones. LINKDEF must be the last one.
  unset(headers)
  foreach(h ${A_HEADERS} ${A_LINKDEF})
    get_filename_component(habs ${CMAKE_CURRENT_SOURCE_DIR}/${h} ABSOLUTE)
    list(APPEND headers ${habs})
  endforeach()

  # check all given filepaths actually exist
  foreach(h ${headers})
    get_filename_component(f ${h} ABSOLUTE)
    if(NOT EXISTS ${f})
      message(
        FATAL_ERROR
        "add_root_dictionary was given an inexistant input include ${f}")
    endif()
  endforeach()

  # Generate the pcm and rootmap files alongside the library
  get_property(lib_output_dir
    TARGET ${target}
    PROPERTY LIBRARY_OUTPUT_DIRECTORY)


  set(lib_output_dir ${CMAKE_BINARY_DIR}/lib)

  # Define the names of generated files
  get_property(basename TARGET ${target} PROPERTY OUTPUT_NAME)
  if(NOT basename)
    set(basename ${target})
  endif()
  set(dictionary G__${basename})
  set(dictionaryFile ${CMAKE_CURRENT_BINARY_DIR}/${dictionary}.cxx)
  set(pcmBase ${dictionary}_rdict.pcm)
  set(pcmFile ${lib_output_dir}/${pcmBase})
  set(rootmapFile ${lib_output_dir}/lib${basename}.rootmap)
  file(MAKE_DIRECTORY ${lib_output_dir})
  
  # get the list of compile_definitions
  set(prop $<REMOVE_DUPLICATES:$<TARGET_PROPERTY:${target},COMPILE_DEFINITIONS>>)

  # Build the LD_LIBRARY_PATH required to get rootcling running fine
  #
  # Need at least root core library
  # get_filename_component(LD_LIBRARY_PATH ROOT::Core DIRECTORY)
  set(LD_LIBRARY_PATH ${ROOT_LIBRARY_DIR})
  # and possibly toolchain libs if we are using a toolchain
  if(DEFINED ENV{GCC_TOOLCHAIN_ROOT})
    set(LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:$ENV{GCC_TOOLCHAIN_ROOT}/lib")
    set(LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:$ENV{GCC_TOOLCHAIN_ROOT}/lib64")
  endif()

  set(includeDirs $<REMOVE_DUPLICATES:$<TARGET_PROPERTY:${target},INCLUDE_DIRECTORIES>>)
    
  # add a custom command to generate the dictionary using rootcling
  # cmake-format: off
  set(space " ")
  add_custom_command(
    OUTPUT ${dictionaryFile} ${pcmFile} ${rootmapFile}
    VERBATIM
    COMMAND ${CMAKE_COMMAND} -E env "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
    ${ROOT_rootcling_CMD}
    -f ${dictionaryFile}
    -inlineInputHeader
    -rmf ${rootmapFile}
    -rml $<TARGET_FILE_NAME:${target}>
    -I$<JOIN:${includeDirs},$<SEMICOLON>-I>
    $<$<BOOL:${prop}>:-D$<JOIN:${prop},$<SEMICOLON>-D>>
    ${headers}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/${pcmBase} ${pcmFile}
    COMMAND_EXPAND_LISTS
    DEPENDS ${headers})
  # cmake-format: on

  # add dictionary source to the target sources and suppress warnings
  target_sources(${target} PRIVATE ${dictionaryFile})
  set_source_files_properties(${dictionaryFile} PROPERTIES COMPILE_FLAGS -Wno-deprecated-declarations )
  
  get_property(libs TARGET ${target} PROPERTY INTERFACE_LINK_LIBRARIES)
  #if(NOT RIO IN_LIST libs)
  if(NOT ROOT::RIO IN_LIST libs)
    # add ROOT::IO if not already there as a target that has a Root dictionary
    # has to depend on ... Root
    target_link_libraries(${target} PUBLIC ROOT::RIO)
  endif()

  # Get the list of include directories that will be required to compile the
  # dictionary itself and add them as private include directories
  foreach(h IN LISTS A_HEADERS)
    if(IS_ABSOLUTE ${h})
      message(FATAL_ERROR "Path ${h} should be relative, not absolute")
    endif()
    get_filename_component(a ${h} ABSOLUTE)
    string(REPLACE "${h}" "" d "${a}")
    list(APPEND dirs ${d})
  endforeach()
  list(REMOVE_DUPLICATES dirs)
  
  target_include_directories(${target} PRIVATE ${dirs})
  #target_include_directories(${target} TODO:Update headers includes so only the include directory is needed
  #  PRIVATE
  #  $<BUILD_INTERFACE:${dirs}>
  #  $<INSTALL_INTERFACE:include>)
  

  # will install the rootmap and pcm files alongside the target's lib
  get_filename_component(dict ${dictionaryFile} NAME_WE)
  #message("Install dir: ${CMAKE_INSTALL_LIBDIR}")
  #gmessage("Install dir: ${CMAKE_INSTALL_INCLUDEDIR}")

  install(FILES ${rootmapFile} ${pcmFile} DESTINATION ${CMAKE_INSTALL_LIBDIR})

endfunction()

################################################################################
# Exchange file extention of file in list from ext1 to ext2,
# only if the resulting file exists in CMAKE_CURRENT_SOURCE_DIR,
# and assign the newly created list to 'output'.
# The input list is not changed at all
# Ex: change_file_extension_if_exists(*.cxx *.h "${TRD_SRCS}" TRD_HEADERS)
################################################################################
function(change_extensions_if_exists ext1 ext2)
  cmake_parse_arguments(ARGS "" "OUTVAR" "FILES" ${ARGN})

  set(required_args "FILES;OUTVAR")
  foreach(required_arg IN LISTS required_args)
    if(NOT ARGS_${required_arg})
      message(FATAL_ERROR "fair_change_extensions_if_exists is missing a required argument: ${required_arg}")
    endif()
  endforeach()

  if(${ext1} MATCHES "^[*][.]+.*$")
    string(REGEX REPLACE "^[*]+([.].*)$" "\\1" ext1new ${ext1})
  else()
    set(ext1new ${ext1})
  endif()
  if(${ext2} MATCHES "^[*][.]+.*$")
    string(REGEX REPLACE "^[*]+([.].*)$" "\\1" ext2new ${ext2})
  else()
    set(ext2new ${ext2})
  endif()
  foreach(file ${ARGS_FILES})
    set(newFile "")
    string(REGEX REPLACE "^(.*)${ext1new}$" "\\1${ext2new}" newFile ${file})
    if(NOT ${file} STREQUAL ${newFile} AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${newFile})
      list(APPEND result ${newFile})
    endif()
  endforeach()
  set(${ARGS_OUTVAR} ${result} PARENT_SCOPE)
endfunction()
