# Create a library called "libAtData" 

Set(target AtData)
Set(LINKDEF MCStackLinkDef.h)

set(sources
  AtStack.cxx 
  AtMCTrack.cxx
)
set(LIB_DIR

  #Set directories for things that don't have targets
  ${FAIRLIBDIR}
  )

# Dependencies, if tar
set(dependencies
  Base # FairRunIdGenerator
  FairTools # FairLogger
  
  ROOT::Core
  ROOT::Physics # TLorentzVector, TVector3
  ROOT::EG # TDatabasePDG, TParticle
  ${VMCLIB} # TMCProcess
  )

generate_target_root_library(${target}
  LINKDEF ${LINKDEF}
  SRCS ${sources}
  LIBRARY_DIR ${LIB_DIR}
  DEPS_PUBLIC ${dependencies}
  )


#fair_change_extensions_if_exists(.cxx .h FILES "${sources}" OUTVAR headers)

#add_library(${target} SHARED ${sources} ${headers})
#set_target_properties(${target} PROPERTIES ${PROJECT_LIBRARY_PROPERTIES})
#message(  ${FAIRLIBDIR} )
#target_link_directories(${target} PUBLIC
 # ${FAIRLIBDIR}
 # )

#target_link_libraries(${target} PUBLIC
  #Base # FairRunIdGenerator
  #FairTools # FairLogger

  #ROOT::Core
  #ROOT::Physics # TLorentzVector, TVector3
  #ROOT::EG # TDatabasePDG, TParticle
 # ${VMCLIB} # TMCProcess
#  )

#target_include_directories(${target} PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
#  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
#  )

#fairroot_target_root_dictionary(${target}
#  HEADERS ${headers}
#  LINKDEF MCStackLinkDef.h
#)

#install(TARGETS ${target} LIBRARY DESTINATION ${PROJECT_INSTALL_LIBDIR})
#install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
