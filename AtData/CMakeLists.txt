Set(LIBRARY_NAME AtData)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)


set(INCLUDE_DIRECTORIES
  ${FAIRROOT_INCLUDE_DIR}
)

set(LINK_DIRECTORIES
${FAIRROOT_LIBRARY_DIR}
)

Set(DEPENDENCIES
  Base
  FairTools

  ROOT::Core
  ROOT::XMLParser
  )

set(LINK_DIRECTORIES
  ${FAIRROOT_LIBRARY_DIR}
  )

set(SRCS
  # Add all the source files below this line. Those must have cxx for their extension.

  AtPad.cxx
  AtAuxPad.cxx
  AtRawEvent.cxx
  AtHit.cxx
  AtHitCluster.cxx
  AtEvent.cxx
  AtProtoEvent.cxx
  AtProtoEventAna.cxx
  AtTrackingEventAna.cxx
  AtPatternEvent.cxx
  AtProtoQuadrant.cxx
  AtTrack.cxx
  )

### Add additional sources and libraries if we found certain modules ###

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
