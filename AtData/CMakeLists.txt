Set(LIBRARY_NAME AtData)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)

Set(DEPENDENCIES
  FairRoot::Base
  FairRoot::FairTools

  ROOT::Core
  ROOT::XMLParser
  ROOT::GenVector
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

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
