Set(LIBRARY_NAME AtData)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)

Set(DEPENDENCIES
  FairRoot::FairTools

  ROOT::Core
  ROOT::XMLParser
  ROOT::GenVector
  ROOT::Physics #For TVector3
  ROOT::Eve
  )

set(SRCS
  # Add all the source files below this line. Those must have cxx for their extension.

  AtPad.cxx
  AtAuxPad.cxx
  AtPadFFT.cxx
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

  AtPattern/AtPattern.cxx
  AtPattern/AtPatternLine.cxx
  AtPattern/AtPatternCircle2D.cxx
  AtPattern/AtPatternY.cxx
  AtPattern/AtPatternRay.cxx
  AtPattern/AtPatternTypes.cxx
  )

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  INCLUDE_DIR ${CMAKE_SOURCE_DIR}/AtData/AtPattern
  )
