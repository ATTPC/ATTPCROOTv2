# Library to hold extentions to fairroot classes

Set(LIBRARY_NAME AtAnalysis)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)

set(INCLUDE_DIRECTORIES
# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/AtReconstruction/AtPatternRecognition
#${CMAKE_SOURCE_DIR}/AtReconstruction/AtPatternRecognition/trackfinder
)

set(DEPENDENCIES
  FairRoot::Base
  ROOT::Core

  ATTPCROOT::AtData

  )

set(SRCS
  AtRunAna.cxx
  AtFindVertex.cxx
  )

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
