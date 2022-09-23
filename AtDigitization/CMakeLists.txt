#Create a library called "libAtDigitization" which includes the source files given in the array.

Set(LINKDEF AtDigiLinkDef.h)
Set(LIBRARY_NAME AtDigitization)
Set(DEPENDENCIES
  FairRoot::Base
  FairRoot::FairTools

  ROOT::Core
  ROOT::GenVector
  
  ATTPCROOT::AtSimulationData
  ATTPCROOT::AtParameter
  ATTPCROOT::AtData
  ATTPCROOT::AtMap
  ATTPCROOT::AtTools
)

set(SRCS
# Add all the source files below this line.
AtClusterizeTask.cxx
AtClusterizeLineTask.cxx
AtPulseTask.cxx
AtPulseLineTask.cxx
AtSimulatedPoint.cxx
AtSimulatedLine.cxx
AtTrigger.cxx
AtTriggerTask.cxx
AtVectorResponse.cxx
)

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
