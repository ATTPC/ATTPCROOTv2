# Create a library called "libAtReco" which includes the source files given in the array .

Set(LIBRARY_NAME AtUnpack)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)


set(INCLUDE_DIRECTORIES
${FAIRROOT_INCLUDE_DIR}

# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/AtUnpack/GETDecoder2
)

set(LINK_DIRECTORIES
${FAIRROOT_LIBRARY_DIR}
)

Set(DEPENDENCIES
  Base
  FairTools

  AtMap
  AtParameter
  AtData
  
  ROOT::Core
  ROOT::Tree
  ROOT::TreePlayer
  #ROOT::VMC # Needed for multi-threading (TMCtls.h)
  
  hdf5::hdf5-shared
  )

set(LINK_DIRECTORIES
  ${FAIRROOT_LIBRARY_DIR}
  )

set(SRCS
  # Add all the source files below this line. Those must have cxx for their extension.

  AtPedestal.cxx
  AtUnpackTask.cxx
  AtUnpacker.cxx
  AtHDFUnpacker.cxx
  AtROOTUnpacker.cxx
  AtGRAWUnpacker.cxx

  GETDecoder2/GETDecoder2.cxx
  GETDecoder2/GETFrameInfo.cxx
  GETDecoder2/GETHeaderBase.cxx
  GETDecoder2/GETBasicFrameHeader.cxx
  GETDecoder2/GETLayerHeader.cxx
  GETDecoder2/GETTopologyFrame.cxx
  GETDecoder2/GETBasicFrame.cxx
  GETDecoder2/GETCoboFrame.cxx
  GETDecoder2/GETLayeredFrame.cxx
  GETDecoder2/GETMath2.cxx
  GETDecoder2/GETFileChecker.cxx
  
  )

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
