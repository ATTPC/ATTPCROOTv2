# Create a library called "libAtReco" which includes the source files given in the array .

Set(LIBRARY_NAME AtReconstruction)
Set(LINKDEF "${LIBRARY_NAME}LinkDef.h")

Set(DEPS_PUBLIC
  
  # External dependencies
  ${HDF5_LIBRARIES}
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
  ${Boost_THREAD_LIBRARY}
  ${Boost_DAtE_TIME_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
    
  Base
  Spectrum
  #MCStack
  FairLogger::FairLogger
  XMLParser

  #Internal dependencies
  AtTpcMap
  AtParameter
  
)
#foreach(l ${PCL_LIBRARIES})
#  message(${l})
#endforeach()
set(LINK_DIRECTORIES

  # Link directories for external libraries that do not support
  # exporting their targets thorugh a Config.cmake file
  ${ROOT_LIBRARY_DIR}
  ${FAIRROOT_LIBRARY_DIR}
  ${Boost_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${GENFIT2_LIBRARY_DIR}
  )

set(INCLUDE_DIRECTORIES

  # Include directories for external libraries that do not support
  # exporting their targets through a Config.cmake file
  ${HDF5_INCLUDE_DIR}
  ${ROOT_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${BASE_INCLUDE_DIRECTORIES}
  ${PCL_INCLUDE_DIRS}
  ${GENFIT2_INCLUDE_DIR}

  # Add all the folders having header files you used in the code.
  ${CMAKE_SOURCE_DIR}/AtMap
  ${CMAKE_SOURCE_DIR}/AtParameter
  ${CMAKE_SOURCE_DIR}/AtTpc
  ${CMAKE_SOURCE_DIR}/AtGenerators

  # Add all the sub directories containing header files in this folder
    ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/GETDecoder
  ${CMAKE_CURRENT_SOURCE_DIR}/GETDecoder2
  ${CMAKE_CURRENT_SOURCE_DIR}/AtDecoder
  ${CMAKE_CURRENT_SOURCE_DIR}/AtPulseAnalyzer
  #${CMAKE_CURRENT_SOURCE_DIR}/HierarchicalClustering
  ${CMAKE_CURRENT_SOURCE_DIR}/HoughTransformation
  ${CMAKE_CURRENT_SOURCE_DIR}/AtProtoPhiReco
  ${CMAKE_CURRENT_SOURCE_DIR}/VMEDecoder
  ${CMAKE_CURRENT_SOURCE_DIR}/AtMinimization
  ${CMAKE_CURRENT_SOURCE_DIR}/AtAnalysis
  ${CMAKE_CURRENT_SOURCE_DIR}/AtRansac
  ${CMAKE_CURRENT_SOURCE_DIR}/AtPatternRecognition
  ${CMAKE_CURRENT_SOURCE_DIR}/AtFitter
  ${CMAKE_CURRENT_SOURCE_DIR}/AtFilter

  )



set(SRCS
# Add all the source files below this line. Those must have cxx for their extension.
GETDecoder/GETDecoder.cxx
GETDecoder/GETFrame.cxx
GETDecoder/GETMath.cxx
GETDecoder/GETPlot.cxx

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

AtDecoder/AtCore.cxx
AtDecoder/AtCore2.cxx
AtDecoder/AtPad.cxx
AtDecoder/AtRawEvent.cxx
AtDecoder/AtHit.cxx
AtDecoder/AtHitCluster.cxx
AtDecoder/AtEvent.cxx
AtDecoder/AtProtoEvent.cxx
AtDecoder/AtProtoEventAna.cxx
AtDecoder/AtTrackingEventAna.cxx
AtDecoder/AtPatternEvent.cxx
AtDecoder/AtProtoQuadrant.cxx
AtDecoder/AtPedestal.cxx
AtDecoder/AtTrack.cxx
AtDecoder/AtHDFParser.cxx

AtPulseAnalyzer/AtPSA.cxx
AtPulseAnalyzer/AtPSASimple.cxx
AtPulseAnalyzer/AtPSASimple2.cxx
AtPulseAnalyzer/AtPSAProto.cxx
AtPulseAnalyzer/AtPSAProtoFull.cxx
AtPulseAnalyzer/AtPSAFull.cxx
AtPulseAnalyzer/AtCalibration.cxx

AtFilter/AtFilter.cxx
AtFilter/AtFilterDivide.cxx

#HierarchicalClustering/AtCubicSplineFit.cxx
#HierarchicalClustering/AtFindVertex.cxx
#HierarchicalClustering/AtHierarchicalClusteringHc.cxx
#HierarchicalClustering/AtHierarchicalClusteringSmoothenCloud.cxx
#HierarchicalClustering/AtTrajectory.cxx

#AtProtoPhiReco/AtPhiReco.cxx
#AtProtoPhiReco/AtPhiRecoSimple.cxx
#AtProtoPhiReco/AtPhiRecoTriple.cxx

#VMEDecoder/VMEDecoder.cxx
#VMEDecoder/VMECore.cxx
#VMEDecoder/AtRawIC.cxx
#VMEDecoder/VMERawEvent.cxx

AtDecoderTask.cxx
AtDecoder2Task.cxx
AtHDFParserTask.cxx
AtPSAtask.cxx
AtFilterTask.cxx

#AtPhiRecoTask.cxx
#AtVMEUnpackTask.cxx
)

### Add additional sources and libraries if we found certain modules ###

# Adding libraries that depend on PCL
if(PCL_FOUND)
  MESSAGE("${Green} libAtTPCReco - Building PCL classes: RANSAC, Pattern Recognition, Hough Transformation, Minimization, Analysis, PSA/PSAFilter ${ColourReset} ")

  add_definitions(${PCL_DEFINITIONS})
  
  list(APPEND SRCS 
    AtRansac/AtRansac.cxx
    AtPulseAnalyzer/AtPSAFilter.cxx
    AtPatternRecognition/AtTrackFinderHC.cxx
    AtPatternRecognition/smoothenCloud.cxx
    AtPatternRecognition/msd.cxx
    AtPatternRecognition/cluster.cxx
    AtPatternRecognition/hc.cxx
    AtPatternRecognition/fastcluster.cxx
    AtPatternRecognition/AtPRA.cxx
    AtPatternRecognition/AtRansacMod.cxx
    AtPatternRecognition/AtMlesacMod.cxx
    AtPatternRecognition/AtLmedsMod.cxx
    AtMinimization/AtMinimization.cxx
    AtMinimization/AtMCMinimization.cxx
    AtMinimization/AtMCQMinimization.cxx
    AtAnalysis/AtAnalysis.cxx
    AtAnalysis/AtProtoAnalysis.cxx
    AtAnalysis/AtTrackingAnalysis.cxx
    AtAnalysis/Atd2HeAnalysis.cxx
    HoughTransformation/AtHoughSpace.cxx
    HoughTransformation/AtHoughSpaceLine.cxx
    HoughTransformation/AtHoughSpaceCircle.cxx
    AtRansac/AtRansac.cxx
    AtPRAtask.cxx
    AtHoughTask.cxx
    AtAnalysisTask.cxx
    AtRansacTask.cxx
    #AtHierarchicalClusteringTask.cxx
    )
endif()

if(GENFIT2_FOUND)
  MESSAGE("${Green} libAtTPCReco - Building GENFIT classes: AtFitter/Genfit ${ColourReset} ")

  list(APPEND SRCS
    AtFitter/AtFitter.cxx
    AtFitter/AtGenfit.cxx
    AtFitter/AtSpacePointMeasurement.cxx
    AtFitterTask.cxx
    )
  list(APPEND DEPS_PUBLIC 
    ${GENFIT2_LIBRARY_DIR}/libgenfit2.so
    )
endif()

if(HiRAEVT_FOUND)
  MESSAGE("${Green} libAtTPCReco - Building HiRAEVT classes ${ColourReset} ")

  list(APPEND SRCS
    AtLinkDAQTask.cxx
    )
  list(APPEND DEPENDENCIES
    HTDetectors)
endif()

# Generate a target for the library and export it 
#generate_target_root_library( ${LIBRARY_NAME}
#  SRCS ${SRCS}
#  LINKDEF ${LINKDEF}
#  DEPS_PUBLIC ${DEPENDENCIES}
#  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
#  LIBRARY_DIR ${LINK_DIRECTORIES}
#  )

link_directories(${LINK_DIRECTORIES})
include_directories(${INCLUDE_DIRECTORIES})
GENERATE_LIBRARY()
