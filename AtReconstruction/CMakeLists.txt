# Create a library called "libAtReco" which includes the source files given in the array .

Set(LIBRARY_NAME AtReconstruction)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)

message("GENFIT2 INCLUDES: ${GENFIT2_INCLUDE_DIR}")
set(INCLUDE_DIRECTORIES
${BASE_INCLUDE_DIRECTORIES}
${GENFIT2_INCLUDE_DIR}

# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/AtReconstruction/GETDecoder
${CMAKE_SOURCE_DIR}/AtReconstruction/GETDecoder2
${CMAKE_SOURCE_DIR}/AtReconstruction/AtDecoder
${CMAKE_SOURCE_DIR}/AtReconstruction/AtPulseAnalyzer
#${CMAKE_SOURCE_DIR}/AtReconstruction/HierarchicalClustering
${CMAKE_SOURCE_DIR}/AtReconstruction/HoughTransformation
${CMAKE_SOURCE_DIR}/AtReconstruction/AtProtoPhiReco
${CMAKE_SOURCE_DIR}/AtReconstruction/VMEDecoder
${CMAKE_SOURCE_DIR}/AtReconstruction/AtMinimization
${CMAKE_SOURCE_DIR}/AtReconstruction/AtAnalysis
${CMAKE_SOURCE_DIR}/AtReconstruction/AtRansac
${CMAKE_SOURCE_DIR}/AtReconstruction/AtPatternRecognition
${CMAKE_SOURCE_DIR}/AtReconstruction/AtFitter
${CMAKE_SOURCE_DIR}/AtReconstruction/AtFilter
)

set(LINK_DIRECTORIES
${FAIRROOT_LIBRARY_DIR}
${GENFIT2_LIBRARY_DIR}
)

Set(DEPENDENCIES
  Base
  FairTools
  XMLParser
  AtAnalysis
  AtTpcMap
  AtParameter
  AtData
  AtGenerators
  ROOT::Spectrum
  ROOT::Core
  
  hdf5::hdf5-shared
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
  ${Boost_THREAD_LIBRARY}
  ${Boost_DAtE_TIME_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  )

set(LINK_DIRECTORIES
  ${FAIRROOT_LIBRARY_DIR}
  ${GENFIT2_LIBRARY_DIR}
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
  AtFilter/AtFilterSubtraction.cxx
  AtFilter/AtFilterDivide.cxx
  AtFilter/AtTrapezoidFilter.cxx
  AtFilter/AtFilterCalibrate.cxx

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
  AtAuxFilterTask.cxx
  AtDataReductionTask.cxx
  
  #AtPhiRecoTask.cxx
  #AtVMEUnpackTask.cxx
  )

### Add additional sources and libraries if we found certain modules ###

# Adding libraries that depend on PCL
if(PCL_FOUND)
  MESSAGE("${Green} libAtTPCReco - Building PCL classes: RANSAC, Pattern Recognition, Hough Transformation, Minimization, Analysis, PSA/PSAFilter ${ColourReset} ")
  set(SRCS ${SRCS} 
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
  set(DEPENDENCIES ${DEPENDENCIES}
    ${PCL_LIBRARIES}
    )
  # Add include directory and link directory if no targets
  if(NOT TARGET pcl_common)
    message("No set targets for PCL!")
    set(LINK_DIRECTORIES ${LINK_DIRECTORIES}
      ${PCL_LIBRARY_DIR}
      )
    set(INCLUDE_DIRECTORIES ${INCLUDE_DIRECTORIES}
      ${PCL_INCLUDE_DIRS}
      )
    
  endif()
endif()

if(GENFIT2_FOUND)
  MESSAGE("${Green} libAtTPCReco - Building GENFIT classes: AtFitter/Genfit ${ColourReset} ")
  set(SRCS ${SRCS}
    AtFitter/AtFitter.cxx
    AtFitter/AtGenfit.cxx
    AtFitter/AtSpacePointMeasurement.cxx
    AtFitterTask.cxx
    )
  set(DEPENDENCIES ${DEPENDENCIES}
    ${GENFIT2_LIBRARY_DIR}/libgenfit2.so
    )
endif()

if(HiRAEVT_FOUND)
  MESSAGE("${Green} libAtTPCReco - Building HiRAEVT classes ${ColourReset} ")
  set(SRCS ${SRCS}
    AtLinkDAQTask.cxx
    )
  set(DEPENDENCIES ${DEPENDENCIES}
    HTDetectors
    )
endif()

generate_target_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
