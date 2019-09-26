# Create a library called "libATReco" which includes the source files given in the array .

set(INCLUDE_DIRECTORIES
${HDF5_INCLUDE_DIR}
${ROOT_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${BASE_INCLUDE_DIRECTORIES}
${PCL_INCLUDE_DIRS}
# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/AtMap
${CMAKE_SOURCE_DIR}/par
${CMAKE_SOURCE_DIR}/reco
${CMAKE_SOURCE_DIR}/reco/GETDecoder
${CMAKE_SOURCE_DIR}/reco/GETDecoder2
${CMAKE_SOURCE_DIR}/reco/ATDecoder
${CMAKE_SOURCE_DIR}/reco/ATPulseAnalyzer
${CMAKE_SOURCE_DIR}/reco/HierarchicalClustering
${CMAKE_SOURCE_DIR}/reco/HoughTransformation
${CMAKE_SOURCE_DIR}/reco/ATProtoPhiReco
${CMAKE_SOURCE_DIR}/reco/VMEDecoder
${CMAKE_SOURCE_DIR}/reco/ATMinimization
${CMAKE_SOURCE_DIR}/reco/ATAnalysis
${CMAKE_SOURCE_DIR}/reco/ATRansac
${CMAKE_SOURCE_DIR}/reco/ATPatternRecognition
${CMAKE_SOURCE_DIR}/tpc

)

include_directories(${INCLUDE_DIRECTORIES})

set(LINK_DIRECTORIES
${ROOT_LIBRARY_DIR}
${FAIRROOT_LIBRARY_DIR}
${Boost_LIBRARY_DIRS}
${PCL_LIBRARY_DIRS}
${PCL_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

link_directories(${LINK_DIRECTORIES})

set(SRCS
# Add all the source files below this line. Those must have cc for their extension.
GETDecoder/GETDecoder.cc
GETDecoder/GETFrame.cc
GETDecoder/GETMath.cc
GETDecoder/GETPlot.cc

GETDecoder2/GETDecoder2.cc
GETDecoder2/GETFrameInfo.cc

GETDecoder2/GETHeaderBase.cc
GETDecoder2/GETBasicFrameHeader.cc
GETDecoder2/GETLayerHeader.cc

GETDecoder2/GETTopologyFrame.cc
GETDecoder2/GETBasicFrame.cc
GETDecoder2/GETCoboFrame.cc
GETDecoder2/GETLayeredFrame.cc

GETDecoder2/GETMath2.cc
GETDecoder2/GETFileChecker.cc

ATDecoder/ATCore.cc
ATDecoder/ATCore2.cc
ATDecoder/ATPad.cc
#ATDecoder/ATPad2.cc
ATDecoder/ATRawEvent.cc
ATDecoder/ATHit.cc
ATDecoder/ATEvent.cc
ATDecoder/ATProtoEvent.cc
ATDecoder/ATProtoEventAna.cc
ATDecoder/ATTrackingEventAna.cc
ATDecoder/ATPatternEvent.cc
ATDecoder/ATProtoQuadrant.cc
ATDecoder/ATPedestal.cc
ATDecoder/ATTrack.cc
ATDecoder/ATHDFParser.cc

ATPulseAnalyzer/ATPSA.cc
ATPulseAnalyzer/ATPSASimple.cc
ATPulseAnalyzer/ATPSASimple2.cc
ATPulseAnalyzer/ATPSAFilter.cc
ATPulseAnalyzer/ATPSAProto.cc
ATPulseAnalyzer/ATPSAProtoFull.cc
ATPulseAnalyzer/ATPSAFull.cc

HierarchicalClustering/ATCubicSplineFit.cc
HierarchicalClustering/ATFindVertex.cc
HierarchicalClustering/ATHierarchicalClusteringHc.cc
HierarchicalClustering/ATHierarchicalClusteringSmoothenCloud.cc
HierarchicalClustering/ATTrajectory.cc

ATPulseAnalyzer/ATCalibration.cc

HoughTransformation/ATHoughSpace.cc
HoughTransformation/ATHoughSpaceLine.cc
HoughTransformation/ATHoughSpaceCircle.cc

ATProtoPhiReco/ATPhiReco.cc
ATProtoPhiReco/ATPhiRecoSimple.cc
ATProtoPhiReco/ATPhiRecoTriple.cc

VMEDecoder/VMEDecoder.cc
VMEDecoder/VMECore.cc
VMEDecoder/ATRawIC.cc
VMEDecoder/VMERawEvent.cc

ATMinimization/ATMinimization.cc
ATMinimization/ATMCMinimization.cc
ATMinimization/ATMCQMinimization.cc

ATAnalysis/ATAnalysis.cc
ATAnalysis/ATProtoAnalysis.cc
ATAnalysis/ATTrackingAnalysis.cc
ATAnalysis/ATd2HeAnalysis.cc

ATRansac/ATRansac.cc

ATPatternRecognition/ATPRA.cc
ATPatternRecognition/ATTrackFinderHC.cc
ATPatternRecognition/smoothenCloud.cc
ATPatternRecognition/msd.cc
ATPatternRecognition/cluster.cc
ATPatternRecognition/hc.cc
ATPatternRecognition/fastcluster.cc


ATDecoderTask.cc
ATDecoder2Task.cc
ATHDFParserTask.cc
ATPSATask.cc
ATPhiRecoTask.cc
ATHoughTask.cc
ATVMEUnpackTask.cc
ATAnalysisTask.cc
ATRansacTask.cc
ATHierarchicalClusteringTask.cc
ATPRATask.cc

)

# Adding libraries that depend on PCL
#if (PCL_FOUND)
#set(SRCS ${SRCS} HoughTransformation/ATHoughSpaceLine3D.cc)
#endif()

CHANGE_FILE_EXTENSION(*.cc *.hh HEADERS "${SRCS}")

Set(LINKDEF ATTPCRecoLinkDef.h)
Set(LIBRARY_NAME ATTPCReco)
Set(DEPENDENCIES
    Base
    FairLogger::FairLogger
#   Cling
#    MCStack
    XMLParser
# Add all library names your code depending on.
# Basically those are the libraries in the header files' directory.
    AtTpcMap
    ATPar
    Spectrum
    ${HDF5_LIBRARIES}
    ${Boost_LOG_LIBRARY}
    ${Boost_LOG_SETUP_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${PCL_LIBRARIES}
)



GENERATE_LIBRARY()
