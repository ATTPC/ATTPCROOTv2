# Create a library called "libAtReco" which includes the source files given in the array .o

Set(LIBRARY_NAME AtReconstruction)
Set(LINKDEF ${LIBRARY_NAME}LinkDef.h)


set(INCLUDE_DIRECTORIES
# Add all the folders having header files you used in the code.
${CMAKE_SOURCE_DIR}/AtReconstruction/AtPulseAnalyzer
${CMAKE_SOURCE_DIR}/AtReconstruction/AtPatternRecognition
#${CMAKE_SOURCE_DIR}/AtReconstruction/AtPatternRecognition/trackfinder
${CMAKE_SOURCE_DIR}/AtReconstruction/AtFitter
${CMAKE_SOURCE_DIR}/AtReconstruction/AtFilter
${CMAKE_SOURCE_DIR}/AtReconstruction/AtPatternRecognition/triplclust/src
${CMAKE_SOURCE_DIR}/AtReconstruction/AtPatternRecognition/triplclust/src/hclust
${CMAKE_SOURCE_DIR}/AtReconstruction/AtPatternRecognition/triplclust/src/kdtree
)

Set(DEPENDENCIES
  FairRoot::Base
  FairRoot::FairTools

  ATTPCROOT::AtAnalysis
  ATTPCROOT::AtMap
  ATTPCROOT::AtParameter
  ATTPCROOT::AtSimulationData
  ATTPCROOT::AtData
  ATTPCROOT::AtTools
  
  ROOT::Spectrum
  ROOT::Core

  Boost::headers #multi_array
  hdf5::hdf5-shared
  )

set(SRCS
  # Add all the source files below this line. Those must have cxx for their extension.

  AtPulseAnalyzer/AtPSA.cxx
  AtPulseAnalyzer/AtPSASimple2.cxx
  AtPulseAnalyzer/AtPSASpectrum.cxx
  AtPulseAnalyzer/AtPSAMax.cxx
  AtPulseAnalyzer/AtPSAHitPerTB.cxx
  AtPulseAnalyzer/AtPSAFull.cxx
  AtPulseAnalyzer/AtPSATBAvg.cxx
  AtPulseAnalyzer/AtCalibration.cxx
  AtPulseAnalyzer/AtPSADeconv.cxx
  
  AtFilter/AtFilter.cxx
  AtFilter/AtFilterSubtraction.cxx
  AtFilter/AtFilterDivide.cxx
  AtFilter/AtTrapezoidFilter.cxx
  AtFilter/AtFilterCalibrate.cxx
  AtFilter/AtFilterFFT.cxx
  AtFilter/AtFilterZero.cxx
  AtFilter/AtRemovePulser.cxx
  AtFilter/AtFilterFPN.cxx
  
  AtPSAtask.cxx
  AtFilterTask.cxx
  AtAuxFilterTask.cxx
  AtDataReductionTask.cxx
  AtSpaceChargeCorrectionTask.cxx
  AtHDF5ReadTask.cxx
  AtHDF5WriteTask.cxx
  
  AtPatternRecognition/AtPRA.cxx
  AtPatternRecognition/AtSampleConsensus.cxx
  AtPatternRecognition/AtSampleEstimator.cxx
  AtPatternRecognition/AtEstimatorMethods.cxx
  AtRansacTask.cxx
  AtSampleConsensusTask.cxx
  AtPRAtask.cxx

  AtPatternRecognition/triplclust/src/cluster.cxx 
  AtPatternRecognition/triplclust/src/triplet.cxx 
  AtPatternRecognition/triplclust/src/main.cxx 
  AtPatternRecognition/triplclust/src/dnn.cxx
  AtPatternRecognition/triplclust/src/hclust/fastcluster.cxx 
  AtPatternRecognition/triplclust/src/kdtree/kdtree.cxx 
  AtPatternRecognition/triplclust/src/pointcloud.cxx 
  AtPatternRecognition/triplclust/src/output.cxx 
  AtPatternRecognition/triplclust/src/option.cxx 
  AtPatternRecognition/triplclust/src/util.cxx 
  AtPatternRecognition/triplclust/src/graph.cxx
  AtPatternRecognition/AtTrackFinderTC.cxx
  
  )

### Add additional sources and libraries if we found certain modules ###

# Adding libraries that depend on PCL
if(PCL_FOUND)
  set(SRCS ${SRCS} 
    #AtPatternRecognition/AtTrackFinderHC.cxx
    #AtPatternRecognition/trackfinder/smoothenCloud.cxx
    #AtPatternRecognition/trackfinder/msd.cxx
    #AtPatternRecognition/trackfinder/cluster.cxx
    #AtPatternRecognition/trackfinder/hc.cxx
    #AtPatternRecognition/trackfinder/fastcluster.cxx
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
  set(SRCS ${SRCS}
    AtFitter/AtFitter.cxx
    AtFitter/AtGenfit.cxx
    AtFitter/AtSpacePointMeasurement.cxx
    AtFitterTask.cxx
    )
  set(DEPENDENCIES ${DEPENDENCIES}
    GENFIT2::genfit2
    )
endif()

if(HiRAEVT_FOUND)
  set(SRCS ${SRCS}
    AtLinkDAQTask.cxx
    )
  set(DEPENDENCIES ${DEPENDENCIES}
    HTDetectors
    )
endif()

generate_target_and_root_library(${LIBRARY_NAME}
  LINKDEF ${LINKDEF}
  SRCS ${SRCS}
  INCLUDE_DIR ${INCLUDE_DIRECTORIES}
  LIBRARY_DIR ${LINK_DIRECTORIES}
  DEPS_PUBLIC ${DEPENDENCIES}
  )
