#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ class AtFilter - !;
#pragma link C++ class AtFilterFFT - !; // Don't generate any IO
#pragma link C++ class AtFilterCalibrate - !;
#pragma link C++ class AtFilterZero - !;
#pragma link C++ class AtFilterSubtraction - !;
#pragma link C++ class AtRemovePulser - !;
#pragma link C++ class AtFilterFPN - !;
#pragma link C++ class AtSCACorrect - !;

#pragma link C++ class AtPSA + ;
#pragma link C++ class AtPSASpectrum + ;
#pragma link C++ class AtPSAHitPerTB + ;
#pragma link C++ class AtPSAFull + ;
#pragma link C++ class AtPSATBAvg + ;
#pragma link C++ class AtPSAMax + ;
#pragma link C++ class AtPSASimple2 + ;
#pragma link C++ class AtPSADeconv - !;
#pragma link C++ class AtPSADeconvFit - !;
#pragma link C++ class AtPSAIterDeconv - !;
#pragma link C++ class AtPSAComposite - !;

#pragma link C++ nestedclass;
#pragma link C++ nestedtypedef;

#pragma link C++ class AtPATTERN::AtTrackFinderTC + ;
#pragma link C++ class AtPATTERN::AtPRA + ;
#pragma link C++ namespace AtPATTERN;

#pragma link C++ namespace SampleConsensus;
#pragma link C++ class SampleConsensus::AtSampleConsensus - !;
#pragma link C++ class SampleConsensus::AtEstimator - !;
#pragma link C++ enum SampleConsensus::Estimators;

#pragma link C++ class AtMacroTask + ;

/* Classes that depend on Genfit2 */
#pragma link C++ class genfit::AtSpacepointMeasurement + ;
#pragma link C++ class AtFITTER::AtFitter + ;
#pragma link C++ class AtFITTER::AtGenfit + ;
#pragma link C++ namespace AtFITTER;
#pragma link C++ class AtFitterTask + ;

#pragma link C++ namespace MCFitter;
#pragma link C++ class MCFitter::AtParameterDistribution - !;
#pragma link C++ class MCFitter::AtUniformDistribution - !;
#pragma link C++ class MCFitter::AtStudentDistribution - !;
#pragma link C++ class MCFitter::AtMCFitter - !;
#pragma link C++ class MCFitter::AtMCFission - !;
#pragma link C++ class AtMCFitterTask + ;

/* Tasks in AtReconstruction */
#pragma link C++ class AtPSAtask + ;
#pragma link C++ class AtPRAtask + ;
#pragma link C++ class AtRansacTask + ;
#pragma link C++ class AtSampleConsensusTask + ;
#pragma link C++ class AtDataReductionTask + ;
#pragma link C++ class AtSpaceChargeCorrectionTask + ;
#pragma link C++ class AtFilterTask + ;
#pragma link C++ class AtHDF5WriteTask + ;
#pragma link C++ class AtHDF5ReadTask + ;
#pragma link C++ class AtCopyTreeTask + ;
#pragma link C++ class AtLinkDAQTask + ;
#pragma link C++ class AtCopyAuxTreeTask + ;

#endif
