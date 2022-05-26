#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ class AtFilterFFT - !; // Don't generate any IO
#pragma link C++ class AtFilterCalibrate - !;

#pragma link C++ class AtPSA + ;
#pragma link C++ class AtPSASpectrum + ;
#pragma link C++ class AtPSAHitPerTB + ;
#pragma link C++ class AtPSAFull + ;
#pragma link C++ class AtPSATBAvg + ;
#pragma link C++ class AtPSAMax + ;
#pragma link C++ class AtPSASimple2 + ;

#pragma link C++ nestedclass;
#pragma link C++ nestedtypedef;

#pragma link C++ class AtPATTERN::AtTrackFinderHC + ;
#pragma link C++ class AtPATTERN::AtPRA + ;
#pragma link C++ namespace AtPATTERN;

#pragma link C++ namespace SampleConsensus;
#pragma link C++ class SampleConsensus::AtSampleConsensus - !;

/* Classes that depend on Genfit2 */
#pragma link C++ class genfit::AtSpacepointMeasurement + ;
#pragma link C++ class AtFITTER::AtFitter + ;
#pragma link C++ class AtFITTER::AtGenfit + ;
#pragma link C++ namespace AtFITTER;
#pragma link C++ class AtFitterTask + ;

/* Tasks in AtReconstruction */
#pragma link C++ class AtPSAtask + ;
#pragma link C++ class AtPRAtask + ;
#pragma link C++ class AtRansacTask + ;
#pragma link C++ class AtDataReductionTask + ;
#pragma link C++ class AtSpaceChargeCorrectionTask + ;

#endif
