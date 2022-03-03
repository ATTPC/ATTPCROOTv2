#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ class AtHDFParser + ;
#pragma link C++ class AtPad + ;
#pragma link C++ class AtAuxPad + ;
#pragma link C++ class AtRawEvent + ;
#pragma link C++ class AtHit + ;
#pragma link C++ class AtHitCluster + ;
#pragma link C++ struct AtHit::MCSimPoint + ;
#pragma link C++ class AtEvent + ;
#pragma link C++ class AtProtoEvent + ;
#pragma link C++ class AtProtoEventAna + ;
#pragma link C++ class AtPatternEvent + ;
#pragma link C++ class AtTrackingEventAna + ;
#pragma link C++ class AtProtoQuadrant + ;
#pragma link C++ class AtTrack + ;

#pragma link C++ class AtPSA + ;
#pragma link C++ class AtPSASimple + ;
#pragma link C++ class AtPSASimple2 + ;
#pragma link C++ class AtPSAProto + ;
#pragma link C++ class AtPSAProtoFull + ;
#pragma link C++ class AtPSAFilter + ;
#pragma link C++ class AtPSAFull + ;
#pragma link C++ class AtCalibration + ;

#pragma link C++ nestedclass;
#pragma link C++ nestedtypedef;
#pragma link C++ namespace AtRANSACN;
#pragma link C++ class AtRANSACN::AtRansac + ;
#pragma link C++ struct AtRANSACN::AtRansac::PairedLines + ;
#pragma link C++ class AtRansacMod + ;
#pragma link C++ class AtMlesacMod + ;
#pragma link C++ class AtLmedsMod + ;

#pragma link C++ class AtPATTERN::AtTrackFinderHC + ;
#pragma link C++ class AtPATTERN::AtPRA + ;
#pragma link C++ namespace AtPATTERN;
#pragma link C++ class AtRansac + ;

/* Classes that depend on Genfit2 */
#pragma link C++ class genfit::AtSpacepointMeasurement + ;
#pragma link C++ class AtFITTER::AtFitter + ;
#pragma link C++ class AtFITTER::AtGenfit + ;
#pragma link C++ namespace AtFITTER;
#pragma link C++ class AtFitterTask + ;

#pragma link C++ class AtPSAtask + ;

#pragma link C++ class AtPRAtask + ;

#pragma link C++ class AtRansacTask + ;

#pragma link C++ struct AtTrack::FitPar + ;
#pragma link C++ class AtHDFParserTask + ;

//#pragma link C++ function AtHoughSpaceLine::CalcGenHoughSpace<AtEvent*>(GenHough event);

#endif
