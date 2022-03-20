#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ class GETDecoder2 + ;
#pragma link C++ class GETFrameInfo + ;
#pragma link C++ class GETHeaderBase + ;
#pragma link C++ class GETBasicFrameHeader + ;
#pragma link C++ class GETLayerHeader + ;
#pragma link C++ class GETTopologyFrame + ;
#pragma link C++ class GETBasicFrame + ;
#pragma link C++ class GETCoboFrame + ;
#pragma link C++ class GETLayeredFrame + ;
#pragma link C++ class GETFileChecker + ;
#pragma link C++ class GETMath2 + ;

#pragma link C++ class AtCore2 + ;
#pragma link C++ class AtPedestal + ;
#pragma link C++ class AtCoreSpecMAT + ;
#pragma link C++ class AtHDFParser + ;

#pragma link C++ class AtDecoder2Task + ;
#pragma link C++ class AtDecoderSpecMATTask + ;

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

#pragma link C++ class AtHDFParserTask + ;
#pragma link C++ class AtDataReductionTask + ;

//#pragma link C++ function AtHoughSpaceLine::CalcGenHoughSpace<AtEvent*>(GenHough event);

#endif
