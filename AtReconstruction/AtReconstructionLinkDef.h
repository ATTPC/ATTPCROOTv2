#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ class GETDecoder + ;
#pragma link C++ class GETFrame + ;
#pragma link C++ class GETMath + ;
#pragma link C++ class GETPlot + ;

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

#pragma link C++ class AtCore + ;
#pragma link C++ class AtCore2 + ;
#pragma link C++ class AtHDFParser + ;
#pragma link C++ class AtPad + ;
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
#pragma link C++ class AtPedestal + ;
#pragma link C++ class AtTrack + ;

#pragma link C++ class AtDecoderTask + ;
#pragma link C++ class AtDecoder2Task + ;

#pragma link C++ class AtPSA + ;
#pragma link C++ class AtPSASimple + ;
#pragma link C++ class AtPSASimple2 + ;
#pragma link C++ class AtPSAProto + ;
#pragma link C++ class AtPSAProtoFull + ;
#pragma link C++ class AtPSAFilter + ;
#pragma link C++ class AtPSAFull + ;
#pragma link C++ class AtCalibration + ;

#pragma link C++ class AtHoughSpace + ;
#pragma link C++ class AtHoughSpaceLine + ;
#pragma link C++ class AtHoughSpaceCircle + ;

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

//#ifdef BUILD_PCL
//#pragma link C++ class AtHoughSpaceLine3D+;
//#pragma link C++ class AtHoughSpaceLine3D::Sphere+;
//#pragma link C++ struct vector3D;
//#endif

#pragma link C++ class AtPhiReco + ;
#pragma link C++ class AtPhiRecoSimple + ;
#pragma link C++ class AtPhiRecoTriple + ;

#pragma link C++ class VMEDecoder + ;
#pragma link C++ class VMECore + ;
#pragma link C++ class AtRawIC + ;
#pragma link C++ class VMERawEvent;

#pragma link C++ class AtPSAtask + ;
#pragma link C++ class AtPhiRecoTask + ;

#pragma link C++ class AtHierarchicalClusteringTask + ;
//#pragma link C++ class AtTrackFinderHCTask+;
#pragma link C++ class AtPRAtask + ;
//#pragma link C++ class AtTrackFinderHC+;

#pragma link C++ class AtHoughTask + ;
#pragma link C++ class AtVMEUnpackTask + ;
#pragma link C++ class AtAnalysisTask + ;
#pragma link C++ class AtRansacTask + ;

#pragma link C++ class AtMinimization + ;
#pragma link C++ class AtMCMinimization + ;
#pragma link C++ class AtMCQMinimization + ;

#pragma link C++ class AtAnalysis + ;
#pragma link C++ class AtProtoAnalysis + ;
#pragma link C++ class Atd2HeAnalysis + ;
#pragma link C++ class AtTrackingAnalysis + ;
#pragma link C++ struct AtHoughSpaceCircle::FitPar + ;
#pragma link C++ struct AtTrack::FitPar + ;
#pragma link C++ class AtHDFParserTask + ;

//#pragma link C++ function AtHoughSpaceLine::CalcGenHoughSpace<AtEvent*>(GenHough event);

#endif
