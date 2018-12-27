#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ class GETDecoder+;
#pragma link C++ class GETFrame+;
#pragma link C++ class GETMath+;
#pragma link C++ class GETPlot+;

#pragma link C++ class GETDecoder2+;
#pragma link C++ class GETFrameInfo+;
#pragma link C++ class GETHeaderBase+;
#pragma link C++ class GETBasicFrameHeader+;
#pragma link C++ class GETLayerHeader+;
#pragma link C++ class GETTopologyFrame+;
#pragma link C++ class GETBasicFrame+;
#pragma link C++ class GETCoboFrame+;
#pragma link C++ class GETLayeredFrame+;
#pragma link C++ class GETFileChecker+;
#pragma link C++ class GETMath2+;

#pragma link C++ class ATCore+;
#pragma link C++ class ATCore2+;
#pragma link C++ class ATHDFParser+;
#pragma link C++ class ATPad+;
#pragma link C++ class ATRawEvent+;
#pragma link C++ class ATHit+;
#pragma link C++ class ATEvent+;
#pragma link C++ class ATProtoEvent+;
#pragma link C++ class ATProtoEventAna+;
#pragma link C++ class ATPatternEvent+;
#pragma link C++ class ATTrackingEventAna+;
#pragma link C++ class ATProtoQuadrant+;
#pragma link C++ class ATPedestal+;
#pragma link C++ class ATTrack+;

#pragma link C++ class ATDecoderTask+;
#pragma link C++ class ATDecoder2Task+;

#pragma link C++ class ATPSA+;
#pragma link C++ class ATPSASimple+;
#pragma link C++ class ATPSASimple2+;
#pragma link C++ class ATPSAProto+;
#pragma link C++ class ATPSAProtoFull+;
#pragma link C++ class ATPSAFilter+;
#pragma link C++ class ATPSAFull+;
#pragma link C++ class ATCalibration+;

#pragma link C++ class ATHoughSpace+;
#pragma link C++ class ATHoughSpaceLine+;
#pragma link C++ class ATHoughSpaceCircle+;

#pragma link C++ nestedclass;
#pragma link C++ nestedtypedef;
#pragma link C++ namespace ATRANSACN;
#pragma link C++ class ATRANSACN::ATRansac+;
#pragma link C++ struct ATRANSACN::ATRansac::PairedLines+;

#pragma link C++ class ATPATTERN::ATTrackFinderHC+;
#pragma link C++ class ATPATTERN::ATPRA+;
#pragma link C++ namespace ATPATTERN;
//#pragma link C++ class ATRansac+;


//#ifdef BUILD_PCL
//#pragma link C++ class ATHoughSpaceLine3D+;
//#pragma link C++ class ATHoughSpaceLine3D::Sphere+;
//#pragma link C++ struct vector3D;
//#endif

#pragma link C++ class ATPhiReco+;
#pragma link C++ class ATPhiRecoSimple+;
#pragma link C++ class ATPhiRecoTriple+;

#pragma link C++ class VMEDecoder+;
#pragma link C++ class VMECore+;
#pragma link C++ class ATRawIC+;
#pragma link C++ class VMERawEvent;


#pragma link C++ class ATPSATask+;
#pragma link C++ class ATPhiRecoTask+;

#pragma link C++ class ATHierarchicalClusteringTask+;
//#pragma link C++ class ATTrackFinderHCTask+;
#pragma link C++ class ATPRATask+;
//#pragma link C++ class ATTrackFinderHC+;



#pragma link C++ class ATHoughTask+;
#pragma link C++ class ATVMEUnpackTask+;
#pragma link C++ class ATAnalysisTask+;
#pragma link C++ class ATRansacTask+;

#pragma link C++ class ATMinimization+;
#pragma link C++ class ATMCMinimization+;
#pragma link C++ class ATMCQMinimization+;

#pragma link C++ class ATAnalysis+;
#pragma link C++ class ATProtoAnalysis+;
#pragma link C++ class ATd2HeAnalysis+;
#pragma link C++ class ATTrackingAnalysis+;
#pragma link C++ struct ATHoughSpaceCircle::FitPar+;
#pragma link C++ struct ATTrack::FitPar+;
#pragma link C++ class ATHDFParserTask+;
//#pragma link C++ function ATHoughSpaceLine::CalcGenHoughSpace<ATEvent*>(GenHough event);



#endif
