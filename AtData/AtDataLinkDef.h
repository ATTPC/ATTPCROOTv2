
#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ class AtPad + ;
#pragma link C++ class AtAuxPad + ;
#pragma link C++ class AtPadFFT + ;
#pragma link C++ class AtPadCharge + ;
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

#pragma link C++ class AtPatterns::AtPattern + ;
#pragma link C++ class AtPatterns::AtPatternLine + ;
#pragma link C++ class AtPatterns::AtPatternRay + ;
#pragma link C++ class AtPatterns::AtPatternCircle2D + ;
#pragma link C++ class AtPatterns::AtPatternY + ;
#pragma link C++ enum AtPatterns::PatternType;
#pragma link C++ function AtPatterns::CreatePattern;

#endif
