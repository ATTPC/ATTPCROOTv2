
#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ struct AtPadReference + ;
#pragma link C++ struct AtElectronicReference + ;

#pragma link C++ class AtPadBase + ;
#pragma link C++ class AtPad + ;
#pragma link C++ class AtAuxPad + ;
#pragma link C++ class AtPadFFT + ;
#pragma link C++ class AtPadArray + ;
#pragma link C++ class AtPadValue + ;
#pragma link C++ class AtPulserInfo + ;

#pragma link C++ class AtBaseEvent + ;
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
#pragma link C++ class AtFissionEvent + ;
#pragma link C++ class AtGenericTrace + ;

#pragma link C++ class AtPatterns::AtPattern + ;
#pragma link C++ class AtPatterns::AtPatternLine + ;
#pragma link C++ class AtPatterns::AtPatternRay + ;
#pragma link C++ class AtPatterns::AtPatternCircle2D + ;
#pragma link C++ class AtPatterns::AtPatternY + ;
#pragma link C++ class AtPatterns::AtPatternFission + ;
#pragma link C++ enum AtPatterns::PatternType;
#pragma link C++ function AtPatterns::CreatePattern;

#pragma link C++ class MCFitter::AtMCResult + ;
#endif
