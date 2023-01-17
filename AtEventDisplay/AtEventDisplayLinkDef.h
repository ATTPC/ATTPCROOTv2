#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ class AtEventManager + ;
#pragma link C++ class AtEventManagerNew + ;
#pragma link C++ class AtEventManagerProto + ;
#pragma link C++ class AtTabTask + ;
#pragma link C++ class AtEventDrawTask + ;
#pragma link C++ class AtEventDrawTaskProto + ;
#pragma link C++ class AtEventManagerS800 + ;
#pragma link C++ class AtEventDrawTaskS800 + ;

#pragma link C++ class AtTabBase + ;
#pragma link C++ class AtTabMain + ;
#pragma link C++ class AtTabPad + ;
#pragma link C++ class AtTabMacro + ;

#pragma link C++ class AtTabInfoBase - !;
#pragma link C++ class AtTabInfo - !;
// clang-format off
#pragma link C++ class AtTabInfoFairRoot<AtEvent>-!;
#pragma link C++ class AtTabInfoFairRoot<AtRawEvent>-!;
#pragma link C++ class AtTabInfoFairRoot<AtPatternEvent>-!;
#pragma link C++ class AtTabInfoHiRAEVT<HTMusicIC>-!;
// clang-format on

#endif
