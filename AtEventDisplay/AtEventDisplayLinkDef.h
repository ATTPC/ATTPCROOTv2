#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ class AtEventManager + ;
#pragma link C++ class AtViewerManager + ;
#pragma link C++ class AtEventManagerProto + ;
#pragma link C++ class AtEventDrawTask + ;
#pragma link C++ class AtEventDrawTaskProto + ;
#pragma link C++ class AtEventManagerS800 + ;
#pragma link C++ class AtEventDrawTaskS800 + ;

#pragma link C++ class AtTabBase + ;
#pragma link C++ class AtTabCanvas + ;
#pragma link C++ class AtTabMain + ;
#pragma link C++ class AtTabPad + ;
#pragma link C++ class AtTabMacro + ;

#pragma link C++ class AtTabInfoBase - !;
#pragma link C++ class AtTabInfo - !;
#pragma link C++ class AtTabInfoTree - !;
// clang-format off
// Removing these because ROOT is struggling to properly include the headers to generate the dictionary
//#pragma link C++ class AtTabInfoFairRoot<AtEvent>-!;
//#pragma link C++ class AtTabInfoFairRoot<AtRawEvent>-!;
//#pragma link C++ class AtTabInfoFairRoot<AtPatternEvent>-!;
//#pragma link C++ class AtTabInfoHiRAEVT<HTMusicIC>-!;
// clang-format on

#pragma link C++ class AtSidebarAddon + ;
#pragma link C++ class AtSidebarInfoMacro + ;
#pragma link C++ class AtSidebarPSA + ;
#pragma link C++ class AtSidebarPSADeconv + ;
#pragma link C++ class AtSidebarPSAIterDeconv + ;

#endif
