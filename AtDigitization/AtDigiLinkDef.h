#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ class AtClusterize - !;
#pragma link C++ class AtClusterizeLine - !;
#pragma link C++ class AtClusterizeTask + ;
#pragma link C++ class AtClusterizeLineTask + ;

#pragma link C++ class AtPulse - !;
#pragma link C++ class AtPulseLine - !;
#pragma link C++ class AtPulseTask + ;
#pragma link C++ class AtPulseTaskGADGET + ;
#pragma link C++ class AtPulseLineTask + ;

#pragma link C++ class AtSimulatedPoint + ;
#pragma link C++ class AtSimulatedLine + ;

#pragma link C++ class AtTrigger + ;
#pragma link C++ class AtTriggerTask + ;
#pragma link C++ class AtVectorResponse - ;
#pragma link C++ class AtTestSimulation + ;
#pragma link C++ class AtSimpleSimulation - !;
#endif
