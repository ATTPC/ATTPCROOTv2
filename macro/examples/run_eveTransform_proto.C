void run_eveTransform_proto() 
{

   TString dir = getenv("VMCWORKDIR");
   TString protomapfile = "proto.map";
   TString protomapdir = dir + "/scripts/"+ protomapfile;
 

  FairLogger *fLogger = FairLogger::GetLogger();
  fLogger -> SetLogToScreen(kTRUE);
  fLogger->SetLogVerbosityLevel("MEDIUM");

  FairRunAna *fRun= new FairRunAna();
  fRun -> SetInputFile("output_proto.root");
  fRun -> SetOutputFile("output_proto.reco_display.root");

  FairRuntimeDb* rtdb = fRun->GetRuntimeDb();
  FairParRootFileIo* parIo1 = new FairParRootFileIo();
  parIo1->open("param.dummy_proto.root");
  rtdb->setFirstInput(parIo1);

  FairRootManager* ioman = FairRootManager::Instance();

  ATEventManager *eveMan = new ATEventManager();
  ATEventDrawTask* eve = new ATEventProtoDrawTask();
  eve->SetGeoOption("Prototype"); // Options: "ATTPC" - "Prototype"
  eve->SetProtoMap(protomapdir.Data());

  eveMan->AddTask(eve);
  eveMan->Init();                    
}
