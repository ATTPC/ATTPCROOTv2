void run_eve_proto(TString  InputDataFile = "output_proto.root",TString  OutputDataFile = "output_proto.reco_display.root", TString unpackDir="/Unpack_GETDecoder2/")
{

   TString dir = getenv("VMCWORKDIR");
   TString protomapfile = "proto.map";
   TString protomapdir = dir + "/scripts/"+ protomapfile;
   TString geoFile = "ATTPC_Proto_v1.0_geomanager.root";

   TString InputDataPath = dir + "/macro/"+ unpackDir + InputDataFile;
   TString OutputDataPath = dir + "/macro/"+ unpackDir + OutputDataFile;
   TString GeoDataPath = dir + "/geometry/" + geoFile;


  FairLogger *fLogger = FairLogger::GetLogger();
  fLogger -> SetLogToScreen(kTRUE);
  fLogger->SetLogVerbosityLevel("MEDIUM");

  FairRunAna *fRun= new FairRunAna();
  fRun -> SetInputFile(InputDataPath);
  fRun -> SetOutputFile(OutputDataPath);
  fRun -> SetGeomFile(GeoDataPath);

  FairRuntimeDb* rtdb = fRun->GetRuntimeDb();
  FairParRootFileIo* parIo1 = new FairParRootFileIo();
  //parIo1->open("param.dummy_proto.root");
  rtdb->setFirstInput(parIo1);

  FairRootManager* ioman = FairRootManager::Instance();

  ATEventManager *eveMan = new ATEventManager();
  ATEventDrawTask* eve = new ATEventProtoDrawTask();
  //eve->UnpackHoughSpace(); // Allows extract the Hough Space from root file. Disable it for a faster performance
  eve->Set3DHitStyleBox();
  //eve->SetSaveTextData(); //To save the waveform of an event in a txt file when pressig the button (NB: Draw all pads MUST be enabled as well)
  // eve->Set3DHitStyleBar();
  eve->SetGeoOption("Prototype"); // Options: "ATTPC" - "Prototype"
  eve->SetProtoMap(protomapdir.Data());
  eve->UnpackHoughSpace();

  eveMan->AddTask(eve);
  eveMan->Init();
}
