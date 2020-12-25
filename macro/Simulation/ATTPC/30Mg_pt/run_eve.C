void run_eve(TString  InputDataFile = "output_digi_4.root",TString  OutputDataFile = "output.reco_display.root", TString unpackDir="/Simulation/30Mg_pt/")
{
  FairLogger *fLogger = FairLogger::GetLogger();
  fLogger -> SetLogToScreen(kTRUE);
  fLogger->SetLogVerbosityLevel("MEDIUM");
  TString dir = getenv("VMCWORKDIR");
  TString geoFile = "AT3PC_3H_5torr_geomanager.root";


  TString InputDataPath = dir + "/macro/"+ unpackDir + InputDataFile;
  TString OutputDataPath = dir + "/macro/"+ unpackDir + OutputDataFile;
  TString GeoDataPath = dir + "/geometry/" + geoFile;

  FairRunAna *fRun= new FairRunAna();
  fRun -> SetInputFile(InputDataPath);
  fRun -> SetOutputFile(OutputDataPath);
  fRun -> SetGeomFile(GeoDataPath);

  FairRuntimeDb* rtdb = fRun->GetRuntimeDb();
  FairParRootFileIo* parIo1 = new FairParRootFileIo();
  //parIo1->open("param.dummy.root");
  rtdb->setFirstInput(parIo1);

  FairRootManager* ioman = FairRootManager::Instance();

  ATEventManager *eveMan = new ATEventManager();
  ATEventDrawTask* eve = new ATEventDrawTask();
  eve->Set3DHitStyleBox();
  eve->SetMultiHit(100); //Set the maximum number of multihits in the visualization
  eve->SetSaveTextData();
  eve->UnpackHoughSpace();

  eveMan->AddTask(eve);
  eveMan->Init();
}
