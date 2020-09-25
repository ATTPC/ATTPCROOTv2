void run_eve(TString  InputDataFile = "run_unpacked_0002_new.root",TString  OutputDataFile = "output.reco_display.root")
{
  FairLogger *fLogger = FairLogger::GetLogger();
  fLogger -> SetLogToScreen(kTRUE);
  fLogger->SetLogVerbosityLevel("MEDIUM");
  TString dir = getenv("VMCWORKDIR");
  TString geoFile = "ATTPC_v1.1_geomanager.root";



  TString InputDataPath = InputDataFile;
  TString OutputDataPath = OutputDataFile;
  TString GeoDataPath = dir + "/geometry/" + geoFile;

  FairRunAna *fRun= new FairRunAna();
  fRun -> SetInputFile(InputDataPath);
  fRun -> SetOutputFile(OutputDataPath);
  fRun -> SetGeomFile(GeoDataPath);

  FairRuntimeDb* rtdb = fRun->GetRuntimeDb();
  FairParRootFileIo* parIo1 = new FairParRootFileIo();
  rtdb->setFirstInput(parIo1);

  FairRootManager* ioman = FairRootManager::Instance();

  ATEventManagerS800 *eveMan = new ATEventManagerS800();
  ATEventDrawTaskS800* eve = new ATEventDrawTaskS800();
  //eve->Set3DHitStyleBox();
  eve->SetMultiHit(100); //Set the maximum number of multihits in the visualization
  eve->SetSaveTextData();
  //eve->UnpackHoughSpace();
  eve->SetAlgorithm(3); // 0=PCL ransac; 1=Homemade Ransac; 2=Homemade Mlesac; 3=Homemade Lmeds;




  eveMan->AddTask(eve);
  eveMan->Init();
}
