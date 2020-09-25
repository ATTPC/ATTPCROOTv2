//void run_eve(TString  InputDataFile = "attpcdigi_d2He_1000.root",TString  OutputDataFile = "output.reco_display.root", TString unpackDir="/Unpack_GETDecoder2/")
void run_eve(TString  InputDataFile = "attpcdigi_d2He_1e6pps_ran.root",TString  OutputDataFile = "output.reco_display.root", TString unpackDir="/Unpack_GETDecoder2/")
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
  //parIo1->open("param.dummy.root");
  rtdb->setFirstInput(parIo1);

  FairRootManager* ioman = FairRootManager::Instance();

  ATEventManager *eveMan = new ATEventManager();
  ATEventDrawTask* eve = new ATEventDrawTask();
  eve->Set3DHitStyleBox();
  eve->SetMultiHit(100); //Set the maximum number of multihits in the visualization
  eve->SetSaveTextData();
  eve->UnpackHoughSpace();
  eve->SetAlgorithm(1); // 0=PCL ransac; 1=Homemade Ransac; 2=Homemade Mlesac; 3=Homemade Lmeds; 

  //----------------------Traks and points -------------------------------------
  //FairMCTracks    *Track     = new FairMCTracks("Monte-Carlo Tracks");
  FairMCPointDraw *AtTpcPoints = new FairMCPointDraw("AtTpcPoint", kBlue, kFullSquare);




  eveMan->AddTask(eve);
  //eveMan->AddTask(Track);
  eveMan->AddTask(AtTpcPoints);
  eveMan->Init();
}
