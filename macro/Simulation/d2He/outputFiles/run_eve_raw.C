//void run_eve(TString  InputDataFile = "/home/juan/FairRoot/ATTPCROOTv2_simon/run_2016_0026.root",TString  OutputDataFile = "output.reco_display.root")
//void run_eve(TString  InputDataFile = "run_unpacked_0002_new.root",TString  OutputDataFile = "output.reco_display.root")
// void run_eve(TString  InputDataFile = "/mnt/simulations/ceclub/giraud/attpc/ATTPCROOTv2/macro/Simulation/d2He/Analyis_d2He/merged_run0002_48Ca.root",TString  OutputDataFile = "output.reco_display.root"){
void run_eve_raw(int runNumberS800=2060, int runNumberATTPC=60){

  //TString InputDataFile = TString::Format("/home/juan/NSCL/run_%04d_%04d.root", runNumberS800, runNumberATTPC);
  //TString InputDataFile = "salida_ran_82.root";
  //TString InputDataFile = "/home/juan/NSCL/run_2063_0063_thre100.root";
  //TString InputDataFile = "/home/juan/NSCL/run_2063_0063_nothres.root";
  //TString InputDataFile = "/home/juan/NSCL/run_2063_0063_2thres.root";
  //TString InputDataFile = "/home/juan/NSCL/run_2063_0063_bg80TSpec.root";
  //TString InputDataFile = "/home/juan/NSCL/run_2063_0063_peak.root";
  TString InputDataFile = "attpcdigi_d2He_test.root"; //Tan exp
  // TString InputDataFile = TString::Format("/mnt/analysis/e18008/rootATTPC/run_unpacked_%04d.root", runNumberATTPC);
  TString  OutputDataFile = "output.reco_display.root";

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


  ATEventManager *eveMan = new ATEventManager();
  ATEventDrawTask* eve = new ATEventDrawTask();
  eve->Set3DHitStyleBox();
  eve->SetMultiHit(100); //Set the maximum number of multihits in the visualization
  eve->SetSaveTextData();


  eveMan->AddTask(eve);
  eveMan->Init();
}
