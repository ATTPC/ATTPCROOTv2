void rundisp_d2He(TString  InputDataFile = "digi_test_decay.root",TString  OutputDataFile = "display_test.root", TString unpackDir="/Unpack_GETDecoder2/")
//void runeve_d2He(TString  InputDataFile = "digi_bench_10k.root",TString  OutputDataFile = "Miles_TEST.root", TString unpackDir="/Unpack_GETDecoder2/")
//void runeve_d2He(TString  InputDataFile = "/mnt/data/fair_install/giraud_ATTPCROOTv2/macro/Simulation/d2He/z50cm_x10mm_15mrad_TEST.root",TString  OutputDataFile = "Miles_TEST.root", TString unpackDir="/Unpack_GETDecoder2/")
{
//void runeve_d2He(TString  InputDataFile = "output_digi_20k.root",TString  OutputDataFile = "output.reco_display.root", TString unpackDir="/Unpack_GETDecoder2/")

  FairLogger *fLogger = FairLogger::GetLogger();
  fLogger -> SetLogToScreen(kTRUE);
  fLogger->SetLogVerbosityLevel("MEDIUM");
  TString dir = getenv("VMCWORKDIR");
  TString geoFile = "ATTPC_d2He_07atm_geomanager.root";
  //TString geoFile = "ATTPC_v1.1.root";


  TString InputDataPath =  InputDataFile;
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
  //eve->SetMultiHit(100); //Set the maximum number of multihits in the visualization
  ///////eve->SetSaveTextData();
  //eve->UnpackHoughSpace();

  eveMan->AddTask(eve);
  eveMan->Init();
}
