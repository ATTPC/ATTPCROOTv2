// void run_eve(TString  InputDataPath = "/mnt/analysis/e18008/rootMerg/giraud/", TString  OutputDataPath =
// "output.reco_display.root")
void run_eve(TString InputDataPath = "/mnt/analysis/e18008/rootAna/giraud/simulation/digi/",
             TString OutputDataPath = "output.reco_display.root")
{
   FairLogger *fLogger = FairLogger::GetLogger();
   fLogger->SetLogToScreen(kTRUE);
   fLogger->SetLogVerbosityLevel("MEDIUM");
   TString dir = getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_v1.1_geomanager.root";
   TString mapFile = "Lookup20150611.xml";

   TString InputDataFile =
      InputDataPath +
      "attpcdigi_d2He_100_run0_Ex10_testUpdates.root"; //  /mnt/analysis/e18008/rootMerg/giraud/run_%04d_%04d_test.root
   // TString InputDataFile = InputDataPath+"run_2063_0063_test.root";
   TString OutputDataFile = OutputDataPath;
   TString GeoDataPath = dir + "/geometry/" + geoFile;
   TString mapDir = dir + "/scripts/" + mapFile;

   FairRunAna *fRun = new FairRunAna();
   FairRootFileSink *sink = new FairRootFileSink(OutputDataFile);
   FairFileSource *source = new FairFileSource(InputDataFile);
   fRun->SetSource(source);
   fRun->SetSink(sink);
   fRun->SetGeomFile(GeoDataPath);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parIo1 = new FairParRootFileIo();
   // parIo1->open("param.dummy.root");
   rtdb->setFirstInput(parIo1);

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(mapDir.Data());

   AtEventManager *eveMan = new AtEventManager();
   AtEventDrawTask *eve = new AtEventDrawTask();
   eve->SetMap(fMap);
   eve->Set3DHitStyleBox();
   eve->SetMultiHit(100); // Set the maximum number of multihits in the visualization
   // eve->SetSaveTextData();
   eve->SetDrawVertexFromLines();
   eve->SetRawEventBranch("AtRawEvent");
   eve->SetEventBranch("AtEventH");
   eve->SetMinTracksPerVertex(2);

   eveMan->AddTask(eve);
   eveMan->Init();

   std::cout << "Finished init" << std::endl;
   // eveMan->RunEvent(27);
}
