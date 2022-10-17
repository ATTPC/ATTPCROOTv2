void run_eve(TString InputDataPath = "/mnt/analysis/e18008/rootMerg/giraud/",
             TString OutputDataPath = "output.reco_display.root")
{
   FairLogger *fLogger = FairLogger::GetLogger();
   fLogger->SetLogToScreen(kTRUE);
   fLogger->SetLogVerbosityLevel("MEDIUM");
   TString dir = getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_v1.1_geomanager.root";
   TString mapFile = "Lookup20150611.xml";

   TString InputDataFile = InputDataPath + "run_2278_0278_test15.root";
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
   // eve->SetRawEventBranch("AtRawEvent");//AtRawEventFiltered
   eve->SetEventBranch("AtEventH"); // AtEventFiltered
   eve->SetMinTracksPerVertex(2);

   // use the same parameters as in the unpack or analysis macro
   std::vector<Double_t> S800MTDCObjCorr;
   S800MTDCObjCorr.push_back(70.);
   S800MTDCObjCorr.push_back(0.0085);
   std::vector<Double_t> S800MTDCObjRange;
   S800MTDCObjRange.push_back(-120);
   S800MTDCObjRange.push_back(-20);
   std::vector<Double_t> S800MTDCXfRange;
   S800MTDCXfRange.push_back(160);
   S800MTDCXfRange.push_back(240);
   eveMan->SetMTDCXfRange(S800MTDCXfRange);
   eveMan->SetMTDCObjRange(S800MTDCObjRange);
   eveMan->SetTofObjCorr(S800MTDCObjCorr);

   eveMan->AddTask(eve);
   eveMan->Init();

   std::cout << "Finished init" << std::endl;
}
