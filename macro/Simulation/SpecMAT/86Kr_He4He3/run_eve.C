void run_eve(TString InputDataFile = "output_digi.root", TString OutputDataFile = "output.reco_display.root",
             TString unpackDir = "Simulation/SpecMAT/86Kr_He4He3/")
{
   FairLogger *fLogger = FairLogger::GetLogger();
   fLogger->SetLogToScreen(kTRUE);
   fLogger->SetLogVerbosityLevel("MEDIUM");
   TString dir = getenv("VMCWORKDIR");

   TString scriptfile = "LookupSpecMATnoScintHisto.xml";
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString geoFile = "SpecMAT_He1Bar.root";

   TString InputDataPath = dir + "/macro/" + unpackDir + InputDataFile;
   TString OutputDataPath = dir + "/macro/" + unpackDir + OutputDataFile;
   TString GeoDataPath = dir + "/geometry/" + geoFile;

   FairRunAna *fRun = new FairRunAna();
   fRun->SetSource(new FairFileSource(InputDataPath));
   fRun->SetSink(new FairRootFileSink(OutputDataPath));
   fRun->SetGeomFile(GeoDataPath);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parIo1 = new FairParRootFileIo();
   // parIo1->open("param.dummy.root");
   rtdb->setFirstInput(parIo1);

   // Create the map that will be pased to tasks that require it
   auto fMapPtr = std::make_shared<AtSpecMATMap>(3174);
   fMapPtr->ParseXMLMap(scriptdir.Data());
   fMapPtr->GeneratePadPlane();

   FairRootManager *ioman = FairRootManager::Instance();

   AtEventManager *eveMan = new AtEventManager();
   AtEventDrawTask *eve = new AtEventDrawTask();
   eve->Set3DHitStyleBox();
   eve->SetMultiHit(100); // Set the maximum number of multihits in the visualization
   eve->SetSaveTextData();
   eve->SetMap(fMapPtr);
   eve->UnpackHoughSpace();

   eveMan->AddTask(eve);
   eveMan->Init();
}
