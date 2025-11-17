void run_eve(int runNum = 2009, TString OutputDataFile = "./run_1003_display.root")
{
  TString InputDataFile = TString::Format("/data/ATTPCROOTv2_results/E534/UnpackerOutput/run_%04d.root", runNum);
  std::cout << "Opening: " << InputDataFile << std::endl;

   TString dir = getenv("VMCWORKDIR");
   TString geoFile = "RCNP_ATTPC_494_3torr_geomanager.root";
   TString mapFile = "rcnp_map.xml";

   TString InputDataPath = InputDataFile;
   TString OutputDataPath = OutputDataFile;
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
   rtdb->setFirstInput(parIo1);

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(mapDir.Data());
   AtViewerManager *eveMan = new AtViewerManager(fMap);

   auto tabMain = std::make_unique<AtTabMain>();
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization

   //auto tabBraggCurve = std::make_unique<AtTabBraggCurve>();
   //tabBraggCurve->SetMultiHit(100);

   //auto tabSiArray = std::make_unique<AtTabPad>();
   //tabSiArray->DrawAuxADC("Si_10_1_1_0", 0, 0);

   eveMan->AddTab(std::move(tabMain));
   //eveMan->AddTab(std::move(tabBraggCurve));
   //eveMan->AddTab(std::move(tabSiArray));

   eveMan->Init();

   std::cout << "Finished init" << std::endl;
}
