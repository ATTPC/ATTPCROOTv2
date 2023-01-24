void run_eve_e20020(TString InputDataFileName = "run_0160")
{

   TString InputDataFile = "./data/" + InputDataFileName + ".root";

   TString OutputDataFile = "./data/" + InputDataFileName + ".reco_display.root";
   TString unpackDir = "/Unpack_HDF5/e20020/";

   TString dir = getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_He1bar_v2_geomanager.root";
   TString mapFile = "e12014_pad_mapping.xml";

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
   // parIo1->open("param.dummy.root");
   rtdb->setFirstInput(parIo1);

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(mapDir.Data());

   AtEventManager *eveMan = new AtEventManager();
   AtEventDrawTask *eve = new AtEventDrawTask();
   eve->SetMap(fMap);
   eve->Set3DHitStyleBox();
   eve->SetMultiHit(100); // Set the maximum number of multihits in the visualization
   eve->SetSaveTextData();

   eveMan->AddTask(eve);
   eveMan->Init();
}
