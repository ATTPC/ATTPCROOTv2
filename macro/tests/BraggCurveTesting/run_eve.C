
void run_eve()
{
   TString InputDataPath = "./output_digi_rcnp_13Be_p_0.0_180.0_hole_550Torr.root";
   TString OutputDataPath = "./output.reco_display.root";
   std::cout << "Opening: " << InputDataPath << std::endl;

   TString attpcrootPath = gSystem->Getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_C3D8_550torr_geomanager.root";
   TString mapFile = "Lookup20150611.xml";
   TString GeoDataPath = attpcrootPath + "/geometry/" + geoFile;
   TString mapDir = attpcrootPath + "/scripts/" + mapFile;

   FairRunAna *fRun = new FairRunAna();
   FairRootFileSink *sink = new FairRootFileSink(OutputDataPath);
   FairFileSource *source = new FairFileSource(InputDataPath);
   fRun->SetSource(source);
   fRun->SetSink(sink);
   fRun->SetGeomFile(GeoDataPath);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parIo = new FairParRootFileIo();
   rtdb->setFirstInput(parIo);

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(mapDir.Data());
   AtViewerManager *eveMan = new AtViewerManager(fMap);

   auto tabMain = std::make_unique<AtTabMain>();
   tabMain->SetMultiHit(100);

   auto tabBraggCurve = std::make_unique<AtTabBraggCurve>();
   tabBraggCurve->SetMultiHit(100);

   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tabBraggCurve));

   eveMan->Init();

   std::cout << " Finished init. " << std::endl;
}
