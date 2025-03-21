
void run_eve()
{
   std::cout << " Hi. " << std::endl;
   TString InputDataPath = "/media/aurio/Cris/11Li/unpackRecal/run_0007_e23031.root";
   TString OutputDataPath = "/home/aurio/fair_install/ATTPCROOTv2_fork/macro/Unpack_HDF5/e23031/output.reco_display.root";
   std::cout << "Opening: " << InputDataPath << std::endl;

   TString attpcrootPath = gSystem->Getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_H600torr_geomanager.root";
   TString mapFile = "e21018_pads_map.xml";
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

   auto tab3DBraggCurve = std::make_unique<AtTab3DBraggCurve>();
   tab3DBraggCurve->SetMultiHit(100);
   tab3DBraggCurve->SetDZ(4.);

   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tab3DBraggCurve));

   eveMan->Init();

   std::cout << " Finished init. " << std::endl;
}
