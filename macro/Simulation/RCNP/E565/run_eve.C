
void run_eve()
{
   //TString InputDataPath = "./digiFiles/output_digi_rcnp_13Be_p_0.0_40.0_600Torr_9mmBinning.root";
   //TString InputDataPath = "/data/ATTPCROOTv2_results/E565/Simulation/digiFiles/output_digi_rcnp_13Be_p_0.0_40.0_500Torr_9mmBinning.root";
   //TString InputDataPath = "/data/ATTPCROOTv2_results/E565/Simulation/digiFiles/output_digi_rcnp_13Be_p_0.0_40.0_600Torr_9mmBinning.root";
   //TString InputDataPath = "/data/ATTPCROOTv2_results/E565/Simulation/digiFiles/output_digi_rcnp_13Be_p_0.0_40.0_700Torr_9mmBinning.root";
   TString InputDataPath = "/data/ATTPCROOTv2_results/E565/Simulation/digiFiles/output_digi_rcnp_13Be_p_0.0_40.0_600Torr_9mmBinning_2_53MeV.root";
   TString OutputDataPath = "./output.reco_display.root";
   std::cout << "Opening: " << InputDataPath << std::endl;

   TString attpcrootPath = gSystem->Getenv("VMCWORKDIR");
   //TString geoFile = "RCNP_ATTPC_500torr_geomanager.root";
   TString geoFile = "RCNP_ATTPC_600torr_geomanager.root";
   //TString geoFile = "RCNP_ATTPC_700torr_geomanager.root";
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
