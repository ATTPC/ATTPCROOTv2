void run_eve_new( TString OutputDataFile = "output.reco_display.root",
             TString unpackDir = "Simulation/GADGET/")
{  TString InputDataFile;
   TString sim;
   std::cin<<"Charge Dispersion (CD) or Diffusion (Diff)? :"<<sim<<std::endl;
   if(sim == "CD"){
      InputDataFile = "./data/CD_digi.root";
   }
   else if(sim == "Diff"){
      InputDataFile = "./diff_digi.root";
   }
   else{
      std::cout<<"Please enter CD or Diff"<<std::endl;
   }

   FairLogger *fLogger = FairLogger::GetLogger();
   fLogger->SetLogToScreen(kTRUE);
   fLogger->SetLogVerbosityLevel("MEDIUM");
   TString dir = getenv("VMCWORKDIR");
   // TString geoFile = "ATTPC_v1.1_geomanager.root";
   TString geoFile = "GADGET_II_lp_geomanager.root";
   TString mapFile = "LookupGADGET08232021.xml";

   TString InputDataPath = dir + "/macro/" + unpackDir + InputDataFile;
   TString OutputDataPath = dir + "/macro/" + unpackDir + OutputDataFile;
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

   auto fMap = std::make_shared<AtGadgetIIMap>();
   fMap->ParseXMLMap(mapDir.Data());
   AtViewerManager *eveMan = new AtViewerManager(fMap);

   auto tabMain = std::make_unique<AtTabMain>();
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization
   eveMan->AddTab(std::move(tabMain));

   auto tabPad = std::make_unique<AtTabPad>(1, 2);
   tabPad->DrawADC(0, 0);
   tabPad->DrawArrayAug("Q", 0, 1);
   eveMan->AddTab(std::move(tabPad));
   
   eveMan->Init();

   std::cout << "Finished init" << std::endl;
  
}
