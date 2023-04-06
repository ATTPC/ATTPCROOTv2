void simpleSim()
{
   TString inOutDir = "./data/";
   TString outputFile = inOutDir + "output_digi.root";
   TString scriptfile = "e12014_pad_mapping.xml";
   TString paramFile = "ATTPC.e12014.par";
   TString geoFileName = "ATTPC_He1bar_geomanager.root";

   TString dir = getenv("VMCWORKDIR");

   // Create the full parameter file paths
   TString digiParFile = dir + "/parameters/" + paramFile;
   TString mapParFile = dir + "/scripts/" + scriptfile;
   TString geoFile = dir + "/geometry/" + geoFileName;
   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;

   // ------------------------------------------------------------------------

   // __ Run ____________________________________________
   FairRunAna *fRun = new FairRunAna();
   fRun->SetOutputFile(outputFile);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);

   // Create the detector map to pass to the simulation
   auto mapping = std::make_shared<AtTpcMap>();
   mapping->ParseXMLMap(mapParFile.Data());
   mapping->GeneratePadPlane();
   // mapping->ParseInhibitMap("./data/inhibit.txt", AtMap::InhibitType::kTotal);

   // Create underlying simulation class
   auto sim = std::make_unique<AtSimpleSimulation>(geoFile.Data());

   // Create and load energy loss models
   auto eloss = std::make_shared<AtTools::AtELossTable>();
   eloss->LoadSrimTable("./../PbinHeFull.txt");
   sim->AddModel(82, 208, eloss);

   AtTestSimulation *simTask = new AtTestSimulation(std::move(sim));

   AtClusterizeLineTask *clusterizer = new AtClusterizeLineTask();
   clusterizer->SetPersistence(kFALSE);

   // AtPulseTask *pulse = new AtPulseTask();
   AtPulseLineTask *pulse = new AtPulseLineTask();
   pulse->SetPersistence(kTRUE);
   pulse->SetMap(mapping);
   pulse->SetSaveMCInfo();

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(0);

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(kTRUE);

   AtRansacTask *ransacTask = new AtRansacTask();
   ransacTask->SetPersistence(kTRUE);
   ransacTask->SetVerbose(kFALSE);
   ransacTask->SetDistanceThreshold(20.0);
   ransacTask->SetMinHitsLine(10);

   fRun->AddTask(simTask);
   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(psaTask);

   //  __ Init and run ___________________________________
   fRun->Init();

   timer.Start();
   // fRun->Run(0, 20001);
   fRun->Run(0, 2);
   timer.Stop();

   std::cout << std::endl << std::endl;
   std::cout << "Macro finished succesfully." << std::endl << std::endl;
   // -----   Finish   -------------------------------------------------------

   Double_t rtime = timer.RealTime();
   Double_t ctime = timer.CpuTime();
   cout << endl;
   cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
   cout << endl;
   // ------------------------------------------------------------------------
}
