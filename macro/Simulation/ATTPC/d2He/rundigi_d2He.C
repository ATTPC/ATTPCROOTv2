void rundigi_d2He(Int_t runNumber = 0, Double_t ExEje = 0)
{
   TString mcFile = Form("/mnt/analysis/e18008/rootAna/giraud/simulation/g4/attpcsim_d2He_run%d_Ex%d_testUpdates.root",
                         runNumber, (Int_t)ExEje); // attpcsim_d2He_14O_07atm_100000.root

   TString scriptfile = "Lookup20150611.xml";
   TString paramFile = "ATTPC.d2HeSim.par";
   TString dir = getenv("VMCWORKDIR");
   TString digiParFile = dir + "/parameters/" + paramFile;
   TString mapParFile = dir + "/scripts/" + scriptfile;

   TString iniMapFile = dir + "/resources/coordmap_inhi.txt";

   TFile *fileSim = new TFile(mcFile, "READ");
   TTree *treeSim = (TTree *)fileSim->Get("cbmsim");
   Int_t nEvents = treeSim->GetEntries();
   TString str_nEvents = std::to_string(nEvents);
   delete treeSim;
   delete fileSim;

   TString outputFile = Form("/mnt/analysis/e18008/rootAna/giraud/simulation/digi/attpcdigi_d2He_" + str_nEvents +
                                "_run%d_Ex%d_testUpdates.root",
                             runNumber, (Int_t)ExEje);

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();
   // ------------------------------------------------------------------------

   // __ Run ____________________________________________
   FairRunAna *fRun = new FairRunAna();
   FairFileSource *source = new FairFileSource(mcFile);
   fRun->SetSource(source);
   fRun->SetOutputFile(outputFile);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);

   // Create the detector map to pass to the simulation
   auto mapping = std::make_shared<AtTpcMap>();
   mapping->ParseXMLMap(mapParFile.Data());
   mapping->GeneratePadPlane();
   mapping->ParseInhibitMap(iniMapFile, AtMap::InhibitType::kTotal);

   // __ AT digi tasks___________________________________
   AtClusterizeLineTask *clusterizer = new AtClusterizeLineTask();
   // AtClusterizeTask *clusterizer = new AtClusterizeTask();
   clusterizer->SetPersistence(kFALSE);

   AtPulseTask *pulse = new AtPulseTask();
   pulse->SetPersistence(kTRUE);
   pulse->SetPersistenceAtTpcPoint(kTRUE);
   pulse->SetMap(mapping);
   pulse->SetSaveMCInfo();

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(0);
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(kTRUE);

   AtRansacTask *ransacTask = new AtRansacTask();
   ransacTask->SetPersistence(kTRUE);
   ransacTask->SetVerbose(kFALSE);
   ransacTask->SetDistanceThreshold(8.0);
   ransacTask->SetMinHitsLine(10);
   // in AtRansacTask parttern type set to line : auto patternType = AtPatterns::PatternType::kLine;
   ransacTask->SetAlgorithm(3); // 1=Homemade Ransac (default); 2=Homemade Mlesac; 3=Homemade Lmeds;
   ransacTask->SetRanSamMode(
      0); // SampleMethod { kUniform = 0, kChargeWeighted = 1, kGaussian = 2, kWeightedGaussian = 3, kWeightedY = 4 };
   ransacTask->SetChargeThreshold(0);

   // RandTask->SetVertexMode(1);

   // fRun -> AddTask(fDecoderTask);
   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(psaTask);
   //    fRun -> AddTask(trigTask);
   fRun->AddTask(ransacTask);

   //  __ Init and run ___________________________________
   fRun->Init();

   timer.Start();
   // fRun->Run(0, 20);
   fRun->Run(0, nEvents);
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
