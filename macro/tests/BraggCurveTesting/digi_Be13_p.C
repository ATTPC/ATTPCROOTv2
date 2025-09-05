bool reduceFunc(AtRawEvent *evt);

void digi_Be13_p(int nEvent = 10000, int subnum = 0, int angle_cm = 180)
{
   const char *filename = "13Be_p";
   double minangle, maxangle;
   minangle = 0, maxangle = angle_cm;
   TString outputFile =
      TString::Format("output_digi_rcnp_%s_%.1f_%.1f_hole_550Torr.root", filename, minangle, maxangle);
   TString scriptfile = "e12014_pad_map_size.xml";

   TString paramFile = "BraggCurveTesting.par";

   TString dir = getenv("VMCWORKDIR");

   TString mcFile = TString::Format("attpcsim_%s_%.1f_%.1f_550Torr.root", filename, minangle, maxangle);

   // Create the full parameter file paths
   TString digiParFile = dir + "/macro/tests/BraggCurveTesting/" + paramFile;
   TString mapParFile = dir + "/scripts/" + scriptfile;

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;

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
   mapping->ParseInhibitMap(dir + "/resources/coordmap_inhi.txt", AtMap::InhibitType::kTotal);

   AtClusterizeTask *clusterizer = new AtClusterizeTask(std::make_shared<AtClusterizeLine>());
   clusterizer->SetPersistence(kTRUE);
   // clusterizer->SetATTPCInverted(true);

   AtPulseTask *pulse = new AtPulseTask(std::make_shared<AtPulseLine>(mapping));
   pulse->SetPersistence(kTRUE);

   auto psa = std::make_unique<AtPSAMax>();
   // psa->SetThreshold(5);

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(kTRUE);
   // psaTask->SetATTPCInverted(true);

   AtRansacTask *ransacTask = new AtRansacTask();
   ransacTask->SetPersistence(kTRUE);
   ransacTask->SetVerbose(kFALSE);
   ransacTask->SetDistanceThreshold(15.0); // 12
   ransacTask->SetMinHitsLine(10);         // 10
   // in AtRansacTask parttern type set to line : auto patternType = AtPatterns::PatternType::kLine;
   ransacTask->SetAlgorithm(1);  // 1=Homemade Ransac (default); 2=Homemade Mlesac; 3=Homemade Lmeds;//4
   ransacTask->SetRanSamMode(5); // SampleMethod { kUniform = 0, kChargeWeighted = 1, kGaussian = 2, kWeightedGaussian =
                                 // 3, kWeightedY = 4 };//2
   ransacTask->SetChargeThreshold(0); // 150
   ransacTask->SetNumItera(500);

   // Create the AtPatternModification task.
   std::vector<std::unique_ptr<AtPatternModification>> patternModifications;

   auto braggCurveFinder = std::make_unique<AtBraggCurveFinder>();
   braggCurveFinder->SetBinSize(6.);
   braggCurveFinder->SetNumSmoothingSteps(200);
   patternModifications.push_back(std::move(braggCurveFinder));

   AtPatternModificationTask *patternModTask = new AtPatternModificationTask(std::move(patternModifications));
   // patternModTask->SetOutputBranch("AtPatternEvent");
   patternModTask->SetPersistence(kTRUE);

   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(psaTask);
   fRun->AddTask(ransacTask);
   fRun->AddTask(patternModTask);

   //  __ Init and run ___________________________________
   fRun->Init();

   timer.Start();
   // fRun->Run(0, 30000);
   fRun->Run(0, nEvent);
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

bool reduceFunc(AtRawEvent *evt)
{
   return (evt->GetNumPads() > 50) && evt->IsGood();
}
