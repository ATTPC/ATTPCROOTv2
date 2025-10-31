bool reduceFunc(AtRawEvent *evt);

void digi_Be13_p(int nEvent = 10000)
{
   Double_t ThetaMinCMS = 0.0;
   Double_t ThetaMaxCMS = 50.0;

   TString mcFile = TString::Format("./simData/attpcsim_13Be_p_%.1f_%.1f_600Torr.root", ThetaMinCMS, ThetaMaxCMS);
   TString outputFile = TString::Format("./digiFiles/output_digi_rcnp_13Be_p_%.1f_%.1f_600Torr_9mmBinning.root", ThetaMinCMS, ThetaMaxCMS);

   TString scriptfile = "RCNP2025.xml";
   TString paramFile = "ATTPC.E656_RCNP.SIM.par";

   TString dir = getenv("VMCWORKDIR");

   // Create the full parameter file paths
   TString digiParFile = "./" + paramFile;
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

   AtPulseTask *pulse = new AtPulseTask(std::make_shared<AtPulseLine>(mapping));
   pulse->SetPersistence(kTRUE);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(5);

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(kTRUE);

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
   braggCurveFinder->SetBinSize(9.);
   braggCurveFinder->SetNumSmoothingSteps(200);
   patternModifications.push_back(std::move(braggCurveFinder));

   AtPatternModificationTask *patternModTask = new AtPatternModificationTask(std::move(patternModifications));
   // patternModTask->SetOutputBranch("AtPatternEvent");
   patternModTask->SetPersistence(kTRUE);

   // Create the AtFitterTask task.
   std::vector<std::unique_ptr<AtTools::AtELossModel>> eLossModels;

   double density = 1.7078e-3; // 600Torr
   std::vector<std::tuple<int, int, int>> materialComponents;
   materialComponents.push_back(std::make_tuple(12, 6, 3));
   materialComponents.push_back(std::make_tuple(2, 1, 8));

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_p = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_600Torr_p");
   eLossModelC3D8_p->SetMaterial(materialComponents);
   eLossModelC3D8_p->SetProjectile(1, 1, 1.007825031898);
   eLossModelC3D8_p->SetPDGCode("1000010010");
   eLossModels.push_back(std::move(eLossModelC3D8_p));

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_d = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_600Torr_d");
   eLossModelC3D8_d->SetMaterial(materialComponents);
   eLossModelC3D8_d->SetProjectile(2, 1, 2.014101777844);
   eLossModelC3D8_d->SetPDGCode("1000010020");
   //eLossModels.push_back(std::move(eLossModelC3D8_d));

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_t = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_600Torr_t");
   eLossModelC3D8_t->SetMaterial(materialComponents);
   eLossModelC3D8_t->SetProjectile(3, 1, 3.01604928132);
   eLossModelC3D8_t->SetPDGCode("1000010030");
   //eLossModels.push_back(std::move(eLossModelC3D8_t));

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_4He = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_600Torr_4He");
   eLossModelC3D8_4He->SetMaterial(materialComponents);
   eLossModelC3D8_4He->SetProjectile(4, 2, 4.00260325413);
   eLossModelC3D8_4He->SetPDGCode("1000020040");
   //eLossModels.push_back(std::move(eLossModelC3D8_4He));

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_12Be = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_600Torr_12Be");
   eLossModelC3D8_12Be->SetMaterial(materialComponents);
   eLossModelC3D8_12Be->SetProjectile(12, 4, 12.026922082);
   eLossModelC3D8_12Be->SetPDGCode("1000040120");
   eLossModels.push_back(std::move(eLossModelC3D8_12Be));

   std::unique_ptr<EventFit::AtBraggCurveFitter> braggCurveFitter = std::make_unique<EventFit::AtBraggCurveFitter>(std::move(eLossModels));
   braggCurveFitter->SetEstimatedAmplitudeFactor(360);
   braggCurveFitter->SetEstimatedAmplitudeFactorPrecision(30);
   braggCurveFitter->SetDistanceThreshold(10);
   braggCurveFitter->Init();

   AtFitterTask *fitterTask = new AtFitterTask(std::move(braggCurveFitter));
   fitterTask->SetPersistence(kTRUE);
   fitterTask->SetInputBranch("AtPatternEventModified");
   fitterTask->SetFitMetadataBranch("AtFitMetadata");

   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(psaTask);
   fRun->AddTask(ransacTask);
   fRun->AddTask(patternModTask);
   fRun->AddTask(fitterTask);

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
