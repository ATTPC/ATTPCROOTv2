bool reduceFunc(AtRawEvent *evt);

void digi_Be13_p(int nEvent = 10000, int subnum = 0, int angle_num = 18)
{
   const char* filename = "13Be_p";
   double minangle, maxangle;
   //minangle = angle_num, maxangle = minangle + 1;
   minangle = 0., maxangle = 45.;
   TString inOutDir = "./Output/";
   TString outputFile = /*inOutDir +*/ TString::Format("output_digi_rcnp_%s_%.1f_%.1f_hole_pdt3He4He12C_4mm.root",filename,minangle,maxangle);
   //TString scriptfile = "Lookup20150611.xml";
   TString scriptfile = "e12014_pad_map_size.xml";

   TString paramFile = "rcnp_attpc_12Be.par";

   TString dir = getenv("VMCWORKDIR");

   TString mcFile = /*inOutDir +*/ TString::Format("attpcsim_%s_%.1f_%.1f.root",filename,minangle,maxangle);

   // Create the full parameter file paths
   TString digiParFile = dir + "/parameters/" + paramFile;
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
   mapping->ParseInhibitMap( dir + "/resources/coordmap_inhi.txt", AtMap::InhibitType::kTotal);

   AtClusterizeTask *clusterizer = new AtClusterizeTask(std::make_shared<AtClusterizeLine>());
   clusterizer->SetPersistence(kFALSE);

   AtPulseTask *pulse = new AtPulseTask(std::make_shared<AtPulseLine>(mapping));
   pulse->SetPersistence(kTRUE);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(0);

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(kTRUE);

   AtRansacTask *ransacTask = new AtRansacTask();
   ransacTask->SetPersistence(kFALSE);
   ransacTask->SetVerbose(kFALSE);
   ransacTask->SetDistanceThreshold(15.0); // 12
   ransacTask->SetMinHitsLine(30);         // 10
   // in AtRansacTask parttern type set to line : auto patternType = AtPatterns::PatternType::kLine;
   ransacTask->SetAlgorithm(1); // 1=Homemade Ransac (default); 2=Homemade Mlesac; 3=Homemade Lmeds;//4
   ransacTask->SetRanSamMode(5); // SampleMethod { kUniform = 0, kChargeWeighted = 1, kGaussian = 2, kWeightedGaussian = 3, kWeightedY = 4 };//2
   ransacTask->SetChargeThreshold(0); // 150
   ransacTask->SetNumItera(500);

   At3DBraggCurveTask *braggTask = new At3DBraggCurveTask();
   braggTask->SetPersistence(kTRUE);

   auto fitter = std::make_unique<AtFITTER::At3DBraggFitter>();

   // Proton
   fitter->AddAZPair({1, 1});
   fitter->AddMassUma(1.007825031898);

   // Deuteron
   fitter->AddAZPair({2, 1});
   fitter->AddMassUma(2.014101777844);

   // Triton
   fitter->AddAZPair({3, 1});
   fitter->AddMassUma(3.01604928132);

   // 3He
   fitter->AddAZPair({3, 2});
   fitter->AddMassUma(3.01602932197);

   // 4He
   fitter->AddAZPair({4, 2});
   fitter->AddMassUma(4.00260325413);

   // 12C
   fitter->AddAZPair({12, 6});
   fitter->AddMassUma(12.);

   fitter->SetMaterialDensity(1.28e-3); // g/cm3
   fitter->AddMaterialComponent({12, 6, 3});
   fitter->AddMaterialComponent({2, 1, 8});

   fitter->SetDZ(4.); // mm
   fitter->SetMinKE(1.); // MeV
   fitter->SetMaxKE(5.); // MeV
   fitter->SetKEGuess(3.); // MeV

   fitter->Init();

   braggTask->Set3DBraggCurveFitter(std::move(fitter));
   braggTask->SetMap(mapping);

   /*AtFitterTask *fitterTask = new AtFitterTask(std::move(fitter));
   fitterTask->SetPersistence(kFALSE);
   fitterTask->SetInputBranch("AtPatternEvent3DBragg");
   fitterTask->SetVerbose(1);*/


   //AtPRAtask *praTask = new AtPRAtask();
   //praTask->SetPersistence(kTRUE);

   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(psaTask);
   //fRun->AddTask(praTask);
   fRun->AddTask(ransacTask);
   fRun->AddTask(braggTask);
   //fRun->AddTask(fitterTask);

   //  __ Init and run ___________________________________
   fRun->Init();

   timer.Start();
   fRun->Run(0, 1000);
   //fRun->Run(0, nEvent);
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
