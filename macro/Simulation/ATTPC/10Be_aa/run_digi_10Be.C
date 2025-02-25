bool reduceFunc(AtRawEvent *evt);

void run_digi_10Be()
{
   TString inOutDir = "./data/";
   //   TString outputFile = inOutDir + "output_digi.root";
   //   TString outputFile = inOutDir + "output_digi_rcnp_12c.root";
   TString outputFile = "/home/david/PhD/PhD-14-02/attpcroot/ATTPCROOTv2/macro/Simulation/ATTPC/10Be_aa/data/output_digi.root";
   TString scriptfile = "ANL2023.xml";

      TString paramFile = "ATTPC.e22502.par";
   //TString paramFile = "rcnp_attpc.par";

   TString dir = getenv("VMCWORKDIR");

   // TString mcFile = "./data/sim_attpc.root";
   //   TString mcFile = inOutDir + "attpcsim.root";
      TString mcFile = inOutDir + "attpcsim.root";
   //TString mcFile = "./data1/attpcsim_12c12c_" + std::to_string(subnum) + ".root";

   // Create the full parameter file paths
   TString digiParFile = dir + "/parameters/" + paramFile;
   TString mapParFile = dir + "/scripts/" + scriptfile;

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;

   // ------------------------------------------------------------------------
TString zlutFile = dir + "/resources/corrections/a1954/zLUT.txt";
   TString radlutFile = dir + "/resources/corrections/a1954/radLUT.txt";
   TString tralutFile = dir + "/resources/corrections/a1954/traLUT.txt";
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
   mapping->ParseInhibitMap("./data/inhibit.txt", AtMap::InhibitType::kTotal);

   AtClusterizeTask *clusterizer = new AtClusterizeTask();
   clusterizer->SetPersistence(kFALSE);

   AtPulseTask *pulse = new AtPulseTask(std::make_shared<AtPulse>(mapping));
   pulse->SetPersistence(kTRUE);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(60);

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(kFALSE);

   auto SCModel = std::make_unique<AtEDistortionModel>();
   SCModel->SetCorrectionMaps(zlutFile.Data(), radlutFile.Data(), tralutFile.Data());
   auto SCTask = new AtSpaceChargeCorrectionTask(std::move(SCModel));
   SCTask->SetInputBranch("AtEventH");
   AtPRAtask *praTask = new AtPRAtask();
   praTask->SetInputBranch("AtEventCorrected");
   praTask->SetOutputBranch("AtPatternEvent");
   praTask->SetPersistence(kTRUE);
   //praTask->SetClusterRadius(clusterRadius);
   //praTask->SetClusterDistance(clusterDistance);
   // praTask->SetMaxNumHits(3000);
   // praTask->SetMinNumHits(100);
    praTask->SetTcluster(8.5);
    praTask->SetMcluster(25);

   // Fitting task
   Float_t gasMediumDensity = 0.0657;
   Float_t magneticField = 2.00;
   Int_t pdg = 1000020040; // 1000010020; 2212;
   Bool_t noMatEffects = 1;
   
   AtFITTER::AtGenfit::Exp exp = AtFITTER::AtGenfit::a1975;
   
   std::string elossFile = (std::string)dir.Data() + "/resources/energy_loss/alpha_He_300torr.txt";
   
   auto fitter = std::make_unique<AtFITTER::AtGenfit>(magneticField, 0.00001, 1000.0, elossFile, gasMediumDensity, pdg,
                                                      5, 20, noMatEffects);
   std::cout << "***** Setting up fitter ******" << std::endl;
   fitter->SetIonName("alpha"); // deuteron proton
   fitter->SetMass(4.002603);     // 2.0135532 1.00727646
   fitter->SetAtomicNumber(2);
   fitter->SetNumFitPoints(1.0);
   fitter->SetVerbosityLevel(1);
   fitter->SetSimulationConvention(1);
   // fitter->SetExpNum(exp);
   fitter->SetFitDirection(0);
   fitter->EnableMerging(1);
   fitter->EnableSingleVertexTrack(1);
   fitter->EnableReclustering(1, 25.0, 8.5);

   AtFitterTask *fitterTask = new AtFitterTask(std::move(fitter));
   fitterTask->SetPersistence(true);

   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(psaTask);
   fRun->AddTask(SCTask);
   fRun->AddTask(praTask);
   fRun->AddTask(fitterTask);

   //  __ Init and run ___________________________________
   fRun->Init();

   timer.Start();
   fRun->Run(0, 2000);
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
   return (evt->GetNumPads() > 0) && evt->IsGood();
}
