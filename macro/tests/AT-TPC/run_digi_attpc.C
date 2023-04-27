// Code to take MC tracks and digitize

bool reduceFunc(AtRawEvent *evt);

void run_digi_attpc()
{
   TString inOutDir = "./data/";
   TString outputFile = inOutDir + "output_digi.root";
   TString scriptfile = "e12014_pad_mapping.xml";
   TString paramFile = "ATTPC.e12014.par";

   TString dir = getenv("VMCWORKDIR");

   // TString mcFile = "./data/sim_attpc.root";
   TString mcFile = inOutDir + "symFissionMC.root";

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
   // mapping->ParseInhibitMap("./data/inhibit.txt", AtMap::InhibitType::kTotal);

   // __ AT digi tasks___________________________________
   // AtClusterizeTask *clusterizer = new AtClusterizeTask(std::make_shared<AtClusterize>());
   AtClusterizeTask *clusterizer = new AtClusterizeTask(std::make_shared<AtClusterizeLine>());
   clusterizer->SetPersistence(kFALSE);

   // AtPulseLineTask *pulse = new AtPulseLineTask();

   // AtPulseTask *pulse = new AtPulseTask(std::make_shared<AtPulse>(mapping));
   auto pulse = std::make_shared<AtPulseLine>(mapping);
   pulse->SetSaveCharge(true);
   AtPulseTask *pulseTask = new AtPulseTask(pulse);
   pulseTask->SetPersistence(kTRUE);

   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEvent");
   reduceTask->SetReductionFunction<AtRawEvent>(&reduceFunc);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(25);

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(kTRUE);

   AtRansacTask *ransacTask = new AtRansacTask();
   ransacTask->SetPersistence(kTRUE);
   ransacTask->SetVerbose(kFALSE);
   ransacTask->SetDistanceThreshold(20.0);
   ransacTask->SetMinHitsLine(10);

   fRun->AddTask(clusterizer);
   fRun->AddTask(pulseTask);
   fRun->AddTask(reduceTask);
   fRun->AddTask(psaTask);
   fRun->AddTask(ransacTask);

   //  __ Init and run ___________________________________
   fRun->Init();

   timer.Start();
   // fRun->Run(0, 20001);
   fRun->Run(0, 10);
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
