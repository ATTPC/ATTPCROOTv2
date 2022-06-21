//Code to take MC tracks and digitize

void rundigi_sim()
{

   TString scriptfile = "e12014_pad_mapping.xml";
   TString paramFile = "ATTPC.e12014.par";

   TString dir = getenv("VMCWORKDIR");
   TString outputDirectory = "/macro/e12014/simulation/eventGenerator/sym90/";

   TString mcFile = dir + outputDirectory + "sim_attpc.root";
   TString outputFile = dir + outputDirectory + "output_digi.root";

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
   mapping->GenerateAtTpc();

   // __ AT digi tasks___________________________________
   // AtClusterizeFastTask *clusterizer = new AtClusterizeFastTask();
   // AtClusterizeTask *clusterizer = new AtClusterizeTask();
   AtClusterizeLineTask *clusterizer = new AtClusterizeLineTask();
   clusterizer->SetPersistence(kFALSE);

   // AtPulseTask *pulse = new AtPulseTask();
   AtPulseLineTask *pulse = new AtPulseLineTask();
   pulse->SetPersistence(kTRUE);
   pulse->SetMap(mapping);
   pulse->SetLowGainFactor(0.08);

   AtPSASimple2 *psa = new AtPSASimple2();
   // psa->SetThreshold(35);
   psa->SetMaxFinder();

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(kTRUE);

   AtRansacTask *ransacTask = new AtRansacTask();
   ransacTask->SetPersistence(kTRUE);
   ransacTask->SetVerbose(kFALSE);
   ransacTask->SetDistanceThreshold(20.0);
   ransacTask->SetTiltAngle(0);
   ransacTask->SetMinHitsLine(10);
   ransacTask->SetFullMode();

   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(psaTask);
   // fRun->AddTask(ransacTask);
   //  __ Init and run ___________________________________
   fRun->Init();

   timer.Start();
   fRun->Run(0, 1000);
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
