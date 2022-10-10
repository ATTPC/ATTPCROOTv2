// Code to take MC tracks and digitize

bool reduceFunc(AtRawEvent *evt);

void rundigi_sim(TString mcFile = "./data/attpcsim.root")
{
   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();

   TString scriptfile = "LookupSpecMATnoScintHisto.xml";
   TString dir = getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString dataDir = dir + "/macro/data/";
   TString geomDir = dir + "/geometry/";
   TString OutFile = "./output_digi.root";
   gSystem->Setenv("GEOMPATH", geomDir.Data());

   // ------------------------------------------------------------------------
   // __ Run ____________________________________________
   FairRunAna *fRun = new FairRunAna();
   fRun->SetSource(new FairFileSource(mcFile));
   fRun->SetSink(new FairRootFileSink(OutFile));

   TString parameterFile = "SpecMAT.run_9901.par";
   TString digiParFile = dir + "/parameters/" + parameterFile;

   TString triggerFile = "SpecMAT.trigger.par";
   TString trigParFile = dir + "/parameters/" + triggerFile;

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);

   // Create the map that will be pased to tasks that require it
   auto fMapPtr = std::make_shared<AtSpecMATMap>(3174);
   fMapPtr->ParseXMLMap(scriptdir.Data());
   fMapPtr->GeneratePadPlane();
   // mapping->ParseInhibitMap("./data/inhibit.txt", AtMap::kTotal);

   // __ AT digi tasks___________________________________

   AtClusterizeTask *clusterizer = new AtClusterizeLineTask();
   clusterizer->SetPersistence(kFALSE);

   AtPulseTask *pulse = new AtPulseLineTask();
   pulse->SetPersistence(kTRUE);
   pulse->SetSaveMCInfo();
   pulse->SetMap(fMapPtr);
   // pulse->SelectDetectorId(kSpecMAT);

   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEvent");
   reduceTask->SetReductionFunction(&reduceFunc);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(1);

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(kTRUE);

   AtRansacTask *ransacTask = new AtRansacTask();
   ransacTask->SetPersistence(kTRUE);
   ransacTask->SetVerbose(kFALSE);
   ransacTask->SetDistanceThreshold(20.0);
   ransacTask->SetMinHitsLine(10);

   /*ATTriggerTask *trigTask = new ATTriggerTask();
   trigTask  ->  SetAtMap(scriptdir);
   trigTask  ->  SetPersistence(kTRUE);*/

   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(reduceTask);
   fRun->AddTask(psaTask);
   fRun->AddTask(ransacTask);
   // fRun -> AddTask(praTask);
   // fRun -> AddTask(trigTask);

   // __ Init and run ___________________________________

   fRun->Init();
   fRun->Run(0, 100);

   std::cout << std::endl << std::endl;
   std::cout << "Macro finished succesfully." << std::endl << std::endl;
   // -----   Finish   -------------------------------------------------------
   timer.Stop();
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
