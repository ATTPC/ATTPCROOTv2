// Code to take MC tracks and digitize

bool reduceFunc(AtRawEvent *evt);
AtPad GetNominalResponse(double par = 3)
{
   AtPad::trace wave;

   TF1 responseFunc("fR", "exp(-[0]*x/[1])*sin(x/[1])*(x/[1])**3", 0, 512);

   responseFunc.SetParameter(0, par);
   responseFunc.SetParameter(1, 0.72 / 0.32);
   double scale = responseFunc.GetMaximum();

   for (int i = 0; i < wave.size(); ++i)
      wave[i] = responseFunc(i) / scale;

   AtPad response;
   response.SetADC(wave);

   return response;
}

void rundigi_fast()
{
   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   // fair::Logger::SetConsoleSeverity("debug");

   // TString outputFile = "data/output_digiFast.root";
   TString inOutDir = "./eventGenerator/sym90/";
   TString outputFile = inOutDir + "output_digi.root";
   TString scriptfile = "e12014_pad_mapping.xml";
   TString paramFile = "ATTPC.e12014.par";

   TString dir = getenv("VMCWORKDIR");

   // TString mcFile = "./data/sim_attpc.root";
   TString mcFile = inOutDir + "sim_attpc.root";

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
   mapping->ParseInhibitMap("inhibit.txt", AtMap::InhibitType::kTotal);

   // __ AT digi tasks___________________________________
   // AtClusterizeFastTask *clusterizer = new AtClusterizeFastTask();
   // AtClusterizeTask *clusterizer = new AtClusterizeTask();
   AtClusterizeLineTask *clusterizer = new AtClusterizeLineTask();
   clusterizer->SetPersistence(kFALSE);

   // AtPulseTask *pulse = new AtPulseTask();
   AtPulseLineTask *pulse = new AtPulseLineTask();
   pulse->SetPersistence(kTRUE);
   pulse->SetMap(mapping);
   pulse->SetSaveMCInfo();
   pulse->UseChargeSave(true);
   // pulse->SetNoiseSigma(3);
   //  pulse->SetNumIntegrationPoints(500);

   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEvent");
   reduceTask->SetReductionFunction(&reduceFunc);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(25);
   auto psaDeconv = std::make_unique<AtPSADeconv>();
   psaDeconv->SetResponse(GetNominalResponse());
   psaDeconv->SetFilterOrder(6);
   psaDeconv->SetCutoffFreq(60);
   psaDeconv->SetThreshold(25);

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(std::move(psaDeconv));
   psaTask->SetPersistence(kTRUE);

   auto sac = std::make_unique<SampleConsensus::AtSampleConsensus>(
      SampleConsensus::Estimators::kRANSAC, AtPatterns::PatternType::kLine, RandomSample::SampleMethod::kUniform);
   auto sacTask = new AtSampleConsensusTask(std::move(sac));
   sacTask->SetPersistence(true);

   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(reduceTask);
   fRun->AddTask(psaTask);
   fRun->AddTask(sacTask);

   //  __ Init and run ___________________________________
   fRun->Init();

   timer.Start();
   fRun->Run(0, 20);
   // fRun->Run(0, 50);
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
