void rundigi_sim(
   TString mcFile = "./data/attpcsim.root",
   TString mapParFile =
      "/mnt/simulations/attpcroot/fair_install_2020/yassid/ATTPCROOTv2/scripts/scripts/Lookup20150611.xml",
   TString trigParFile = "/mnt/simulations/attpcroot/fair_install_2020/yassid/ATTPCROOTv2/parameters/AT.trigger.par")
{
   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();

   TString scriptfile = "Lookup20150611.xml";
   TString dir = getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString dataDir = dir + "/macro/data/";
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());

   // ------------------------------------------------------------------------
   // __ Run ____________________________________________
   FairRunAna *fRun = new FairRunAna();
   fRun->SetInputFile(mcFile);
   fRun -> SetGeomFile("/mnt/simulations/attpcroot/fair_install_2020/ATTPCROOTv2_develop/geometry/ATTPC_He1bar_v2_geomanager.root");
   fRun->SetOutputFile("output_digi.root");

   TString parameterFile = "ATTPC.e20020_sim.par";
   TString digiParFile = dir + "/parameters/" + parameterFile;

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);

   // __ AT digi tasks___________________________________

   AtClusterizeTask *clusterizer = new AtClusterizeTask();
   clusterizer->SetPersistence(kFALSE);

   AtPulseTask *pulse = new AtPulseTask();
   pulse->SetPersistence(kTRUE);
   pulse->SetSaveMCInfo();

   AtPSASimple2 *psa = new AtPSASimple2();
   // psa -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
   // psa -> SetBaseCorrection(kFALSE);
   // psa -> SetTimeCorrection(kFALSE);

   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(kTRUE);
   psa->SetThreshold(10);
   psa->SetMaxFinder();

   AtPRAtask *praTask = new AtPRAtask();
   praTask->SetPersistence(kTRUE);

   AtFitterTask *fitterTask = new AtFitterTask();
   fitterTask->SetPersistence(kTRUE);

   /*ATTriggerTask *trigTask = new ATTriggerTask();
   trigTask  ->  SetAtMap(mapParFile);
   trigTask  ->  SetPersistence(kTRUE);*/

   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(psaTask);
   fRun->AddTask(praTask);
   fRun->AddTask(fitterTask);
   //fRun -> AddTask(trigTask);

   // __ Init and run ___________________________________

   fRun->Init();
   fRun->Run(0, 50);

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
