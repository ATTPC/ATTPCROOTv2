void runreco_sim(
   TString mcFile = "output_digi_10Be_aaHe6.root",
   TString digiParFile =
      "/mnt/simulations/attpcroot/fair_install_2020/yassid/ATTPCROOTv2/parameters/ATTPC.e15250_sim.par",
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
   fRun->SetOutputFile("output_digi_10Be_aaHe6_PRA.root");

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   FairParAsciiFileIo *parIo2 = new FairParAsciiFileIo();
   parIo2->open(trigParFile.Data(), "in");
   rtdb->setSecondInput(parIo2);

   // __ AT digi tasks___________________________________

   ATPRATask *praTask = new ATPRATask();
   praTask->SetPersistence(kTRUE);

   /*ATTriggerTask *trigTask = new ATTriggerTask();
   trigTask  ->  SetAtMap(mapParFile);
   trigTask  ->  SetPersistence(kTRUE);*/

   fRun->AddTask(praTask);
   // fRun -> AddTask(trigTask);

   // __ Init and run ___________________________________

   fRun->Init();
   fRun->Run(0, 20);

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
