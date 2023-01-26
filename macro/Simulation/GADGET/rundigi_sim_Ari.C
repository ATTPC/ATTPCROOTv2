<<<<<<< HEAD
void rundigi_sim_Ari(
=======
void rundigi_sim(
>>>>>>>  added AtPulseTaskGADGET version of digi_sim
   TString mcFile = "./data/gadgetsim.root",
   TString mapParFile =
      "/mnt/simulations/attpcroot/fair_install_2020/yassid/ATTPCROOTv2/scripts/scripts/Lookup20150611.xml",
   TString trigParFile = "/mnt/simulations/attpcroot/fair_install_2020/yassid/ATTPCROOTv2/parameters/AT.trigger.par")
{
   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();

   TString scriptfile = "LookupGADGET08232021.xml";
   TString dir = getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString dataDir = dir + "/macro/data/";
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());

   // ------------------------------------------------------------------------
   // __ Run ____________________________________________
   FairRunAna *fRun = new FairRunAna();
   FairFileSource *source = new FairFileSource(mcFile);
   fRun->SetSource(source);
   fRun->SetOutputFile("./data/CD_digi.root");

   TString parameterFile = "GADGET.sim.par";
   TString digiParFile = dir + "/parameters/" + parameterFile;

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);

   auto mapping = std::make_shared<AtGadgetIIMap>();
   mapping->ParseXMLMap(mapParFile.Data());
   mapping->GeneratePadPlane();

   // __ AT digi tasks___________________________________

   AtClusterizeTask *clusterizer = new AtClusterizeTask();
   clusterizer->SetPersistence(kFALSE);

   AtPulseTaskGADGET *pulse = new AtPulseTaskGADGET();
   pulse->SetPersistence(kTRUE);
   pulse->SetSaveMCInfo();
   pulse->SetMap(mapping);
   pulse->UseChargeSave(kTRUE);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(5);

   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(kTRUE);

   AtPRAtask *praTask = new AtPRAtask();
   praTask->SetPersistence(kTRUE);

   auto *wHDF = new AtHDF5WriteTask("data/CD_digi.h5", "AtEventH");
   wHDF->SetUseEventNum(true);

   /*ATTriggerTask *trigTask = new ATTriggerTask();
     trigTask  ->  SetAtMap(mapParFile);
     trigTask  ->  SetPersistence(kTRUE);*/

   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(psaTask);
   fRun->AddTask(wHDF);
   // fRun -> AddTask(praTask);
   // fRun -> AddTask(trigTask);

   // __ Init and run ___________________________________

   fRun->Init();
   fRun->Run(0, 10);

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
