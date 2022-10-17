void rundigi_d2He(
   TString mcFile = "outputFiles/attpcsim_d2He.root", // attpcsim_d2He_14O_07atm_100000.root
   //(TString mcFile = "outputFiles/attpcsim_d2He_1e6pps.root",//attpcsim_d2He_14O_07atm_100000.root
   TString digiParFile = "../../../parameters/ATTPC.d2HeSim.par",
   TString mapParFile = "../../../scripts/Lookup20150611.xml",
   TString parFile =
      "outputFiles/attpcpar_d2He.root") //"attpcpar_d2He_test_10k.root")//attpcpar_d2He_14O_07atm_100000.root
// TString parFile = "outputFiles/attpcpar_d2He_1e6pps.root")
// //"attpcpar_d2He_test_10k.root")//attpcpar_d2He_14O_07atm_100000.root TString trigParFile =
// "../../../parameters/AT.trigger.par")
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
   TString inimap = dir + "/resources/coordmap_inhi.txt";

   TFile *fileSim = new TFile(mcFile, "READ");
   TTree *treeSim = (TTree *)fileSim->Get("cbmsim");
   Int_t nEvents = treeSim->GetEntries();
   TString str_nEvents = std::to_string(nEvents);
   delete treeSim;
   delete fileSim;

   // ------------------------------------------------------------------------
   // __ Run ____________________________________________
   FairRunAna *fRun = new FairRunAna();
   fRun->SetInputFile(mcFile);
   // fRun -> SetOutputFile("outputFiles/attpcdigi_d2He_"+str_nEvents+".root");
   fRun->SetOutputFile("outputFiles/attpcdigi_d2He_test.root");
   // fRun -> SetOutputFile("outputFiles/attpcdigi_d2He_1e6pps_lmed.root");

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parIo1 = new FairParRootFileIo();
   parIo1->open(parFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   FairParAsciiFileIo *parIo2 = new FairParAsciiFileIo();
   parIo2->open(digiParFile.Data(), "in");
   rtdb->setSecondInput(parIo2);

   // __ AT digi tasks___________________________________

   ATClusterizeTask *clusterizer = new ATClusterizeTask();
   clusterizer->SetPersistence(kFALSE);

   ATPulseTask *pulse = new ATPulseTask();
   pulse->SetPersistence(kTRUE);
   pulse->IsInhibitMap(kTRUE);
   pulse->SetInhibitMaps(inimap, "0", "0");

   //      ATTriggerTask *trigTask = new ATTriggerTask();
   //      trigTask  ->  SetAtMap(mapParFile);
   //      trigTask  ->  SetPersistence(kTRUE);

   ATPSATask *psaTask = new ATPSATask();
   psaTask->SetPersistence(kTRUE);
   // psaTask -> SetThreshold(1);//10
   psaTask->SetThreshold(0.0);

   // psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC
   psaTask->SetPSAMode(1); // 1 : SImple2, 4 FULL mode
   // psaTask -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
   psaTask->SetMaxFinder();
   psaTask->SetBaseCorrection(kFALSE); // Directly apply the base line correction to the pulse amplitude to correct for
                                       // the mesh induction. If false the correction is just saved
   psaTask->SetTimeCorrection(kFALSE); // Interpolation around the maximum of the signal peak

   ATRansacTask *RandTask = new ATRansacTask();
   RandTask->SetPersistence(kTRUE);
   // RandTask ->SetModelType(1);
   // RandTask ->SetFullMode();
   RandTask->SetTiltAngle(0.0);
   RandTask->SetDistanceThreshold(10.0);
   RandTask->SetMinHitsLine(10);
   RandTask->SetAlgorithm(3);  // 0=PCL ransac; 1=Homemade Ransac; 2=Homemade Mlesac; 3=Homemade Lmeds;
   RandTask->SetRanSamMode(1); // 0=Uniform; 1=Gaussian; 2=Weighted; 3=Gaussian+Weighted
   RandTask->SetVertexMode(1); // set to 1 for two tracks same vertex

   /*
     ATHoughTask *HoughTask = new ATHoughTask();
     HoughTask ->SetPersistence(kTRUE);
     HoughTask ->SetLinearHough();
     HoughTask ->SetHoughThreshold(10.0); // Charge threshold for Hough
     //HoughTask ->SetEnableMap(); //Enables an instance of the ATTPC map:  This enables the MC with Q instead of
     position
     //HoughTask ->SetMap(scriptdir.Data());
   */

   //  ATHoughTask *HoughTask = new ATHoughTask();
   //  HoughTask ->SetPersistence(kTRUE);

   // fRun -> AddTask(fDecoderTask);
   fRun->AddTask(clusterizer);
   fRun->AddTask(pulse);
   fRun->AddTask(psaTask);
   //    fRun -> AddTask(trigTask);
   fRun->AddTask(RandTask);

   // __ Init and run ___________________________________

   fRun->Init();
   fRun->Run(0, nEvents);
   // fRun -> Run(0,100);

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
