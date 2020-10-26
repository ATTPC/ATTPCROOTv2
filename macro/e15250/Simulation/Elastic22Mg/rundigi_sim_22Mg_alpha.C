void rundigi_sim_22Mg_alpha
(TString mcFile = "./data/attpcsim_e15250.root",
 TString digiParFile = "../../../../parameters/ATTPC.e15250.par",
 TString mapParFile = "../../../../scripts/Lookup20150611.xml",
 TString trigParFile = "../../../../parameters/AT.trigger.par")
{
  // -----   Timer   --------------------------------------------------------
 TStopwatch timer;
 timer.Start();


  TString scriptfile = "Lookup20150611.xml";
  TString dir = getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString dataDir = dir + "/macro/data/";
  TString geomDir = dir + "/geometry/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());

 // ------------------------------------------------------------------------
  // __ Run ____________________________________________
  FairRunAna* fRun = new FairRunAna();
              fRun -> SetInputFile(mcFile);
              fRun -> SetOutputFile("output_digi_22Mg.root");


  FairRuntimeDb* rtdb = fRun->GetRuntimeDb();
              FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
              parIo1 -> open(digiParFile.Data(), "in");
              rtdb -> setFirstInput(parIo1);
              FairParAsciiFileIo* parIo2 = new FairParAsciiFileIo();
              parIo2 -> open(trigParFile.Data(), "in");
              rtdb -> setSecondInput(parIo2);

  // __ AT digi tasks___________________________________

      ATClusterizeTask* clusterizer = new ATClusterizeTask();
      clusterizer -> SetPersistence(kFALSE);

      ATPulseTask* pulse = new ATPulseTask();
      pulse -> SetPersistence(kTRUE);

      ATPSATask *psaTask = new ATPSATask();
      psaTask -> SetPersistence(kTRUE);
      psaTask -> SetThreshold(10);
      //psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC
      psaTask -> SetPSAMode(1); //FULL mode
      //psaTask -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
      psaTask -> SetMaxFinder();
      psaTask -> SetBaseCorrection(kFALSE); //Directly apply the base line correction to the pulse amplitude to correct for the mesh induction. If false the correction is just saved
      psaTask -> SetTimeCorrection(kFALSE); //Interpolation around the maximum of the signal peak

      ATTriggerTask *trigTask = new ATTriggerTask();
      trigTask  ->  SetAtMap(mapParFile);
      trigTask  ->  SetPersistence(kTRUE);
     
      ATPRATask *praTask = new ATPRATask();
      praTask -> SetPersistence(kTRUE);

	ATRansacTask *RandTask = new ATRansacTask();
  	RandTask ->SetPersistence(kTRUE);
  	//RandTask ->SetModelType(1);
  	//RandTask ->SetFullMode();
 	 RandTask->SetTiltAngle(0.0);
  	RandTask->SetDistanceThreshold(15.0);
 	 RandTask->SetMinHitsLine(7);
  	RandTask->SetAlgorithm(3); // 0=PCL ransac; 1=Homemade Ransac; 2=Homemade Mlesac; 3=Homemade Lmeds;
 	 RandTask->SetRanSamMode(3);// 0=Uniform; 1=Gaussian; 2=Weighted; 3=Gaussian+Weighted
	
/*
      ATHoughTask *HoughTask = new ATHoughTask();
      HoughTask ->SetPersistence();
      HoughTask ->SetCircularHough();
      HoughTask ->SetHoughThreshold(10.0); // Charge threshold for Hough
      HoughTask ->SetEnableMap(); //Enables an instance of the ATTPC map:  This enables the MC with Q instead of position
      HoughTask ->SetMap(scriptdir.Data());
*/
  fRun -> AddTask(clusterizer);
  fRun -> AddTask(pulse);
  fRun -> AddTask(psaTask);
  fRun -> AddTask(RandTask);
  //fRun -> AddTask(praTask);
  //fRun -> AddTask(trigTask);
  //fRun -> AddTask(HoughTask);
  //fRun -> AddTask(hierarchicalClusteringTask);

  // __ Init and run ___________________________________

  fRun -> Init();
  fRun -> Run(0,10);

  std::cout << std::endl << std::endl;
  std::cout << "Macro finished succesfully."  << std::endl << std::endl;
  // -----   Finish   -------------------------------------------------------
  timer.Stop();
  Double_t rtime = timer.RealTime();
  Double_t ctime = timer.CpuTime();
  cout << endl;
  cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
  cout << endl;
  // ------------------------------------------------------------------------
}
