void rundigi_sim
(TString mcFile = "./data/attpcsim.root",
 TString digiParFile = "../../../parameters/ATTPC.30Mgtp.par",
 TString mapParFile = "../../../scripts/Lookup20150611.xml",
 TString trigParFile = "../../../parameters/AT.trigger.par")
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
              fRun -> SetOutputFile("output_digi_4.root");


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

      /*ATTriggerTask *trigTask = new ATTriggerTask();
      trigTask  ->  SetAtMap(mapParFile);
      trigTask  ->  SetPersistence(kTRUE);*/

      ATHoughTask *HoughTask = new ATHoughTask();
      HoughTask ->SetPersistence();
      HoughTask ->SetCircularHough();
      HoughTask ->SetHoughThreshold(10.0); // Charge threshold for Hough
      HoughTask ->SetEnableMap(); //Enables an instance of the ATTPC map:  This enables the MC with Q instead of position
      HoughTask ->SetMap(scriptdir.Data());
      


  fRun -> AddTask(clusterizer);
  fRun -> AddTask(pulse);
  fRun -> AddTask(psaTask);
  //fRun -> AddTask(trigTask);
  fRun -> AddTask(HoughTask);

  // __ Init and run ___________________________________

  fRun -> Init();
  fRun -> Run(0,2000);

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
