void rundigi_sim
(TString mcFile = "/Users/yassid/fair_install/ATTPCROOTv2/macro/Simulation/data/attpcsim_proto.root",
 TString digiParFile = "/Users/yassid/fair_install/ATTPCROOTv2/parameters/AT.digi.par",
 TString mapParFile = "/Users/yassid/fair_install/ATTPCROOTv2/scripts/Lookup20150611.txt",
TString trigParFile = "/Users/yassid/fair_install/ATTPCROOTv2/parameters/AT.trigger.par")
{


  // -----   Timer   --------------------------------------------------------
 TStopwatch timer;
 timer.Start();
 // ------------------------------------------------------------------------
  // __ Run ____________________________________________
  FairRunAna* fRun = new FairRunAna();
              fRun -> SetInputFile(mcFile);
              fRun -> SetOutputFile("output_digi.root");


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
      pulse -> SetPersistence(kFALSE);

      ATPSATask *psaTask = new ATPSATask();
      psaTask -> SetPersistence(kTRUE);
      psaTask -> SetThreshold(20);
      psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC
      //psaTask -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
      psaTask -> SetMaxFinder();
      psaTask -> SetBaseCorrection(kTRUE); //Directly apply the base line correction to the pulse amplitude to correct for the mesh induction. If false the correction is just saved
      psaTask -> SetTimeCorrection(kFALSE); //Interpolation around the maximum of the signal peak

      ATTriggerTask *trigTask = new ATTriggerTask();
      trigTask  ->  SetAtMap(mapParFile);
      trigTask  ->  SetPersistence(kTRUE);


  fRun -> AddTask(clusterizer);
  fRun -> AddTask(pulse);
  //fRun -> AddTask(psaTask);
  //`fRun -> AddTask(trigTask);

  // __ Init and run ___________________________________

  fRun -> Init();
  fRun -> Run(0,2);

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
