void rundigi_sim
(TString mcFile = "../data/attpcsim_e15250.root",
 TString digiParFile = "../../../parameters/ATTPC.e20020_sim.par",
 TString mapParFile = "../../../scripts/Lookup20150611.txt",
 TString trigParFile = "../../../parameters/AT.trigger.par")
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
  AtClusterizeTask* clusterizer = new AtClusterizeTask();
  clusterizer -> SetPersistence(kFALSE);
  
  AtPulseTask* pulse = new AtPulseTask();
  pulse -> SetPersistence(kTRUE);

   AtPSASimple2 *psa = new AtPSASimple2();
   // psa -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
   // psa -> SetBaseCorrection(kFALSE);
   // psa -> SetTimeCorrection(kFALSE);

   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(kTRUE);
   psa->SetThreshold(3);
   psa->SetMaxFinder();

   AtPRAtask *praTask = new AtPRAtask();
   praTask->SetPersistence(kTRUE);
  
  fRun -> AddTask(clusterizer);
  fRun -> AddTask(pulse);
  fRun -> AddTask(psaTask);
  //fRun -> AddTask(trigTask);
  fRun -> AddTask(praTask);
  
  
  // __ Init and run ___________________________________
  fRun -> Init();
  fRun -> Run(0,10000);
  
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
