/**************************************************************
    - Pulse digitization from simulated data
***************************************************************/
void run_Trigger_sim(TString tag = "test")
{
  TStopwatch timer;
  timer.Start();

  TString workDir     = gSystem -> Getenv("VMCWORKDIR");
  TString geomDir     = workDir + "/geometry";
  TString confDir     = workDir + "/gconfig";
  TString mcFile      = workDir +"/macro/Simulation/digi/attpcsim.root";
  TString mcParFile   = workDir +"/macro/Simulation/digi/attpcpar.root";
  TString digiFile    = workDir +"/macro/Simulation/digi/attpcdigi.root";
  TString digiParFile = workDir + "/parameters/ATTPC.alpha.par";
  //TString trigParFile = workDir + "/parameters/AT.trigger.par";
  TString mapParFile = workDir + "/scripts/Lookup20150611.txt";

  gSystem->Setenv("GEOMPATH",geomDir.Data());
  gSystem->Setenv("CONFIG_DIR",confDir.Data());



  FairLogger *logger = FairLogger::GetLogger();
              logger -> SetLogFileName("log/digi.log");
              logger -> SetLogToScreen(kTRUE);
              logger -> SetLogToFile(kTRUE);
              logger -> SetLogVerbosityLevel("HIGH");

  FairRunAna* fRun = new FairRunAna();
              fRun -> SetInputFile(mcFile.Data());
              fRun -> SetOutputFile(digiFile.Data());

  FairParRootFileIo*  mcParInput = new FairParRootFileIo();
                      mcParInput -> open(mcParFile);
  FairParAsciiFileIo* digiParInput = new FairParAsciiFileIo();
                      digiParInput -> open(digiParFile);
  /*FairParAsciiFileIo* TrigParInput = new FairParAsciiFileIo();
                      TrigParInput -> open(trigParFile);*/

  FairRuntimeDb* fDb = fRun -> GetRuntimeDb();
                 fDb -> setFirstInput(mcParInput);
                 fDb -> setSecondInput(digiParInput);
                 //fDb -> setSecondInput(TrigParInput);


   // __ AT digi tasks___________________________________

  ATClusterizeTask* clusterizer = new ATClusterizeTask();
     clusterizer -> SetPersistence(kTRUE);

  ATPulseTask* pulse = new ATPulseTask();
      pulse -> SetPersistence(kTRUE);

  ATPSATask *psaTask = new ATPSATask();
      psaTask -> SetPersistence(kTRUE);
      psaTask -> SetThreshold(0.01);
      psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC
      psaTask -> SetMaxFinder();
      psaTask -> SetBaseCorrection(kTRUE); //Directly apply the base line correction to the pulse amplitude to correct for the mesh induction. If false the correction is just saved
      psaTask -> SetTimeCorrection(kFALSE); //Interpolation around the maximum of the signal peak
      //psaTask -> SetGainCalibration(digiParFile);


 /*ATTriggerTask *trigTask = new ATTriggerTask();
    trigTask  ->  SetAtMap(mapParFile);
    trigTask  ->  SetPersistence(kTRUE);*/

  fRun -> AddTask(clusterizer);
  fRun -> AddTask(pulse);
  fRun -> AddTask(psaTask);
  //fRun -> AddTask(trigTask);

 // __ Init and run ___________________________________
  fRun -> Init();
  fRun -> Run(0,2);

  timer.Stop();
  cout << endl << endl;
  cout << "Digitization macro finished succesfully." << endl;
  cout << "Output file : " << digiFile       << endl;
  cout << "Real time " << timer.RealTime()   << " s" << endl;
  cout << "CPU  time " << timer.CpuTime()    << " s" << endl << endl;
}
