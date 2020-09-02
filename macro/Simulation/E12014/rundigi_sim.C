//Code to take MC tracks and digitize


void rundigi_sim()
{

  TString scriptfile = "Lookup20150611.xml";
  TString paramFile  = "ATTPC.e12014.par";
  
  TString dir = getenv("VMCWORKDIR");
  
  TString mcFile      = "./data/PbFission_sim.root";
  TString digiParFile = dir + "/parameters/" + paramFile;
  TString mapParFile  = dir + "/scripts/" + scriptfile;

  
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

  //FairParAsciiFileIo* parIo2 = new FairParAsciiFileIo();
  //parIo2 -> open(trigParFile.Data(), "in");
  //rtdb -> setSecondInput(parIo2);
  
  // __ AT digi tasks___________________________________
  ATClusterizeTask* clusterizer = new ATClusterizeTask();
  clusterizer -> SetPersistence(kFALSE);
  
  ATPulseTask* pulse = new ATPulseTask();
  pulse -> SetPersistence(kTRUE);
  
  ATPSATask *psaTask = new ATPSATask();
  psaTask -> SetPersistence(kTRUE);
  psaTask -> SetThreshold(10);
  psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC
  
  fRun -> AddTask(clusterizer);
  fRun -> AddTask(pulse);
  fRun -> AddTask(psaTask);
  
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
