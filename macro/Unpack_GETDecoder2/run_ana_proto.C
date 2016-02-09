void run_ana_proto(TString dataFile = "output_proto_reco.root",TString parameterFile = "pATTPC.TRIUMF2015.par"){

  // -----   Timer   --------------------------------------------------------
TStopwatch timer;
timer.Start();
// ------------------------------------------------------------------------


 gSystem->Load("libXMLParser.so");


 TString dir = getenv("VMCWORKDIR");
 TString paraDir = dir + "/parameters/";
 TString paramterFileWithPath = paraDir + parameterFile;

 FairLogger *logger = FairLogger::GetLogger();
 logger -> SetLogFileName("ATTPC_AnaLog.log");
 logger -> SetLogToFile(kTRUE);
 logger -> SetLogToScreen(kTRUE);
 logger -> SetLogVerbosityLevel("MEDIUM");


 FairRunAna* run = new FairRunAna();
 run -> SetInputFile(dataFile.Data());
 run -> SetOutputFile("output_proto_ana.root");



 FairRuntimeDb* rtdb = run->GetRuntimeDb();
 FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
 parIo1 -> open(paramterFileWithPath.Data(), "in");
 rtdb -> setSecondInput(parIo1);

  ATAnalysisTask *AnaTask = new ATAnalysisTask();
  AnaTask->SetPhiReco();
  AnaTask->SetHoughDist(2.0);
  AnaTask->SetPersistence(kTRUE);

  run->AddTask(AnaTask);

  run->Init();

  run->Run(0,100000);
  //run -> RunOnTBData();

// -----   Finish   -------------------------------------------------------
 timer.Stop();
 Double_t rtime = timer.RealTime();
 Double_t ctime = timer.CpuTime();
 cout << endl << endl;
 cout << "Macro finished succesfully." << endl;
 cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
 cout << endl;
 // ------------------------------------------------------------------------


 }
