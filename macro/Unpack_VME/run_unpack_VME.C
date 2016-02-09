void run_unpack_VME(){

   // -----   Timer   --------------------------------------------------------
	TStopwatch timer;
	timer.Start();
  // ------------------------------------------------------------------------

   gSystem->Load("libXMLParser.so");
   TString dir = getenv("VMCWORKDIR");

  FairLogger *logger = FairLogger::GetLogger();
  logger -> SetLogFileName("ATTPCLog.log");
  logger -> SetLogToFile(kTRUE);
  logger -> SetLogToScreen(kTRUE);
  logger -> SetLogVerbosityLevel("MEDIUM");

  FairRunAna* run = new FairRunAna();
  run -> SetOutputFile("output_VME.root");

    TString file = "../../parameters/AT.parameters.par";





  ATVMEUnpackTask *decoderTask = new ATVMEUnpackTask();

  decoderTask -> AddData("/data/e15503-data/e15503B/run66/run66-VM0238.cra");
  decoderTask -> SetICChannel(1);
  decoderTask -> SetMeshChannel(0);
  decoderTask -> SetTriggerChannel(2);
  decoderTask -> SetPersistence();
  run -> AddTask(decoderTask);


  run->Init();

  run->Run(0,10000000);

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
