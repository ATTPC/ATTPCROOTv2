void run_unpack(){

   // -----   Timer   --------------------------------------------------------
	TStopwatch timer;
	timer.Start();
  // ------------------------------------------------------------------------

   gSystem->Load("libXMLParser.so");

   TString scriptfile = "Lookup20150611.xml";
   TString dir = getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/"+ scriptfile;


  FairLogger *logger = FairLogger::GetLogger();
  logger -> SetLogFileName("ATTPCLog.log");
  logger -> SetLogToFile(kTRUE);
  logger -> SetLogToScreen(kTRUE);
  logger -> SetLogVerbosityLevel("MEDIUM");

  FairRunAna* run = new FairRunAna();
  //run -> SetInputFile("mc.dummy.root");
  run -> SetOutputFile("output.root");
  run -> SetGeomFile("../../geometry/ATTPC_v1.2.root");

    TString file = "../../parameters/AT.parameters.par";

   FairRuntimeDb* rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
   parIo1 -> open(file.Data(), "in");
   //FairParRootFileIo* parIo2 = new FairParRootFileIo();
   //parIo2 -> open("param.dummy.root");
   //rtdb -> setFirstInput(parIo2);
   rtdb -> setSecondInput(parIo1);



  ATDecoderTask *decoderTask = new ATDecoderTask();
  decoderTask ->SetMap(scriptdir.Data());
  decoderTask ->SetMapOpt(0); // ATTPC : 0  - Prototype: 1 |||| Default value = 0
 // decoderTask ->SetMap("/home/daq/fair_install_2015/ATTPCROOT_09032015/scripts/Lookup20141208.xml");
  //decoderTask -> AddData("/home/daq/Desktop/Yassid/ATTPC/run_0225/test");
  //decoderTask ->SetMap("/Users/yassidayyad/fair_install/ATTPCROOT_Apr/scripts/Lookup20141208.xml");
  //decoderTask -> AddData("/Users/yassidayyad/Desktop/ATTPC/Data/run_0225/test");
  decoderTask -> AddData("/home/ayyadlim/Desktop/Yassid/ATTPC/Data/run_0225/test");
    //decoderTask -> AddData("/data/ar40/run_0122/run_0122.graw");
 // decoderTask -> AddData("/data/ar46/run_0085/run_0085.graw");
  decoderTask -> SetPositivePolarity(kTRUE);

  //decoderTask -> SetData(0);
  //decoderTask -> SetInternalPedestal(5, 20);
  decoderTask -> SetFPNPedestal(20);
  //decoderTask -> SetFPNPedestal();
  decoderTask -> SetNumTbs(512);
  decoderTask -> SetPersistence();
  run -> AddTask(decoderTask);



  ATPSATask *psaTask = new ATPSATask();
  psaTask -> SetPersistence();
  psaTask -> SetThreshold(20);
  psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC
	//psaTask -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
	psaTask -> SetMaxFinder(); //Much faster, obviously
  run -> AddTask(psaTask);

	ATHoughTask *HoughTask = new ATHoughTask();
	HoughTask ->SetPersistence();
	//HoughTask ->SetLinearHough();
	HoughTask ->SetCircularHough();
	run -> AddTask(HoughTask);

  run->Init();

  run->Run(0,10);
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
