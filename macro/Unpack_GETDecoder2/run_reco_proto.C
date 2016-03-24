void run_reco_proto(TString dataFile = "output_proto.root",TString parameterFile = "pATTPC.TRIUMF2015.par"){


    // -----   Timer   --------------------------------------------------------
	TStopwatch timer;
	timer.Start();
  // ------------------------------------------------------------------------


   gSystem->Load("libXMLParser.so");

   TString scriptfile = "LookupProto20150331.xml";
   TString protomapfile = "proto.map";
   TString dir = getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/"+ scriptfile;
   TString protomapdir = dir + "/scripts/"+ protomapfile;
   TString geo = "proto_geo_hires.root";
	 TString paraDir = dir + "/parameters/";

   FairLogger *logger = FairLogger::GetLogger();
   logger -> SetLogFileName("ATTPC_RecoLog.log");
   logger -> SetLogToFile(kTRUE);
   logger -> SetLogToScreen(kTRUE);
   logger -> SetLogVerbosityLevel("MEDIUM");


   FairRunAna* run = new FairRunAna();
	 run -> SetInputFile(dataFile.Data());
   run -> SetOutputFile("output_proto_reco.root");
   //run -> SetGeomFile("../geometry/ATTPC_Proto_v1.0.root");

   TString paramterFileWithPath = paraDir + parameterFile;

   FairRuntimeDb* rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
   parIo1 -> open(paramterFileWithPath.Data(), "in");
   //FairParRootFileIo* parIo2 = new FairParRootFileIo();
   //parIo2 -> open("param.dummy_proto.root");
  // rtdb -> setFirstInput(parIo2);
   rtdb -> setSecondInput(parIo1);


	 ATPhiRecoTask *phirecoTask = new ATPhiRecoTask();
   phirecoTask -> SetPersistence();
   run -> AddTask(phirecoTask);

   ATHoughTask *HoughTask = new ATHoughTask();
   HoughTask->SetPhiReco();
   HoughTask->SetPersistence();
   HoughTask->SetLinearHough();
	 HoughTask->SetRadiusThreshold(3.0); // Truncate Hough Space Calculation
   //HoughTask ->SetCircularHough();
   run ->AddTask(HoughTask);

   run->Init();

   run->Run(0,1000);
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
