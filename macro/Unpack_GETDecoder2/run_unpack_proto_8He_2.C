//void run_unpack_proto_8He_2(TString dataFile = "runfiles/run_ISAC2015_0259.txt",TString parameterFile = "pATTPC.TRIUMF2015.par"){
//void run_unpack_proto_8He_2(TString dataFile = "/data/TRIUMF/ISAC_2015/CoBo_AsAd0_2015-12-06T01:48:29.974_0000.graw",TString parameterFile = "pATTPC.TRIUMF2015.par"){
void run_unpack_proto_8He_2(TString dataFile = "/Users/Yassid/Desktop/ATTPC/Data/TRIUMF/run_0259/CoBo_AsAd0_2015-12-05T14_33_58.545_0000.graw",TString parameterFile = "pATTPC.TRIUMF2015.par"){

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
   logger -> SetLogFileName("ATTPCLog.log");
   logger -> SetLogToFile(kTRUE);
   logger -> SetLogToScreen(kTRUE);
   logger -> SetLogVerbosityLevel("MEDIUM");


   FairRunAna* run = new FairRunAna();
   run -> SetOutputFile("output_proto.root");
   //run -> SetGeomFile("../geometry/ATTPC_Proto_v1.0.root");

   TString paramterFileWithPath = paraDir + parameterFile;

   FairRuntimeDb* rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
   parIo1 -> open(paramterFileWithPath.Data(), "in");
   //FairParRootFileIo* parIo2 = new FairParRootFileIo();
   //parIo2 -> open("param.dummy_proto.root");
  // rtdb -> setFirstInput(parIo2);
   rtdb -> setSecondInput(parIo1);

   ATDecoder2Task *decoderTask = new ATDecoder2Task();
   //decoderTask ->SetDebugMode(kTRUE);
   decoderTask ->SetMapOpt(1); // ATTPC : 0  - Prototype: 1 |||| Default value = 0

           if (dataFile.EndsWith(".txt")){
	          		std::ifstream listFile(dataFile.Data());
	          		TString dataFileWithPath;
										while (dataFileWithPath.ReadLine(listFile)) {
										decoderTask -> AddData(dataFileWithPath);
										}
					}else decoderTask -> AddData(dataFile.Data());

	 decoderTask ->SetGeo(geo.Data());
   decoderTask ->SetProtoMap(protomapdir.Data());
   decoderTask ->SetMap((Char_t const*) scriptdir.Data());
   //decoderTask -> SetPersistence();
   run -> AddTask(decoderTask);

   ATPSATask *psaTask = new ATPSATask();
   psaTask -> SetPersistence();
   psaTask -> SetBackGroundPeakFinder(kFALSE); // Suppress background of each pad for noisy data (Larger computing Time)
   psaTask -> SetThreshold(20);
   psaTask -> SetPeakFinder(); //Note: For the moment not affecting the prototype PSA Task
   run -> AddTask(psaTask);

    //Moved to analysis macro!
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

	 ATAnalysisTask *AnaTask = new ATAnalysisTask();
   AnaTask->SetPhiReco();
   AnaTask->SetHoughDist(2.0);
   AnaTask->SetPersistence(kTRUE);

   run->AddTask(AnaTask);

   run->Init();

   run->Run(0,10000);
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
