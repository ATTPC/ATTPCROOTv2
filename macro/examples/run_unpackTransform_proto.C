/**************************************************************
    - Unpack of p-ATTPC data
    - Simple pulse shape analysis
		- Calculation of the Hough Space
***************************************************************/

void run_unpackTransform_proto(){


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

   FairLogger *logger = FairLogger::GetLogger();
  logger -> SetLogFileName("ATTPCLog.log");
  logger -> SetLogToFile(kTRUE);
  logger -> SetLogToScreen(kTRUE);
  logger -> SetLogVerbosityLevel("MEDIUM");


   FairRunAna* run = new FairRunAna();
  run -> SetInputFile("mc.dummy_proto.root");
  run -> SetOutputFile("output_proto.root");

    TString file = "../parameters/AT.parameters.par";

   FairRuntimeDb* rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
   parIo1 -> open(file.Data(), "in");
   FairParRootFileIo* parIo2 = new FairParRootFileIo();
   parIo2 -> open("param.dummy_proto.root");
   rtdb -> setFirstInput(parIo2);
   rtdb -> setSecondInput(parIo1);

   ATDecoderTask *decoderTask = new ATDecoderTask();
   //decoderTask ->SetDebugMode(kTRUE);
   decoderTask ->SetMapOpt(1); // ATTPC : 0  - Prototype: 1 |||| Default value = 0
   //decoderTask -> AddData("/home/ayyadlim/Desktop/Yassid/ATTPC/Data/Notre_Dame_data/CoBo_AsAd0_2015-01-27T15_19_34.962_0000.graw");//12B
   //decoderTask -> AddData("/Users/yassidayyad/Desktop/ATTPC/Data/Notre_Dame_data/CoBo_AsAd0_2015-01-27T15_19_34.962_0000.graw");
   //decoderTask ->AddData("/home/ayyadlim/Desktop/Yassid/ATTPC/Data/Notre_Dame_data/CoBo_AsAd0_2015-01-26T19_33_23.451_0003.graw"); //12N
    decoderTask -> AddData("/home/ayyadlim/Desktop/Yassid/ATTPC/Data/Notre_Dame_data/CoBo_AsAd0_2015-01-28T16:56:24.135_0000.graw");//12B Low Pressure
   decoderTask ->SetGeo(geo.Data());
   decoderTask ->SetProtoMap(protomapdir.Data());
   decoderTask ->SetMap(scriptdir.Data());
   decoderTask -> SetPositivePolarity(kTRUE);
   decoderTask -> SetFPNPedestal();
   decoderTask -> SetNumTbs(512);
   decoderTask -> SetPersistence();
   run -> AddTask(decoderTask);

   ATPSATask *psaTask = new ATPSATask();
   psaTask -> SetPersistence();
   psaTask -> SetThreshold(30);
   run -> AddTask(psaTask);

   ATHoughTask *HoughTask = new ATHoughTask();
   HoughTask ->SetPersistence();
   run -> AddTask(HoughTask);

   run->Init();

   run->Run(0, 10); // Number must be lower than the number of events in dummy

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
