void run_unpack_proto_v2(){


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
   //run -> SetInputFile("mc.dummy_proto.root");
   run -> SetOutputFile("output_proto.root");
   //run -> SetGeomFile("../geometry/ATTPC_Proto_v1.0.root");

   TString file = "../../parameters/AT.parameters.par";

   FairRuntimeDb* rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
   parIo1 -> open(file.Data(), "in");
   //FairParRootFileIo* parIo2 = new FairParRootFileIo();
   //parIo2 -> open("param.dummy_proto.root");
  // rtdb -> setFirstInput(parIo2);
   rtdb -> setSecondInput(parIo1);

   ATDecoderTask *decoderTask = new ATDecoderTask();
   //decoderTask ->SetDebugMode(kTRUE);
   decoderTask ->SetMapOpt(1); // ATTPC : 0  - Prototype: 1 |||| Default value = 0
   //decoderTask -> AddData("/home/ayyadlim/Desktop/Yassid/ATTPC/Data/Notre_Dame_data/CoBo_AsAd0_2015-01-27T15_19_34.962_0000.graw");//12B
   //decoderTask -> AddData("/home/ayyadlim/Desktop/Yassid/ATTPC/Data/Notre_Dame_data/CoBo_AsAd0_2015-01-27T07_16_00.696_0000.graw");
   //decoderTask -> AddData("/Users/yassidayyad/Desktop/ATTPC/Data/Notre_Dame_data/CoBo_AsAd0_2015-01-27T15_19_34.962_0000.graw");
   //decoderTask ->AddData("/home/ayyadlim/Desktop/Yassid/ATTPC/Data/Notre_Dame_data/CoBo_AsAd0_2015-01-26T19_33_23.451_0003.graw"); //12N
   //decoderTask -> AddData("/home/ayyadlim/Desktop/Yassid/ATTPC/Data/Notre_Dame_data/CoBo_AsAd0_2015-01-28T07:02:50.291_0000.graw");//12B High Pressure
   //decoderTask -> AddData("/home/ayyadlim/Desktop/Yassid/ATTPC/Data/Notre_Dame_data/CoBo_AsAd0_2015-01-28T16:56:24.135_0000.graw");//12B Low Pressure
   decoderTask -> AddData("/home/ayyadlim/Desktop/Yassid/ATTPC/Data/TRIUMF/CoBo_AsAd0_2015-12-01T07_35_27.482_0009.graw");//8He TRIUMF
	//  decoderTask -> AddData("/home/ayyadlim/Desktop/Yassid/ATTPC/Data/Notre_Dame_data/10Be/CoBo_2013-02-21_12-52-57_0006.graw"); //10Be
   //decoderTask -> AddData("/home/s1257/fair_install_ROOT6/data/CoBo_AsAd0_2015-12-03T05:47:43.571_0001.graw");//12B Low Pressure
   //decoderTask -> AddData("/home/daq/Desktop/Data/run_0014/CoBo_AsAd0_2015-07-29T15_45_17.971_0000.graw");
   //decoderTask->AddData("/home/daq/Desktop/Data/run_0028/CoBo_AsAd0_2015-07-29T19_02_32.783_0000.graw");
   decoderTask ->SetGeo(geo.Data());
   decoderTask ->SetProtoMap(protomapdir.Data());
   decoderTask ->SetMap((Char_t const*) scriptdir.Data());
   decoderTask -> SetPositivePolarity(kTRUE);
   decoderTask -> SetFPNPedestal(6);
   //decoderTask->SetInternalPedestal();
   decoderTask -> SetNumTbs(512);
   //decoderTask -> SetPersistence();
   decoderTask -> SetGetRawEventMode(1);
   run -> AddTask(decoderTask);

   ATPSATask *psaTask = new ATPSATask();
   psaTask -> SetPersistence();
   psaTask -> SetBackGroundPeakFinder(kFALSE); // Suppress background of each pad for noisy data (Larger computing Time)
   psaTask -> SetThreshold(20);
   psaTask -> SetPeakFinder(); //Note: For the moment not affecting the prototype PSA Task
   run -> AddTask(psaTask);

   ATPhiRecoTask *phirecoTask = new ATPhiRecoTask();
   phirecoTask -> SetPersistence();
   run -> AddTask(phirecoTask);

   /*ATHoughTask *HoughTask = new ATHoughTask();
   HoughTask->SetPhiReco();
   HoughTask->SetPersistence();
   HoughTask->SetLinearHough();
	 HoughTask->SetRadiusThreshold(3.0); // Truncate Hough Space Calculation
   //HoughTask ->SetCircularHough();
   run ->AddTask(HoughTask);*/

   run->Init();

   run->Run(0,100); // Number must be lower than the number of events in dummy

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
