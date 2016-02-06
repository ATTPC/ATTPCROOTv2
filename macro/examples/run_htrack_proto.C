/**************************************************************
		- Calculation of the Hough Space from unpacked data
***************************************************************/

void run_htrack_proto(){


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
  logger -> SetLogFileName("ATTPCLog_track.log");
  logger -> SetLogToFile(kTRUE);
  logger -> SetLogToScreen(kTRUE);
  logger -> SetLogVerbosityLevel("MEDIUM");


  FairRunAna* run = new FairRunAna();
  run -> SetInputFile("output_proto.root");
  run -> SetOutputFile("output_proto_track.root");

    TString file = "../parameters/AT.parameters.par";

   FairRuntimeDb* rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
   parIo1 -> open(file.Data(), "in");
   FairParRootFileIo* parIo2 = new FairParRootFileIo();
   parIo2 -> open("param.dummy_proto.root");
   rtdb -> setFirstInput(parIo2);
   rtdb -> setSecondInput(parIo1);

   ATHoughTask *HoughTask = new ATHoughTask();

   run -> AddTask(HoughTask);



   run->Init();

   run->Run(0, 1); // Number must be lower than the number of events in dummy

 // -----   Finish   -------------------------------------------------------
	timer.Stop();
	Double_t rtime = timer.RealTime();
	Double_t ctime = timer.CpuTime();
	cout << endl << endl;
	cout << "Tracking finished succesfully." << endl;
	cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
	cout << endl;
  // ------------------------------------------------------------------------


}
